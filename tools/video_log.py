"""tools/video_log.py - video da corrida no S9, ao lado do JSONL do brain (tools/flight_log.py).

Formato: MJPEG CRU (<nome>.mjpg = JPEGs completos um atras do outro) + indice <nome>_indice.csv
  (quadro, w = relogio de parede em ms como no JSONL, t_s = s desde o inicio do video, offset, bytes).
Por que nao AVI/MP4: o cv2.VideoWriter do OpenCV 4.5.1 do Chaquopy nao e garantido no celular (sem ffmpeg; o AVI MJPG
embutido so escreve o indice e o numero de quadros no release(): se o app cair, o arquivo fica sem indice). O MJPEG cru
so precisa do cv2.imencode (o mesmo que o viz.py ja usa no S9) e cada quadro e um JPEG completo: se o app cair no meio,
tudo o que foi gravado ate ali continua legivel (no maximo o ultimo quadro fica cortado e e ignorado).
Assistir: VLC abre o .mjpg direto (taxa padrao do VLC, sem os tempos reais); no navegador: /play/<nome> do viz.py
(toca no ritmo gravado); no PC: python -m tools.video_to_mp4 <nome>.mjpg  -> MP4 com a taxa certa (serve ao replay).

Custo no laco da camera: push() so confere o relogio e poe a REFERENCIA do quadro numa fila curta (sem copia, sem
JPEG); fila cheia = quadro descartado (nunca bloqueia). O JPEG (~5 ms no S9) e a escrita sao na thread do gravador
(o OpenCV solta o GIL). RAM: no maximo queue_max quadros crus. Falha na gravacao nunca derruba o brain.
Ciclo de vida: o FlightLog chama start(pasta, nome) quando abre o JSONL da corrida (goal explorar) e stop() quando
fecha (parar, brain.stop no onPause do app). Taxa e teto em segundos (config.RecordConfig).
"""
import os
import queue
import threading
import time

import cv2

VIDEO_EXT = ".mjpg"
INDEX_SUFFIX = "_indice.csv"
VIZ_SUFFIX = "_visor.mjpg"
FLUSH_S = 1.0
CSV_HEADER = "quadro,w,t_s,offset,bytes\n"


def _wall_ms():
    return int(time.time() * 1000)


def _default_cfg():
    from config import RecordConfig
    return RecordConfig()


class VideoLog:
    def __init__(self, cfg=None, clock=time.monotonic):
        self.cfg = cfg or _default_cfg()
        self._clock = clock
        self._lock = threading.Lock()
        self._rec = None              # gravacao atual (dict) ou None
        self.dropped = 0              # quadros descartados por fila cheia (total)
        self.last = None              # resumo da ultima gravacao fechada

    def configure(self, cfg):
        if cfg is not None:
            self.cfg = cfg

    @property
    def recording(self):
        r = self._rec
        return r is not None and not r["full"]

    @property
    def current(self):
        r = self._rec
        return r["name"] if r is not None else None

    # ---------------- controle (thread do FlightLog) ----------------
    def start(self, directory, stem):
        """Abre <stem>.mjpg + <stem>_indice.csv em directory (fecha a gravacao anterior). False se desligado/falhou."""
        self.stop()
        if not self.cfg.enabled:
            return False
        try:
            os.makedirs(directory, exist_ok=True)
            rotate(directory, self.cfg.keep_n - 1, self.cfg.keep_mb * 1e6)   # abre espaco para o novo
            name = stem + VIDEO_EXT
            vf = open(os.path.join(directory, name), "wb")
            xf = open(os.path.join(directory, stem + INDEX_SUFFIX), "w", encoding="utf-8")
            xf.write(CSV_HEADER)
            af = open(os.path.join(directory, stem + VIZ_SUFFIX), "wb") if self.cfg.annotated else None
        except Exception:
            return False
        rec = {"name": name, "dir": directory, "vf": vf, "xf": xf, "af": af, "q": queue.Queue(maxsize=self.cfg.queue_max),
               "t0": self._clock(), "next": 0.0, "n": 0, "written": 0, "bytes": 0, "full": False, "why": None}
        rec["thread"] = threading.Thread(target=self._run, args=(rec,), daemon=True, name="video-log")
        with self._lock:
            self._rec = rec
        rec["thread"].start()
        return True

    def stop(self, timeout=2.0):
        """Fecha a gravacao atual (grava o que esta na fila). Seguro chamar sem gravacao."""
        with self._lock:
            rec, self._rec = self._rec, None
        if rec is None:
            return None
        try:
            rec["q"].put(None, timeout=timeout)
        except queue.Full:
            pass
        rec["thread"].join(timeout=timeout)
        self.last = {k: rec[k] for k in ("name", "n", "written", "bytes", "full", "why")}
        return self.last

    # ---------------- entrada (thread da camera / do viz) ----------------
    def push(self, frame_bgr, t=None):
        """Quadro ORIGINAL da camera (BGR, ja rotacionado; NAO pode ser alterado depois). Devolve o indice do quadro no
        video ou None (sem gravacao, fora da taxa, teto atingido ou fila cheia). Nunca bloqueia nem levanta excecao."""
        rec = self._rec
        if rec is None or rec["full"] or frame_bgr is None:
            return None
        try:
            now = self._clock() if t is None else float(t)
            el = now - rec["t0"]
            if el < rec["next"]:
                return None                                   # taxa: no maximo cfg.fps quadros por segundo
            if el > self.cfg.max_s:
                rec["full"], rec["why"] = True, "max_s"
                return None
            per = 1.0 / self.cfg.fps
            rec["next"] = max(rec["next"] + per, el + 0.5 * per)   # sem rajada depois de uma pausa
            idx = rec["n"]
            rec["q"].put_nowait(("f", idx, _wall_ms(), el, frame_bgr))
            rec["n"] = idx + 1
            return idx
        except queue.Full:
            self.dropped += 1
            return None
        except Exception:
            return None

    def push_annotated(self, jpeg_bytes):
        """JPEG pronto do painel do visor (viz.py) para <nome>_visor.mjpg (so com cfg.annotated)."""
        rec = self._rec
        if rec is None or rec["af"] is None or rec["full"]:
            return
        try:
            rec["q"].put_nowait(("a", jpeg_bytes))
        except queue.Full:
            self.dropped += 1

    # ---------------- thread do gravador ----------------
    def _run(self, rec):
        last_flush = time.monotonic()
        params = (cv2.IMWRITE_JPEG_QUALITY, int(self.cfg.jpeg_q))
        limit = self.cfg.max_mb * 1e6
        while True:
            try:
                item = rec["q"].get(timeout=0.25)
            except queue.Empty:
                item = False
            if item is None:
                break
            if item:
                try:
                    if item[0] == "f":
                        _, idx, w, el, frame = item
                        ok, enc = cv2.imencode(".jpg", frame, params)
                        if ok:
                            data = enc.tobytes()
                            if rec["bytes"] + len(data) > limit:
                                rec["full"], rec["why"] = True, "max_mb"
                                rec["xf"].write(f"# cheio ({self.cfg.max_mb:.0f} MB): parou de gravar\n")
                            else:
                                rec["xf"].write(f"{idx},{w},{el:.3f},{rec['bytes']},{len(data)}\n")
                                rec["vf"].write(data)
                                rec["bytes"] += len(data)
                                rec["written"] += 1
                    elif rec["af"] is not None:
                        rec["af"].write(item[1])
                except Exception:
                    pass                                      # gravacao nunca derruba o brain
            if time.monotonic() - last_flush >= FLUSH_S:
                last_flush = time.monotonic()
                for f in (rec["vf"], rec["xf"], rec["af"]):
                    try:
                        if f is not None:
                            f.flush()
                    except Exception:
                        pass
        for f in (rec["vf"], rec["xf"], rec["af"]):
            try:
                if f is not None:
                    f.close()
            except Exception:
                pass


# ---------------- arquivos ----------------
def _videos(directory):
    try:
        names = [f for f in os.listdir(directory) if f.endswith(VIDEO_EXT) and not f.endswith(VIZ_SUFFIX)]
    except OSError:
        return []
    return sorted(names, key=lambda f: os.path.getmtime(os.path.join(directory, f)))   # mais velho primeiro


def _group(directory, video):
    stem = video[:-len(VIDEO_EXT)]
    return [os.path.join(directory, n) for n in (video, stem + INDEX_SUFFIX, stem + VIZ_SUFFIX)]


def rotate(directory, keep_n, keep_bytes):
    """Apaga os videos mais velhos (com indice e trilha anotada) ate sobrarem keep_n e no maximo keep_bytes."""
    vids = _videos(directory)

    def size(v):
        return sum(os.path.getsize(p) for p in _group(directory, v) if os.path.exists(p))
    total = sum(size(v) for v in vids)
    removed = []
    while vids and (len(vids) > max(0, keep_n) or total > keep_bytes):
        v = vids.pop(0)
        total -= size(v)
        for p in _group(directory, v):
            try:
                os.remove(p)
            except OSError:
                pass
        removed.append(v)
    return removed


def read_index(path):
    """Linhas do <nome>_indice.csv: [(quadro, w_ms, t_s, offset, bytes)]. Linha cortada (crash) e ignorada."""
    rows = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                p = line.strip().split(",")
                if len(p) != 5 or not p[0].isdigit():
                    continue
                try:
                    rows.append((int(p[0]), int(p[1]), float(p[2]), int(p[3]), int(p[4])))
                except ValueError:
                    continue
    except OSError:
        pass
    return rows


def iter_jpegs(path, index=None):
    """JPEGs completos do .mjpg (bytes). Com o indice usa os offsets; sem ele procura os marcadores SOI/EOI.
    Um quadro cortado no fim (app caiu) e ignorado."""
    with open(path, "rb") as f:                          # em pedacos: nunca o arquivo inteiro na RAM
        if index:
            for _, _, _, off, n in index:
                f.seek(off)
                chunk = f.read(n)
                if len(chunk) == n and chunk[:2] == b"\xff\xd8" and chunk[-2:] == b"\xff\xd9":
                    yield chunk
            return
        buf = b""
        while True:
            more = f.read(1 << 20)
            if not more:
                return                                   # o que sobrou sem EOI = ultimo quadro cortado
            buf += more
            while True:
                s = buf.find(b"\xff\xd8")
                if s < 0:
                    buf = buf[-1:]
                    break
                e = buf.find(b"\xff\xd9", s + 2)
                if e < 0:
                    buf = buf[s:]
                    break
                yield buf[s:e + 2]
                buf = buf[e + 2:]


REC = VideoLog()                  # gravador do app (main.py empurra quadros; FlightLog liga/desliga)


def push(frame_bgr, t=None):
    return REC.push(frame_bgr, t)


def push_annotated(jpeg_bytes):
    REC.push_annotated(jpeg_bytes)


def configure(cfg):
    REC.configure(cfg)
