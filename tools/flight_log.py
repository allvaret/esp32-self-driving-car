"""tools/flight_log.py - log local (JSONL) do S9: um arquivo por motivacao, escrito em thread propria.

Registros (uma linha JSON cada; w = relogio de parede em ms):
  {"k":"f","w":..,"t":..,"vs":"LIVRE","r":0.93,"sec":[..5..],"e":{"h":..,"c":..,"n":0},"ok":1,"g":"explorar","src":"visao","v":0.35,"s":0.02}
  {"k":"tx","w":..,"m":{...}}   tudo que o brain ENVIOU ao ESP32 (drive, vis, yaw, avoid, mode, think)
  {"k":"rx","w":..,"m":{...}}   TODA mensagem RECEBIDA do ESP32 (state, range, scan, log, goal, ...)
Nada e transmitido durante a corrida: os arquivos ficam em <filesDir>/logs e sao baixados depois pelo
visor (viz.py: /logs e /log/<nome>, na rede CARRINHO).

Nome: corrida_<data>_<hora>_<fonte>.jsonl (run_name), ex.: corrida_2026-11-26_13h01m18_sensor.jsonl (ate a 0.7.0:
explorar_<fonte>_<data>_<hora>.jsonl).
Um arquivo por goal: abre quando chega {"t":"goal","g":"explorar","src":..} (ou muda goal/src) e fecha em
"parar". Sem arquivo aberto nada e gravado (o carro parado nao enche o armazenamento).
Escrita: fila + thread propria; flush a cada FLUSH_S (1 s); nunca bloqueia o laco da camera.
Video (tools/video_log.py): com `video`, o arquivo de video da corrida abre e fecha JUNTO com o JSONL e tem o mesmo
nome (<nome>.mjpg + <nome>_indice.csv). O registro "f" leva "vf" = indice do quadro gravado no video (quando foi gravado)
e "mov" = indice de movimento da camera (vision/motion.py, o mesmo "m" do vis ao ESP32).
"""
import json
import os
import queue
import threading
import time

FLUSH_S = 1.0
MAX_BYTES = 64 * 1024 * 1024          # teto por arquivo (ao passar, grava um aviso e para de gravar nele)
QUEUE_MAX = 20000


def _wall_ms():
    return int(time.time() * 1000)


def run_name(goal, src, t=None):
    """Nome base da corrida: corrida_<data>_<hora>_<fonte> (ex.: corrida_2026-11-26_13h01m18_sensor). Data primeiro =
    a ordem alfabetica e a ordem das corridas. Todos os arquivos da corrida usam este nome: .jsonl (log), .mjpg
    (video), _indice.csv (indice do video), _visor.mjpg (painel anotado). Goal diferente de explorar vai no fim."""
    stamp = time.strftime("%Y-%m-%d_%Hh%Mm%S", time.localtime(t))
    name = f"corrida_{stamp}_{src or 'sensor'}"
    return name if goal in (None, "", "explorar") else f"{name}_{goal}"


class FlightLog:
    def __init__(self, directory, video=None):
        self.dir = str(directory)
        self.video = video            # tools.video_log.VideoLog (ou None: so o JSONL)
        os.makedirs(self.dir, exist_ok=True)
        self._q = queue.Queue(maxsize=QUEUE_MAX)
        self._thread = None
        self._f = None
        self._key = None              # (goal, src) do arquivo aberto
        self._bytes = 0
        self._full = False
        self.dropped = 0              # itens perdidos por fila cheia (deve ficar em 0)
        self.current = None           # nome do arquivo aberto

    # ---- API chamada pelo brain (qualquer thread; so enfileira) ----
    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name="flight-log")
            self._thread.start()
        return self

    def frame(self, p, goal, src, v, s):
        """p = brain._parse(...) ({"state","conf","ok","sectors","raw"})."""
        raw = p.get("raw") or {}
        self._put({"k": "f", "w": _wall_ms(), "t": raw.get("t"), "n": raw.get("n"), "vs": p.get("state"),
                   "r": raw.get("r"), "sec": p.get("sectors"), "ok": 1 if p.get("ok") else 0,
                   "e": {"h": raw.get("edge_h_cen"), "c": raw.get("edge_cov"), "n": 1 if raw.get("edge_near") else 0},
                   "ph": raw.get("phase"), "g": goal, "src": src, "v": round(v, 3), "s": round(s, 3),
                   "ms": raw.get("proc_ms"), "mov": raw.get("mov"), "vf": raw.get("vf")})

    def sent(self, obj):
        self._put({"k": "tx", "w": _wall_ms(), "m": obj})

    def recv(self, m):
        self._put({"k": "rx", "w": _wall_ms(), "m": m})

    def close(self):
        self._put(None)
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _put(self, item):
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    # ---- thread de escrita ----
    def _run(self):
        last_flush = time.monotonic()
        while True:
            try:
                item = self._q.get(timeout=0.25)
            except queue.Empty:
                item = False
            if item is None:
                break
            if item is not False:
                try:
                    self._write(item)
                except Exception:
                    pass                      # o log nunca derruba o cerebro
            if self._f is not None and time.monotonic() - last_flush >= FLUSH_S:
                try:
                    self._f.flush()
                except Exception:
                    pass
                last_flush = time.monotonic()
        self._close_file()

    def _open(self, goal, src):
        self._close_file()
        base = run_name(goal, src)
        name = base + ".jsonl"
        n = 1
        while os.path.exists(os.path.join(self.dir, name)):
            n += 1
            name = f"{base}_{n}.jsonl"
        self._f = open(os.path.join(self.dir, name), "w", encoding="utf-8")
        self._key, self._bytes, self._full, self.current = (goal, src), 0, False, name
        if self.video is not None:
            try:
                self.video.start(self.dir, name[:-len(".jsonl")])
            except Exception:
                pass                          # sem video a corrida segue (so o JSONL)

    def _close_file(self):
        if self.video is not None and self._f is not None:
            try:
                info = self.video.stop()
                if info:
                    self._line({"k": "info", "w": _wall_ms(), "video": info})
            except Exception:
                pass
        if self._f is not None:
            try:
                self._f.close()
            except Exception:
                pass
        self._f, self._key, self.current = None, None, None

    def _write(self, rec):
        m = rec.get("m")
        if rec["k"] == "rx" and isinstance(m, dict) and m.get("t") == "goal":
            g = m.get("g")
            if g == "explorar":
                key = (g, m.get("src") or "sensor")
                if key != self._key:
                    self._open(*key)
            elif self._f is not None:             # parar/nenhuma: grava esta mensagem e fecha o arquivo
                self._line(rec)
                self._close_file()
                return
        if self._f is None:
            return
        self._line(rec)

    def _line(self, rec):
        if self._full:
            return
        s = json.dumps(rec, separators=(",", ":"), default=str) + "\n"
        self._bytes += len(s)
        if self._bytes > MAX_BYTES:
            self._full = True
            s = json.dumps({"k": "info", "w": _wall_ms(), "msg": "arquivo cheio: parou de gravar"}) + "\n"
        self._f.write(s)


def list_runs(directory):
    """Corridas para a pagina /logs: [(stem, [(nome, bytes, tipo)])], mais novas primeiro. tipo: log | video | indice |
    anotado. Video sem JSONL (ex.: app caiu antes de criar o log) tambem aparece."""
    from tools import video_log as vl
    try:
        names = os.listdir(directory)
    except OSError:
        return []
    kinds = ((vl.VIZ_SUFFIX, "anotado"), (vl.INDEX_SUFFIX, "indice"), (vl.VIDEO_EXT, "video"), (".jsonl", "log"))
    runs = {}
    for n in names:
        for suf, kind in kinds:
            if n.endswith(suf):
                p = os.path.join(directory, n)
                runs.setdefault(n[:-len(suf)], []).append((n, os.path.getsize(p), kind, os.path.getmtime(p)))
                break
    order = {"log": 0, "video": 1, "indice": 2, "anotado": 3}
    out = [(stem, sorted(fs, key=lambda x: order[x[2]])) for stem, fs in runs.items()]
    out.sort(key=lambda r: max(f[3] for f in r[1]), reverse=True)
    return [(stem, [(n, sz, k) for n, sz, k, _ in fs]) for stem, fs in out]


DOWNLOAD_EXTS = (".jsonl", ".mjpg", ".csv")


def list_logs(directory):
    """[(nome, bytes)] dos .jsonl, mais novos primeiro."""
    try:
        files = [f for f in os.listdir(directory) if f.endswith(".jsonl")]
    except OSError:
        return []
    files.sort(key=lambda f: os.path.getmtime(os.path.join(directory, f)), reverse=True)
    return [(f, os.path.getsize(os.path.join(directory, f))) for f in files]


def safe_path(directory, name, exts=(".jsonl",)):
    """Caminho do arquivo `name` dentro de `directory` ou None (sem barras nem '..', so as extensoes `exts`, tem que
    existir). O viz.py passa DOWNLOAD_EXTS (log, video e indice)."""
    if (not name or name != os.path.basename(name) or "/" in name or "\\" in name or name.startswith(".")
            or not name.endswith(tuple(exts))):
        return None
    p = os.path.join(directory, name)
    return p if os.path.isfile(p) else None
