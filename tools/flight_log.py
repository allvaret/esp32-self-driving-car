"""tools/flight_log.py - log local (JSONL) do S9: um arquivo por motivacao, escrito em thread propria.

Registros (uma linha JSON cada; w = relogio de parede em ms):
  {"k":"f","w":..,"t":..,"vs":"LIVRE","r":0.93,"sec":[..5..],"e":{"h":..,"c":..,"n":0},"ok":1,"g":"explorar","src":"visao","v":0.35,"s":0.02}
  {"k":"tx","w":..,"m":{...}}   tudo que o brain ENVIOU ao ESP32 (drive, vis, yaw, avoid, mode, think)
  {"k":"rx","w":..,"m":{...}}   TODA mensagem RECEBIDA do ESP32 (state, range, scan, log, goal, ...)
Nada e transmitido durante a corrida: os arquivos ficam em <filesDir>/logs e sao baixados depois pelo
visor (viz.py: /logs e /log/<nome>, na rede CARRINHO).

Um arquivo por goal: abre quando chega {"t":"goal","g":"explorar","src":..} (ou muda goal/src) e fecha em
"parar". Sem arquivo aberto nada e gravado (o carro parado nao enche o armazenamento).
Escrita: fila + thread propria; flush a cada FLUSH_S (1 s); nunca bloqueia o laco da camera.
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


class FlightLog:
    def __init__(self, directory):
        self.dir = str(directory)
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
                   "ms": raw.get("proc_ms")})

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
        stamp = time.strftime("%Y%m%d_%H%M%S")
        name = f"{goal}_{src or 'sensor'}_{stamp}.jsonl"
        n = 1
        while os.path.exists(os.path.join(self.dir, name)):
            n += 1
            name = f"{goal}_{src or 'sensor'}_{stamp}_{n}.jsonl"
        self._f = open(os.path.join(self.dir, name), "w", encoding="utf-8")
        self._key, self._bytes, self._full, self.current = (goal, src), 0, False, name

    def _close_file(self):
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


def list_logs(directory):
    """[(nome, bytes)] dos .jsonl, mais novos primeiro."""
    try:
        files = [f for f in os.listdir(directory) if f.endswith(".jsonl")]
    except OSError:
        return []
    files.sort(key=lambda f: os.path.getmtime(os.path.join(directory, f)), reverse=True)
    return [(f, os.path.getsize(os.path.join(directory, f))) for f in files]


def safe_path(directory, name):
    """Caminho do log `name` dentro de `directory` ou None (sem barras, so .jsonl, tem que existir)."""
    if not name or name != os.path.basename(name) or name.startswith(".") or not name.endswith(".jsonl"):
        return None
    p = os.path.join(directory, name)
    return p if os.path.isfile(p) else None
