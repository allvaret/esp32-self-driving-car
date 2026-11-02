"""ws_link.py - cliente WebSocket do carrinho (S9 ou PC conectado na rede CARRINHO).

Mesmo protocolo JSON da serial. Uso como biblioteca (cerebro no S9):

    link = WsLink(on_msg=print)          # ws://192.168.4.1:81/
    link.start()
    link.mode("phone"); link.arm(True)
    while ...:                            # laco do cerebro, ~10 Hz (watchdog do ESP32 = 500 ms)
        link.drive(0.3, 0.0)
        link.think("chao livre a frente", state="LIVRE", conf=0.8)

Teste pelo PC (conectado na rede CARRINHO):
    python tools/ws_link.py                         # so escuta
    python tools/ws_link.py --drive 0.3 0 --secs 2  # modo phone, arma, anda 2 s (RODAS NO AR)

Dependencia: pip install websocket-client   (no Chaquopy: pip { install("websocket-client") })
"""
import argparse
import json
import threading
import time

import websocket  # websocket-client

DEFAULT_URL = "ws://192.168.4.1:81/"


class WsLink:
    def __init__(self, url=DEFAULT_URL, on_msg=None):
        self.url = url
        self.on_msg = on_msg            # callback(dict) para cada mensagem recebida
        self.state = {}                 # ultimo {"t":"state"}
        self.ranges = {}                # deg -> (cm|None, t)
        self.goal = None                # ultima motivacao
        self.connected = False
        self._ws = None
        self._run = False
        self._lock = threading.Lock()

    # ---------------- conexao (thread com reconexao) ----------------
    def start(self):
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def stop(self):
        self._run = False
        if self._ws:
            self._ws.close()

    def _loop(self):
        while self._run:
            self._ws = websocket.WebSocketApp(
                self.url,
                on_open=lambda ws: self._set_conn(True),
                on_close=lambda ws, *a: self._set_conn(False),
                on_error=lambda ws, e: None,
                on_message=self._on_message,
            )
            self._ws.run_forever(ping_interval=5, ping_timeout=3)
            self._set_conn(False)
            if self._run:
                time.sleep(1.0)

    def _set_conn(self, ok):
        self.connected = ok

    def _on_message(self, ws, text):
        try:
            m = json.loads(text)
        except ValueError:
            return
        t = m.get("t")
        if t == "state":
            self.state = m
        elif t == "range":
            self.ranges[m.get("deg")] = (m.get("cm"), time.time())
        elif t == "goal":
            self.goal = m.get("g")
        if self.on_msg:
            self.on_msg(m)

    # ---------------- envio ----------------
    def send(self, obj):
        if not self.connected or self._ws is None:
            return False
        try:
            with self._lock:
                self._ws.send(json.dumps(obj, separators=(",", ":")))
            return True
        except Exception:
            return False

    def drive(self, v, s=0.0):
        return self.send({"t": "drive", "v": round(float(v), 3), "s": round(float(s), 3)})

    def aim(self, deg):
        return self.send({"t": "aim", "deg": round(float(deg), 1)})

    def arm(self, on=True):
        return self.send({"t": "arm", "on": bool(on)})

    def mode(self, m):
        return self.send({"t": "mode", "m": m})

    def cfg(self, **params):
        return self.send({"t": "cfg", **params})

    def think(self, msg, **extra):
        """Pensamento do cerebro -> aparece na pagina do visor. Mantenha curto (< ~300 bytes)."""
        return self.send({"t": "think", "msg": msg, **extra})


def _main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--drive", nargs=2, type=float, metavar=("V", "S"))
    ap.add_argument("--secs", type=float, default=2.0)
    a = ap.parse_args()

    def show(m):
        if m.get("t") == "state":
            print(f"state {m.get('mode')} armed={m.get('armed')} {m.get('st')}/{m.get('sub')} "
                  f"c={m.get('c')} L={m.get('L')} R={m.get('R')} why={m.get('why')!r}")
        elif m.get("t") != "range":
            print(m)

    link = WsLink(a.url, on_msg=show).start()
    t0 = time.time()
    while not link.connected:
        if time.time() - t0 > 10:
            raise SystemExit("sem conexao: o PC esta na rede CARRINHO?")
        time.sleep(0.1)
    print("conectado em", a.url)

    try:
        if a.drive:
            link.mode("phone")
            link.arm(True)
            time.sleep(3.3)                       # espera de armar
            t_end = time.time() + a.secs
            while time.time() < t_end:            # 10 Hz: alimenta o watchdog
                link.drive(*a.drive)
                time.sleep(0.1)
            link.drive(0, 0)
            link.arm(False)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        link.drive(0, 0)
        link.arm(False)
        time.sleep(0.2)
        link.stop()


if __name__ == "__main__":
    _main()