"""viz.py - mostra a "visao" do robo (frame, retroprojecao, mascara binaria) no visor web.

O S9 serve um MJPEG em http://<ip-do-S9>:8080/stream. A pagina do ESP32 recebe esse endereco
pelo brain (mensagem think com campo "viz") e mostra num <img>. A imagem NAO passa pelo ESP32.

Uso (dentro do StreamRunner.process_rgba_frame, depois de calcular a mascara):
    import viz
    viz.push(frame=bgr, prob=backproj, mask=mask)      # qualquer subconjunto; nomes livres
O brain.start() ja chama viz.start(). Custo: so codifica a cada 1/FPS s (~1-2 ms no S9).
"""
import html
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PORT = 8080
FPS = 5             # quadros por segundo enviados ao visor
TILE_H = 120        # altura de cada painel (px)
JPEG_Q = 60

_jpeg = None
_cond = threading.Condition()
_last_push = 0.0
_server = None
_log_dir = None


def _tile(img, label):
    a = np.asarray(img)
    if a.dtype == bool:
        a = a.astype(np.uint8) * 255
    elif a.dtype != np.uint8:
        a = a.astype(np.float32)
        mx = float(a.max()) if a.size else 0.0
        a = (a * (255.0 / mx) if mx > 0 else a * 0).astype(np.uint8)
    elif a.ndim == 2 and a.max() <= 1:                 # mascara 0/1
        a = a * 255
    if a.ndim == 2:
        a = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    elif a.shape[2] == 4:
        a = cv2.cvtColor(a, cv2.COLOR_RGBA2BGR)
    h, w = a.shape[:2]
    a = cv2.resize(a, (max(1, int(w * TILE_H / h)), TILE_H), interpolation=cv2.INTER_NEAREST)
    cv2.putText(a, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    return a


def push(**layers):
    """Recebe paineis nomeados (BGR, cinza, float ou bool). Ignora chamadas acima de FPS."""
    global _jpeg, _last_push
    now = time.time()
    if _server is None or now - _last_push < 1.0 / FPS:
        return
    _last_push = now
    try:
        tiles = [_tile(v, k) for k, v in layers.items() if v is not None]
        if not tiles:
            return
        ok, enc = cv2.imencode(".jpg", np.hstack(tiles), (cv2.IMWRITE_JPEG_QUALITY, JPEG_Q))
        if ok:
            with _cond:
                _jpeg = enc.tobytes()
                _cond.notify_all()
    except Exception:
        pass                                           # visualizacao nunca derruba o cerebro


def set_log_dir(path):
    """Pasta dos logs JSONL do brain (rotas /logs e /log/<nome>)."""
    global _log_dir
    _log_dir = path


class _H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _logs(self):
        from tools import flight_log
        rows = flight_log.list_logs(_log_dir) if _log_dir else []
        body = ["<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
                "<title>Logs do carrinho</title><body style='font:16px system-ui;padding:12px'><h3>Logs do celular</h3>"]
        body += [f"<p><a href='/log/{html.escape(n)}' download>{html.escape(n)}</a> ({sz // 1024} KB)</p>" for n, sz in rows]
        if not rows:
            body.append("<p>nenhum log ainda</p>")
        data = "".join(body).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _log(self, name):
        from tools import flight_log
        p = flight_log.safe_path(_log_dir, name) if _log_dir else None
        if p is None:
            self.send_error(404)
            return
        size = os.path.getsize(p)
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Content-Disposition", 'attachment; filename="%s"' % name)
        self.send_header("Content-Length", str(size))
        self.end_headers()
        with open(p, "rb") as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/logs":
            return self._logs()
        if path.startswith("/log/"):
            from urllib.parse import unquote
            return self._log(unquote(path[5:]))
        if not self.path.startswith("/stream"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        last = None
        try:
            while True:
                with _cond:
                    _cond.wait_for(lambda: _jpeg is not None and _jpeg is not last, timeout=2.0)
                    data = _jpeg
                if data is None or data is last:
                    continue
                last = data
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def start(port=PORT):
    global _server
    if _server is not None:
        return
    try:
        _server = ThreadingHTTPServer(("0.0.0.0", port), _H)
        _server.daemon_threads = True
        threading.Thread(target=_server.serve_forever, daemon=True).start()
    except OSError:
        _server = None                                 # porta ocupada: segue sem visualizacao


def stop():
    global _server
    if _server is not None:
        _server.shutdown()
        _server.server_close()
        _server = None


def url(peer="192.168.4.1"):
    """Endereco do stream visto pela rede do carro (IP local usado para chegar ao ESP32)."""
    if _server is None:
        return None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((peer, 80))
        ip = s.getsockname()[0]
        s.close()
        return f"http://{ip}:{_server.server_address[1]}/stream"
    except OSError:
        return None