"""tools/android_entry.py - funcoes chamadas pelo app Android (Kotlin -> Python via Chaquopy).

Mantenha este arquivo fino: a logica fica em vision/, control/, main.py, brain.py e tools/bench_s9.py
(o laco offline de run_main esta em replay/offline.py; mocks so sao importados dentro das funcoes).
"""


def smoke():
    """Confere o ambiente: versoes, modulos do OpenCV e o que ele sabe de video."""
    import sys

    import cv2
    import numpy as np

    lines = [f"python {sys.version.split()[0]} | numpy {np.__version__} | cv2 {cv2.__version__}",
             f"cv2.VideoCapture existe: {'sim' if hasattr(cv2, 'VideoCapture') else 'NAO'}"]
    info = cv2.getBuildInformation().splitlines()
    # modulos compilados / indisponiveis (mostra se o videoio foi construido)
    for l in info:
        s = l.strip()
        if s.startswith(("To be built:", "Unavailable:", "Disabled:")):
            lines.append(s)
    # qualquer linha da build info que fale de video/ffmpeg/mediandk/gstreamer
    keys = ("video", "ffmpeg", "mediandk", "gstreamer")
    hits = [l.strip() for l in info if any(k in l.lower() for k in keys)]
    lines.append("Linhas de video na build info:" + ("" if hits else " (nenhuma)"))
    lines += hits[:12]
    ok, buf = cv2.imencode(".jpg", np.zeros((8, 8, 3), np.uint8))
    lines.append(f"imencode/imdecode jpg: {'ok' if ok and cv2.imdecode(buf, 1) is not None else 'FALHOU'}")
    return "\n".join(lines)


def bench(path, threads=0, use_edges=True, max_frames=150):
    """path: arquivo de video OU pasta de frames. threads=0 -> padrao do OpenCV."""
    from tools import bench_s9
    return bench_s9.bench_text(path, max_frames=int(max_frames), use_edges=bool(use_edges),
                               threads=(int(threads) or None))


def run_main(path, use_edges=True, max_frames=300):
    """Bancada: 'for com frames' completo (visao + Controller v1 + sensor MOCK); devolve o resumo.
    NAO e o caminho ao vivo (la o ESP32 e o executor)."""
    from replay import offline          # import preguicoso: so a bancada usa (Controller + sensor MOCK)
    return offline.run_text(path, use_edges=bool(use_edges), max_frames=int(max_frames))