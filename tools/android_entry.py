"""tools/android_entry.py - funcoes chamadas pelo app Android (Kotlin -> Python via Chaquopy).

Mantenha este arquivo fino: a logica fica em vision/, control/, hardware/ e tools/bench_s9.py.
"""


def smoke():
    """Confere o ambiente: versoes dos pacotes e o que o OpenCV tem de E/S de video."""
    import sys

    import cv2
    import numpy as np

    info = cv2.getBuildInformation().splitlines()
    video_io = []
    for i, line in enumerate(info):
        if line.strip() == "Video I/O:":
            base = len(line) - len(line.lstrip())
            for sub in info[i + 1:i + 12]:
                if len(sub) - len(sub.lstrip()) <= base:
                    break
                video_io.append(sub.strip())
            break
    lines = [f"python {sys.version.split()[0]} | numpy {np.__version__} | cv2 {cv2.__version__}",
             "Video I/O do OpenCV:"] + (video_io or ["(nao encontrei a secao na build info)"])
    ok, buf = cv2.imencode(".jpg", np.zeros((8, 8, 3), np.uint8))
    lines.append(f"imencode/imdecode jpg: {'ok' if ok and cv2.imdecode(buf, 1) is not None else 'FALHOU'}")
    return "\n".join(lines)


def bench(path, threads=0, use_edges=True, max_frames=150):
    """path: arquivo de video OU pasta de frames. threads=0 -> padrao do OpenCV."""
    from tools import bench_s9
    return bench_s9.bench_text(path, max_frames=int(max_frames), use_edges=bool(use_edges),
                               threads=(int(threads) or None))