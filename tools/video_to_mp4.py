"""tools/video_to_mp4.py - converte o video da corrida do S9 (<nome>.mjpg + <nome>_indice.csv) em MP4/AVI no PC.

    python -m tools.video_to_mp4 docs/logs/corrida_2026-11-26_13h01m18_sensor.jsonl   # -> ..._sensor.mp4 (10 fps)
    python -m tools.video_to_mp4 docs/logs/corrida_2026-11-26_13h01m18_sensor          # o mesmo (nome sem extensao)
    python -m tools.video_to_mp4 docs/logs                                              # TODAS as corridas da pasta sem .mp4
    python -m tools.video_to_mp4 x.mjpg --fps 15 --out x.mp4
    python -m tools.video_to_mp4 x.mjpg --avi                                           # AVI MJPG
    python -m replay.replay_main docs/logs/corrida_2026-11-26_13h01m18_sensor.mp4 --out run_x   # replay da corrida
Aceita o .mjpg, o .jsonl ou o _indice.csv da corrida (ou o nome sem extensao): acha o video pelo nome.

A saida tem TAXA CONSTANTE (o replay usa quadro/fps como tempo): cada instante k/fps recebe o ultimo quadro gravado
ate ali pelo t_s do indice (quadro repetido quando o celular pulou, descartado quando sobrou). Sem o .csv (perdido)
usa os quadros na ordem a --fps. Arquivo cortado (app caiu) converte ate o ultimo quadro inteiro.
Codec: com ffmpeg no PATH, H.264 (toca no navegador e no VLC); sem ele, mp4v do OpenCV (VLC e o replay leem; o
navegador nao). --avi grava MJPG (os JPEGs do celular sao recodificados pelo OpenCV, qualidade 95).
Sincronia com o JSONL: o quadro k do video e o registro "f" com "vf": k; o .csv da o w (relogio de parede, ms) de cada um.
"""
import argparse
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np

from tools import video_log


def schedule(times, fps):
    """Indices dos quadros gravados para cada instante k/fps (taxa constante). times = t_s de cada quadro (crescente)."""
    if not times:
        return []
    t0, out, j = times[0], [], 0
    n = int((times[-1] - t0) * fps) + 1
    for k in range(n):
        tk = t0 + k / fps + 1e-9
        while j + 1 < len(times) and times[j + 1] <= tk:
            j += 1
        out.append(j)
    return out


def video_path(arg):
    """O .mjpg da corrida a partir do .mjpg, .jsonl, _indice.csv ou do nome sem extensao."""
    for suf in (video_log.VIZ_SUFFIX, video_log.INDEX_SUFFIX, video_log.VIDEO_EXT, ".jsonl", ".mp4", ".avi"):
        if arg.endswith(suf):
            if suf == video_log.VIZ_SUFFIX:
                return arg                        # trilha anotada: converte ela mesma
            arg = arg[:-len(suf)]
            break
    p = arg + video_log.VIDEO_EXT
    if not os.path.exists(p):
        raise SystemExit(f"nao achei o video {p} (o app gravou? record.enabled; corridas antes da 0.7.0 nao tem video)")
    return p


def pending(folder):
    """Videos da pasta que ainda nao tem .mp4 ao lado (mais velhos primeiro)."""
    names = sorted(f for f in os.listdir(folder) if f.endswith(video_log.VIDEO_EXT) and not f.endswith(video_log.VIZ_SUFFIX))
    return [os.path.join(folder, f) for f in names
            if not os.path.exists(os.path.join(folder, f[:-len(video_log.VIDEO_EXT)] + ".mp4"))]


def convert(src, out=None, fps=10.0, avi=False, use_ffmpeg=True):
    src = video_path(src)
    idx_path = src[:-len(video_log.VIDEO_EXT)] + video_log.INDEX_SUFFIX if not src.endswith(video_log.VIZ_SUFFIX) else None
    index = video_log.read_index(idx_path) if idx_path and os.path.exists(idx_path) else []
    jpegs = list(video_log.iter_jpegs(src, index or None))
    if not jpegs:
        raise SystemExit(f"nenhum quadro em {src}")
    times = [r[2] for r in index][:len(jpegs)] if index else [i / fps for i in range(len(jpegs))]
    order = schedule(times, fps)
    out = out or os.path.splitext(src)[0] + (".avi" if avi else ".mp4")
    ff = shutil.which("ffmpeg") if use_ffmpeg and not avi else None
    if ff:                                           # os JPEGs vao direto ao ffmpeg (sem decodificar aqui)
        p = subprocess.Popen([ff, "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", str(fps), "-c:v", "mjpeg",
                              "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
        for j in order:
            p.stdin.write(jpegs[j])
        p.stdin.close()
        if p.wait() != 0:
            raise SystemExit("ffmpeg falhou")
    else:
        first = cv2.imdecode(np.frombuffer(jpegs[0], np.uint8), cv2.IMREAD_COLOR)
        h, w = first.shape[:2]
        wr = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*("MJPG" if avi else "mp4v")), fps, (w, h))
        if not wr.isOpened():
            raise SystemExit(f"OpenCV nao abriu o writer para {out}")
        if avi:
            wr.set(cv2.VIDEOWRITER_PROP_QUALITY, 95)
        last_j, last_img = -1, None
        for j in order:
            if j != last_j:
                last_img = cv2.imdecode(np.frombuffer(jpegs[j], np.uint8), cv2.IMREAD_COLOR)
                last_j = j
            wr.write(last_img)
        wr.release()
    dur = len(order) / fps
    print(f"{out}: {len(order)} quadros a {fps:g} fps ({dur:.1f} s) de {len(jpegs)} gravados"
          f"{'' if index else ' (sem indice: ordem simples)'}{' [ffmpeg H.264]' if ff else ''}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("src", help="a corrida (.mjpg, .jsonl ou nome sem extensao) ou uma PASTA (converte as que faltam)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--fps", type=float, default=10.0, help="taxa do arquivo de saida (padrao = config.RecordConfig.fps)")
    ap.add_argument("--avi", action="store_true", help="AVI MJPG em vez de MP4")
    ap.add_argument("--no-ffmpeg", action="store_true", help="usa o OpenCV mesmo com ffmpeg no PATH")
    a = ap.parse_args(argv)
    if os.path.isdir(a.src):
        todo = pending(a.src)
        if not todo:
            print(f"nenhum video novo em {a.src}")
        for p in todo:
            convert(p, None, a.fps, a.avi, not a.no_ffmpeg)
        return 0
    convert(a.src, a.out, a.fps, a.avi, not a.no_ffmpeg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
