"""Export an existing Scene Lab result with deterministic GLB overlay."""
import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path
import cv2
import numpy as np
import imageio_ffmpeg
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.scene_preview import EgoTimeline, render_ego, overlay_ego


def export(directory, output):
    root, output = Path(directory), Path(output)
    if output.exists(): raise FileExistsError(output)
    manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    rows = [json.loads(line) for line in (root/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    if not rows: raise ValueError('Empty observations')
    timeline = EgoTimeline(rows, max_gap=1.5/float(manifest['sample_hz']))
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='ego-preview-',dir=output.parent) as directory:
        tmp = Path(directory)
        for i,state in enumerate(timeline.states):
            frame = cv2.imdecode(np.fromfile(str(root/f'frame_{i:05d}.jpg'),np.uint8),cv2.IMREAD_COLOR)
            if frame is None: raise ValueError(f'Missing frame {i}')
            data,t,spin = state
            composite = overlay_ego(frame,render_ego(data,time_s=t,spin=spin))
            ok,buffer=cv2.imencode('.png',composite)
            if not ok: raise RuntimeError('Image encoding failed')
            buffer.tofile(str(tmp/f'{i:05d}.png'))
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-framerate',str(manifest['sample_hz']),
            '-i',str(tmp/'%05d.png'),'-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(tmp/'preview.mp4')],
            check=True,timeout=120)
        (tmp/'preview.mp4').rename(output)
    return output


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory'); parser.add_argument('--output',required=True)
    args=parser.parse_args()
    print(export(args.directory,args.output))
