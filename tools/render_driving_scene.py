"""Render a labeled driving-scene demo or observed result, without modifying inputs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2
import imageio_ffmpeg
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.driving_scene import DrivingRenderer, demo_scene
from core.scene_fusion import FusedSceneTimeline


def export(output, result=None, camera='all', fps=20):
    output=Path(output)
    if output.exists(): raise FileExistsError(output)
    if fps<=0: raise ValueError('FPS must be positive')
    timeline=None
    start,end=0.,10.
    if result:
        root=Path(result)
        manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
        rows=[json.loads(line) for line in (root/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
        if not rows: raise ValueError('Empty observations')
        meta=manifest.get('cameras',{}).get(camera,{})
        if camera!='all': rows=[dict(row,cameras={camera:row.get('cameras',{}).get(camera,{'available':False,'objects':[]})}) for row in rows]
        timeline=FusedSceneTimeline(rows,manifest)
        start,end=rows[0]['time_s'],rows[-1]['time_s']
    output.parent.mkdir(parents=True,exist_ok=True)
    renderer=DrivingRenderer()
    with tempfile.TemporaryDirectory(prefix='driving-preview-',dir=output.parent) as directory:
        tmp=Path(directory)
        count=max(1,int(round((end-start)*fps))+1)
        for i in range(count):
            t=min(end,start+i/fps)
            scene=timeline.sample(t) if timeline else demo_scene(t)
            frame=renderer.render(scene)
            ok,buffer=cv2.imencode('.png',frame)
            if not ok: raise RuntimeError('Frame encoding failed')
            buffer.tofile(str(tmp/f'{i:05d}.png'))
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-framerate',str(fps),
            '-i',str(tmp/'%05d.png'),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(tmp/'scene.mp4')],check=True,timeout=120)
        (tmp/'scene.mp4').rename(output)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--demo',action='store_true')
    source.add_argument('--result',type=Path)
    parser.add_argument('--camera',choices=['all','front','back','left_repeater','right_repeater','left_pillar','right_pillar'],default='all')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(export(args.output,args.result,args.camera))
