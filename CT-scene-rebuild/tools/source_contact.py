"""Raw frame overview, selected by per-camera request time, not sensor sync."""
from pathlib import Path
import argparse
import json
from PIL import Image, ImageDraw

parser=argparse.ArgumentParser()
parser.add_argument('package',type=Path)
parser.add_argument('output',type=Path)
args=parser.parse_args()
frames=json.loads((args.package/'frames.json').read_text(encoding='utf-8'))
cameras=['front','back','left_repeater','right_repeater']
canvas=Image.new('RGB',(4*384,3*316),(30,30,30));draw=ImageDraw.Draw(canvas)
for j,t in enumerate([2,15,33]):
    for k,camera in enumerate(cameras):
        candidates=[f for f in frames if f['camera_id']==camera]
        if not candidates:continue
        f=min(candidates,key=lambda f:abs(f['relative_request_s']-t))
        im=Image.open(args.package/f['image']);im.thumbnail((384,288))
        x,y=k*384,j*316
        canvas.paste(im,(x+(384-im.width)//2,y+26+(288-im.height)//2))
        draw.text((x+3,y+5),f"{camera} rel {f['relative_request_s']:.1f}s PTS {f['pts']}",fill='white')
canvas.save(args.output)
