"""Saved full-frame before/after visual review sheet; no numeric accuracy claim."""
import argparse
from pathlib import Path
import cv2
from PIL import Image,ImageDraw

ap=argparse.ArgumentParser();ap.add_argument('--before',type=Path,required=True);ap.add_argument('--after',type=Path,required=True);ap.add_argument('--times',nargs='+',type=float,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
if a.output.exists():raise FileExistsError(a.output)
caps=[cv2.VideoCapture(str(p/'surrounding-world.mp4')) for p in [a.before,a.after]]
sheet=Image.new('RGB',(1280,525*len(a.times)),'#16181a');draw=ImageDraw.Draw(sheet)
for row,t in enumerate(a.times):
    for col,cap in enumerate(caps):
        cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read()
        if not ok:raise ValueError(f'Decode failed {col}/{t}')
        image=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)).resize((640,500));sheet.paste(image,(col*640,row*525+25))
        draw.text((col*640+8,row*525+5),('BEFORE' if col==0 else 'AFTER')+f' {t:.2f}s',fill='white')
for cap in caps:cap.release()
a.output.parent.mkdir(parents=True,exist_ok=True);sheet.save(a.output)
