"""Full rendered frames including original-camera strip at explicit event times."""
import argparse,math,json
from pathlib import Path
import cv2
from PIL import Image,ImageDraw

ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--times',nargs='+',type=float,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
if a.output.exists():raise FileExistsError(a.output)
cap=cv2.VideoCapture(str(a.scene/'surrounding-world.mp4'));sheet=Image.new('RGB',(1536,628*math.ceil(len(a.times)/2)),'#171b20');d=ImageDraw.Draw(sheet)
source_times=[json.loads(line)['time_s'] for line in (a.scene/'render-states.jsonl').read_text().splitlines()]
for i,t in enumerate(a.times):
    index=min(range(len(source_times)),key=lambda i:abs(source_times[i]-t))
    if abs(source_times[index]-t)>.26:raise ValueError('Requested source time not rendered: '+str(t))
    cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,frame=cap.read()
    if not ok:raise ValueError('Missing frame '+str(t))
    x=i%2*768;y=i//2*628;sheet.paste(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)).resize((768,600)),(x,y+28));d.text((x+5,y+5),f'{source_times[index]:.2f}s source time, raw-camera strip included',fill='white')
cap.release();a.output.parent.mkdir(parents=True,exist_ok=True);sheet.save(a.output)
