"""Compare saved before/after reference support without changing source poses."""
import json
from pathlib import Path
import cv2
from PIL import Image,ImageDraw

root=Path('runs');old=root/'oct18-solid-edge-ramp-001';new=root/'oct18-camera-edge-continuation-001'
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
out=root/'oct18-camera-edge-source-review-001';out.mkdir(exist_ok=True)
a=load(old/'ego-lane-state.json');b=load(new/'ego-lane-state.json')
summary={}
for name,d in [('before',a),('after',b)]:
    rows=[r for r in d['records'] if 8<=r['time_s']<=42]
    summary[name]={'samples':len(rows),'lane_zero_fraction':sum(r['lane_id']==0 for r in rows)/len(rows),
                   'unknown_samples':sum(r['lane_id'] is None for r in rows)}
summary['ego_lateral_samples_exactly_unchanged']=[(r['time_s'],r['road_lateral_m']) for r in a['records']]==[(r['time_s'],r['road_lateral_m']) for r in b['records']]
summary['render_poses_exactly_unchanged']=(old/'render-states.jsonl').read_bytes()==(new/'render-states.jsonl').read_bytes()
road=load(new/'report.json')['road'];summary['reference_line']=road['reference_line']
(out/'review.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
cap=cv2.VideoCapture(str(new/'surrounding-world.mp4'))
sheet=Image.new('RGB',(1280,1440),'#16181a');draw=ImageDraw.Draw(sheet)
for i,t in enumerate([8,18,20,26,30,40]):
    cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read()
    if not ok:raise RuntimeError(f'decode failure {t}')
    im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));im.thumbnail((640,450))
    x=(i%2)*640;y=(i//2)*480;sheet.paste(im,(x,y+25));draw.text((x+8,y+5),f'{t}s rendered scene + raw four cameras',fill='white')
cap.release();sheet.save(out/'contact.jpg')
print(json.dumps({k:v for k,v in summary.items() if k!='reference_line'}))
