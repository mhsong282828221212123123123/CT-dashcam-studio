"""Prepare source-only manual labels; never convert estimated XYZ into targets."""
import json,argparse,hashlib
from pathlib import Path
from collections import defaultdict
import cv2,numpy as np
from PIL import Image,ImageDraw
from ct_scene.artifacts import verify
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--cases',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
index={}
for depth in ['*/artifact.json','*/*/artifact.json','*/*/*/artifact.json','*/*/*/*/artifact.json']:
 for f in Path('runs').glob(depth):
  m=json.loads(f.read_text());
  if 'artifact_id' in m:index.setdefault(m['artifact_id'],f.parent)
cases=[('oct02','train','runs/final-oct02-015'),('oct18turn','train','runs/final-oct18-junction-015'),('oct18ramp','train','runs/fresh-oct18-ramp-005'),('aug20','validation','runs/final-aug20-014'),('nov01','test','runs/fresh-nov01-highway-003'),('nov01city','test','runs/fresh-nov01-city-road-002')]
if a.cases:cases=json.loads(a.cases.read_text())
records=[]
for case,split,scene in cases:
 m=verify(Path(scene));roles={r['role']:r['artifact_id'] for r in m['inputs']};source=index[roles['source']];tracks=index[roles['tracks']];frames={f['frame_id']:f for f in json.loads((source/'frames.json').read_text())};groups=defaultdict(list)
 for r in map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()):
  b=r['bbox_xyxy'];f=frames[r['frame_id']]
  if r['camera_id'] not in ['front','left_repeater','right_repeater'] or r['support']!='detector_observation' or r['class_id'] not in [2,5,7]:continue
  if r.get('score',0)<.7 or min(b[0],b[1],f['width']-b[2],f['height']-b[3])<8 or b[2]-b[0]<85 or b[3]-b[1]<55:continue
  groups[(r['camera_id'],r['local_track_id'])].append(r)
 selected=[]
 for camera in ['front','left_repeater','right_repeater']:
  candidates=[]
  for (cam,track),rows in groups.items():
   if cam!=camera:continue
   rows.sort(key=lambda r:(r['bbox_xyxy'][2]-r['bbox_xyxy'][0])*(r['bbox_xyxy'][3]-r['bbox_xyxy'][1]))
   candidates.append(rows[int((len(rows)-1)*.65)])
  candidates.sort(key=lambda r:r['camera_time_s'])
  if len(candidates)>4:candidates=[candidates[i] for i in np.linspace(0,len(candidates)-1,4).round().astype(int)]
  selected+=candidates
 sheet=Image.new('RGB',(1440,270*int(np.ceil(len(selected)/4))),'#20252a');draw=ImageDraw.Draw(sheet)
 for r in selected:
  f=frames[r['frame_id']];im=cv2.imread(str(source/f['image']));b=np.array(r['bbox_xyxy']);w,h=b[2:]-b[:2];rect=np.array([b[0]-.12*w,b[1]-.12*h,b[2]+.12*w,b[3]+.16*h]).round().astype(int);rect[[0,2]]=rect[[0,2]].clip(0,f['width']);rect[[1,3]]=rect[[1,3]].clip(0,f['height']);x,y,X,Y=rect
  crop=im[y:Y,x:X];scale=min(360/(X-x),230/(Y-y));cw,ch=round((X-x)*scale),round((Y-y)*scale);dx=(360-cw)//2;dy=(230-ch)//2
  uid=f'{case}-{len(records):03d}';path=a.output/(uid+'.jpg');cv2.imwrite(str(path),crop)
  tile=Image.new('RGB',(360,230),'#111');tile.paste(Image.fromarray(cv2.cvtColor(cv2.resize(crop,(cw,ch)),cv2.COLOR_BGR2RGB)),(dx,dy));i=selected.index(r);ox=(i%4)*360;oy=(i//4)*270;sheet.paste(tile,(ox,oy+35));draw.text((ox+5,oy+5),uid+' '+r['camera_id']+' '+str(round(r['camera_time_s'],1))+'s',fill='white')
  records.append(dict(id=uid,split=split,session='2026-10-18' if case.startswith('oct18') else ('2026-11-01' if case.startswith('nov01') else case),camera=r['camera_id'],source_artifact_id=roles['source'],tracks_artifact_id=roles['tracks'],frame_id=r['frame_id'],local_track_id=r['local_track_id'],source_image=str((source/f['image']).resolve()),source_image_sha256=hashlib.sha256((source/f['image']).read_bytes()).hexdigest(),crop=str(path.resolve()),crop_rect_xyxy=rect.tolist(),bbox_xyxy=b.tolist(),tile_transform=dict(scale=scale,dx=dx,dy=dy),contact_tile_xy=None,direction=None,annotation_status='unreviewed'))
 sheet.save(a.output/(case+'-sheet.jpg'))
(a.output/'manifest.json').write_text(json.dumps(dict(records=records,coordinate_target='visible_tire_contact_midpoint; image_only',directions=['away','toward','left','right'],metric_xyz_labels=False),indent=2));print('Prepared',len(records),'unlabelled source crops')
