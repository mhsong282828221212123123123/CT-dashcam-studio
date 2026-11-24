import argparse,json
from pathlib import Path
from PIL import Image,ImageDraw
from ct_scene.surrounding_lamps import extract

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'));frames=load(a.case/'source/frames.json');actors=load(a.scene/'actors.json');membership={k:x['global_id'] for x in actors for k in x['local_ids']}
observations=[json.loads(l) for l in (a.case/'tracks/observations.jsonl').read_text().splitlines()];r=extract(a.case/'source',frames,observations,membership)
(a.output/'review.json').write_text(json.dumps(r,indent=2),encoding='utf-8');by={f['frame_id']:f for f in frames};chosen=[]
for k,rows in r.items():
    if rows:chosen.append((k,rows[len(rows)//2]))
sheet=Image.new('RGB',(1200,max(1,(len(chosen)+3)//4)*250),'#161b1f');d=ImageDraw.Draw(sheet)
for i,(key,row) in enumerate(chosen):
    im=Image.open(a.case/'source'/by[row['frame_id']]['image']);box=row['bbox_xyxy'];crop=im.crop(tuple(box));crop.thumbnail((296,215));x=(i%4)*300;y=(i//4)*250;sheet.paste(crop,(x,y+30));d.text((x+3,y+5),key+' '+str(round(row['time_s'],2)),fill='white')
sheet.save(a.output/'source-crops.jpg');print('confirmed',[(k,len(v)) for k,v in r.items() if v])
