"""Verified source frames for explicit identity/time checks; never guesses a folder."""
import argparse,json,math
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from ct_scene.artifacts import verify,canonical
from review_guards import require_input_artifact


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--ids',nargs='+',required=True);p.add_argument('--times',nargs='+',type=float,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    scene=verify(a.scene);source=verify(a.source);require_input_artifact(scene,source,'source')
    actors=json.loads((a.scene/'actors.json').read_text());frames={r['frame_id']:r for r in json.loads((a.source/'frames.json').read_text())};columns=min(3,len(a.ids));tiles=[];proof=[];font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
    for t in a.times:
        for key in a.ids:
            actor=next(r for r in actors if key in r.get('local_ids',[]) or key in [r['id'],r.get('global_id')]);cam=key.rsplit('-',1)[0];rows=[r for r in actor['samples'] if r['camera_id']==cam];row=min(rows,key=lambda r:abs(r['time_s']-t));f=frames[row['frame_id']]
            if f['camera_id']!=cam:raise ValueError('Frame camera mismatch')
            if abs(row['time_s']-t)>.15:raise ValueError('Requested identity not observed near requested time')
            im=Image.open(a.source/f['image']).convert('RGB');d=ImageDraw.Draw(im);d.rectangle(row['bbox_xyxy'],outline='cyan',width=3)
            tile=Image.new('RGB',(640,530),'#17191c');tile.paste(im.resize((640,480)),(0,50));d=ImageDraw.Draw(tile);d.text((5,4),f"{key} scene {row['time_s']:.3f} / camera {f['camera_time_s']:.3f}s",fill='white',font=font);d.text((5,27),row['support'],fill='cyan',font=font);tiles.append(tile)
            proof.append(dict(id=key,requested_time_s=t,scene_time_s=row['time_s'],camera_time_s=f['camera_time_s'],frame_id=f['frame_id'],image=f['image'],bbox_xyxy=row['bbox_xyxy'],support=row['support']))
    sheet=Image.new('RGB',(640*columns,530*math.ceil(len(tiles)/columns)),'#17191c')
    for i,tile in enumerate(tiles):sheet.paste(tile,((i%columns)*640,(i//columns)*530))
    sheet.save(a.output);a.output.with_suffix('.json').write_bytes(canonical(dict(scene_artifact_id=scene['artifact_id'],source_artifact_id=source['artifact_id'],frames=proof,scope='exact frame-id original visual evidence; no metric 3D ground truth')))
