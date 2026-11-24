"""Add target markers to a review copy; never alter the published scene."""
import argparse,json
from pathlib import Path
from itertools import product
from fractions import Fraction
import av,numpy as np
from PIL import ImageDraw,ImageFont
from ct_scene.prototype_front import Renderer,model_vertices,rotation

p=argparse.ArgumentParser()
for key in ('scene','package','control','assets','output'):p.add_argument('--'+key,type=Path,required=True)
p.add_argument('--review',type=Path)
a=p.parse_args()
review_status='source checks not supplied'
if a.review:
 review=json.loads(a.review.read_text())
 scene_manifest=json.loads((a.scene/'artifact.json').read_text())
 if review.get('scene_artifact_id')!=scene_manifest['artifact_id']:
  raise ValueError('Source review belongs to a different scene')
 review_status='selected source checks passed' if review.get('overall_source_gate')=='PASS' else 'selected source checks failed'
if a.output.exists():raise FileExistsError(a.output)
control=json.loads(a.control.read_text());actors=json.loads((a.scene/'actors.json').read_text());target=next(x for x in actors if control['source_local_id'] in x['local_ids'])
states=[json.loads(l) for l in (a.scene/'render-states.jsonl').read_text().splitlines()]
cfg=json.loads(Path('configs/surrounding-prototype.json').read_text())['render_style']
asset=a.assets/('tesla_fsd_'+target['model']+'.glb');renderer=Renderer(a.assets/'tesla_fsd_sedan.glb',asset,cfg['camera_eye'],cfg['camera_target'],camera_zoom=cfg['camera_zoom'])
mvp=np.frombuffer(renderer.program['mvp'].read(),np.float32).reshape(4,4).T
v=model_vertices(asset)[:,:3];corners=np.array(list(product(*zip(v.min(0),v.max(0)))))
frames=[f for f in json.loads((a.package/'frames.json').read_text()) if f['camera_id']=='front']
byframe={r['frame_id']:r for r in target['samples'] if r.get('camera_id')=='front'}
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',20);out=av.open(str(a.output),'w',options={'movflags':'+faststart'})
stream=out.add_stream('libx264',rate=20);stream.width=1280;stream.height=1000;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'};n=0
with av.open(str(a.scene/'surrounding-world.mp4')) as c:
 for i,f in enumerate(c.decode(video=0)):
  if i<560:continue
  if i>=1160:break
  s=states[i];im=f.to_image();d=ImageDraw.Draw(im)
  d.rectangle((15,674,1130,709),fill=(25,29,34));d.text((20,679),'REVIEW: BLACK SUV RIGHT CHANGE | green target | '+review_status,font=font,fill=(190,245,200))
  actor=next((x for x in s['actors'] if x['id']==target['global_id']),None)
  if actor:
   view=s['camera_view'];yaw=view['yaw_rad'];relative=rotation(-yaw)@(np.asarray(actor['position_W'])-view['position_W'])
   vertices=corners@rotation(actor['yaw_rad']-yaw).T+relative;clip=np.c_[vertices,np.ones(8)]@mvp.T
   if np.all(clip[:,3]>0):
    ndc=clip[:,:3]/clip[:,3,None];pixels=np.c_[(ndc[:,0]+1)*640,(1-ndc[:,1])*360];lo=pixels.min(0);hi=pixels.max(0)
    if hi[0]>0 and lo[0]<1280 and hi[1]>0 and lo[1]<670:
     d.rectangle((max(0,lo[0]-4),max(0,lo[1]-4),min(1279,hi[0]+4),min(669,hi[1]+4)),outline=(80,255,110),width=2)
  frame=min(frames,key=lambda f:abs(f['camera_time_s']-s['time_s']));row=byframe.get(frame['frame_id'])
  if row and row['support'] in ('detector_observation','flow_observation'):
   x,y,X,Y=row['bbox_xyxy'];d.rectangle((x/4,720+y/4,X/4,720+Y/4),outline=(80,255,110),width=2)
  vf=av.VideoFrame.from_image(im);vf.pts=n;vf.time_base=Fraction(1,20)
  for packet in stream.encode(vf):out.mux(packet)
  n+=1
 for packet in stream.encode():out.mux(packet)
out.close()
if n!=600:raise ValueError('Review interval missing')
with av.open(str(a.output)) as c:
 if sum(1 for _ in c.decode(video=0))!=600:raise ValueError('Review decode incomplete')
print('Review copy: 28-58s, 600 frames, target markers only.')
