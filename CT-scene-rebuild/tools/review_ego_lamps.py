"""Render each actual lamp mesh independently; verify shared origin and pixels."""
import json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from ct_scene.surrounding import SurroundRenderer
from ct_scene.prototype_front import model_vertices
from ct_scene.ego_lamps import role
from ct_scene.artifacts import canonical

out=Path('runs/ego-lamp-render-review-001');out.mkdir(exist_ok=True)
cfg=json.loads(Path('configs/surrounding-prototype.json').read_text());road=json.loads(Path('runs/oct21-class-001/report.json').read_text())['road']
root=Path('E:/tesla/project/fsd-sedan-3d/models');assets={k:str(root/('tesla_fsd_'+v+'.glb')) for k,v in [('ego','sedan'),('van','van'),('suv','suv'),('sedan','sedan'),('truck','small_truck'),('box_truck','box_truck'),('bus','bus')]}
renderer=SurroundRenderer(assets,road,cfg['render_style']);base=renderer.frame_many([],np.zeros(3),0,lamps={});pixels=np.asarray(base).astype(int)
sheet=Image.new('RGB',(1000,600),'#202329');records={};states=[('off',{}),('brake',{'brake':True}),('left',{'left_lit':True}),('right',{'right_lit':True}),('reverse',{'reverse':True})]
for i,(name,state) in enumerate(states):
    image=renderer.frame_many([],np.zeros(3),0,lamps=state);image.save(out/(name+'.png'))
    crop=image.crop((580,310,700,450)).resize((240,280));sheet.paste(crop,((i%4)*250,(i//4)*300));ImageDraw.Draw(sheet).text(((i%4)*250+5,(i//4)*300+5),name,fill='white')
    delta=np.any(abs(np.asarray(image).astype(int)-pixels)>2,axis=2);ys,xs=np.where(delta)
    records[name]=dict(changed_pixels=len(xs),change_bbox=None if not len(xs) else [int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())])
    if name!='off':assert len(xs)>0,(name,records[name])
sheet.save(out/'contact.png')
mesh_bounds={}
for group in ('brake','left','right','reverse'):
    vertices=model_vertices(renderer.ego_visual,node_filter=lambda n,m,g=group:role(n,m)==g)
    lo,hi=vertices[:,:3].min(0),vertices[:,:3].max(0);mesh_bounds[group]=dict(min=lo.tolist(),max=hi.tolist())
    if group in ('brake','reverse'):assert hi[0]<-1.5
    if group=='left':assert lo[1]>0
    if group=='right':assert hi[1]<0
review=dict(mesh_bounds=mesh_bounds,pixels=records,visual_asset=str(renderer.ego_visual),pose_inputs_modified=False,synthetic_lamp_states=True,actual_telemetry_preview='runs/oct21-lamps-preview-001')
(out/'review.json').write_bytes(canonical(review));print(json.dumps(review))
