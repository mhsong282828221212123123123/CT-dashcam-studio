"""Verify observed rear colour changes only the audited rear mesh, not pose."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from ct_scene.surrounding import SurroundRenderer
from ct_scene.surrounding_lamps import rear_material
from ct_scene.prototype_front import model_vertices

ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
cfg=json.loads(Path('configs/surrounding-prototype.json').read_text());road=json.loads(Path('runs/aug28-curved-pocket-005/report.json').read_text())['road'];root=Path('E:/tesla/project/fsd-sedan-3d/models')
assets={k:str(root/('tesla_fsd_'+v+'.glb')) for k,v in [('ego','sedan'),('van','van'),('suv','suv'),('sedan','sedan'),('truck','small_truck'),('box_truck','box_truck'),('heavy_truck','heavy_truck'),('bus','bus')]};renderer=SurroundRenderer(assets,road,cfg['render_style']);sheet=Image.new('RGB',(1200,280*7),'#171b20');d=ImageDraw.Draw(sheet);results={}
for i,name in enumerate(k for k in assets if k!='ego'):
    actor=dict(id='fixture',position_W=np.array([8.,0,0]),yaw_rad=0.,model=name,ghost=False)
    unknown=renderer.frame_many([actor],np.zeros(3),0);actor['rear_appearance']={'rear_red_visible':True};red=renderer.frame_many([actor],np.zeros(3),0)
    actor['ghost']=True;ghost=renderer.frame_many([actor],np.zeros(3),0);actor['rear_appearance']={};ghost_unknown=renderer.frame_many([actor],np.zeros(3),0)
    diff=np.any(abs(np.asarray(red).astype(int)-np.asarray(unknown).astype(int))>2,axis=2);ys,xs=np.where(diff);tail=model_vertices(assets[name],node_filter=lambda n,m:rear_material(m))
    results[name]=dict(changed_pixels=int(diff.sum()),rear_mesh_max_x=float(tail[:,0].max()),ghost_unchanged=bool(np.array_equal(np.asarray(ghost),np.asarray(ghost_unknown))),bbox=[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())] if len(xs) else None)
    assert results[name]['changed_pixels']>0 and results[name]['rear_mesh_max_x']<-1.5 and results[name]['ghost_unchanged'],results[name]
    for col,(label,image) in enumerate([('unknown / clay',unknown),('observed red appearance',red),('ghost / no red state',ghost)]):
        crop=image.crop((420,180,860,490)).resize((400,260));sheet.paste(crop,(col*400,i*280+20));d.text((col*400+5,i*280+3),name+' '+label,fill='white')
sheet.save(a.output/'contact.jpg');(a.output/'review.json').write_text(json.dumps(dict(scope='synthetic mesh-render check; source cues reviewed separately; brake and indicators remain unknown',results=results),indent=2));print(json.dumps(results))
