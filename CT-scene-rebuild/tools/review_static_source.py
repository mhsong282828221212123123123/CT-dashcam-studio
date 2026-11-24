"""Source-space envelope review for a conditional static pose; no GT claim."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from ct_scene.static_actor_refine import refine_actor,projected_box
from ct_scene.prototype_front import model_vertices,rotation
from ct_scene.affine_probe import camera_matrix
ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
poses=json.loads((a.scene.parent/'base/trajectory.json').read_text());actors=json.loads((a.scene/'actors.json').read_text());actor=next(x for x in actors if 'front-0044' in x['local_ids']);frames={f['frame_id']:f for f in json.loads((a.case/'source/frames.json').read_text())};h=json.loads((a.scene/'image-body-heading-review.json').read_text())[actor['global_id']]
profiles=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras'];mounts=json.loads(Path('configs/surrounding-prototype.json').read_text())['cameras'];v=np.unique(model_vertices('E:/tesla/project/fsd-sedan-3d/models/tesla_fsd_sedan.glb')[:,:3],axis=0);t=np.array([p['camera_time_s'] for p in poses]);ep=np.array([p['ego_position_W'] for p in poses]);ey=np.unwrap([p['ego_yaw_rad'] for p in poses])
r=refine_actor(actor,h,frames,profiles,mounts,t,ep,ey,v);(a.output/'review.json').write_text(json.dumps(r,indent=2));sheet=Image.new('RGB',(1920,960));requests=[('front',29),('front',38),('right_repeater',44),('right_repeater',48),('right_repeater',50),('right_repeater',54)]
for i,(camera,tm) in enumerate(requests):
    row=min((x for x in actor['samples'] if x['camera_id']==camera),key=lambda x:abs(x['time_s']-tm));f=frames[row['frame_id']];b=row['bbox_xyxy'];now=row['time_s'];yaw=np.interp(now,t,ey);cp=np.array([np.interp(now,t,ep[:,k]) for k in range(3)])+rotation(yaw)@np.array(mounts[camera]['offset_W_ego_m']);cp[2]+=mounts[camera]['height_m'];K,D=camera_matrix(profiles[camera]);q=projected_box(v,np.array(r['position_W']),r['yaw_rad'],cp,yaw+np.radians(mounts[camera]['yaw_deg']),K,D)
    im=Image.open(a.case/'source'/f['image']).convert('RGB');d=ImageDraw.Draw(im);d.rectangle(b,outline='red',width=3)
    if q is not None:d.rectangle(q.tolist(),outline='lime',width=3)
    d.text((20,30),camera+' '+str(round(now,2))+'s | RED detector / GREEN static full mesh envelope',fill='yellow');im=im.resize((640,480));sheet.paste(im,((i%3)*640,(i//3)*480))
sheet.save(a.output/'source.jpg');print(json.dumps(dict(accepted=r['accepted'],metrics=r.get('metrics'),changed_samples=r.get('changed_samples'),additional_source_frames=len(r.get('additional_source_reprojection_supported_frame_ids',[])),unrefined_samples=[x['time_s'] for x in actor['samples'] if 'position_before_static_mesh_fit' not in x])))
