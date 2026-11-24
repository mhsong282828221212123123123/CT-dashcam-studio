"""Recheck a frozen map with descriptor-only consensus; do not refit the map."""
import argparse
from pathlib import Path
import json
import io
import cv2
import numpy as np
import pycolmap as pc
from PIL import Image,ImageDraw
from ct_scene.artifacts import publish,verify,canonical,digest
from ct_scene.sfm import consensus_matches

parser=argparse.ArgumentParser();parser.add_argument('artifact',type=Path);parser.add_argument('work',type=Path);parser.add_argument('output',type=Path)
args=parser.parse_args();manifest=verify(args.artifact)
model=pc.Reconstruction(args.artifact/'reconstruction');camera=next(iter(model.cameras.values()))
points={pid:p.xyz for pid,p in model.points3D.items() if p.track.length()>=3};views=[]
frame_times={f['sfm_name']:f['camera_time_s'] for f in json.loads((args.artifact/'input-index.json').read_text())}
with pc.Database.open(args.work/'train.db') as db:
    for im in model.images.values():views.append((im.image_id,np.asarray(db.read_descriptors(im.image_id).data),[p.point3D_id for p in im.points2D],frame_times[im.name]))
results=[];payloads={};cv2.setNumThreads(2)
with pc.Database.open(args.work/'query.db') as db:
    for im in db.read_all_images():
        desc=db.read_descriptors(im.image_id).data;kp=db.read_keypoints(im.image_id)[:,:2]
        ids,xyz=consensus_matches(desc,views,points,frame_times[im.name])
        row={'image':im.name,'map_matches':len(ids),'gate':'FAIL'}
        if len(ids)>=45:
            order=np.random.default_rng(20261114).permutation(len(ids));test=order[::3];train=np.setdiff1d(order,test)
            est=pc.AbsolutePoseEstimationOptions();est.ransac.max_error=3;est.ransac.random_seed=20261114
            refine=pc.AbsolutePoseRefinementOptions();refine.refine_focal_length=False;refine.refine_extra_params=False
            pose=pc.estimate_and_refine_absolute_pose(kp[ids[train]],xyz[train],camera,est,refine)
            if pose is not None:
                predicted=camera.img_from_cam(pose['cam_from_world']*xyz[test]);errors=np.linalg.norm(predicted-kp[ids[test]],axis=1);valid=np.isfinite(errors)
                med=float(np.median(errors)) if valid.all() else None;p95=float(np.quantile(errors,.95)) if valid.all() else None
                row.update(test_points=len(test),pose_fit_points=len(train),median_px=med,p95_px=p95,invalid_test_projections=int((~valid).sum()),gate='PASS' if len(test)>=30 and med is not None and med<=3 and p95<=8 else 'FAIL')
                image=Image.open(args.work/'query'/im.name).convert('RGB');draw=ImageDraw.Draw(image)
                for observed,projected,good in zip(kp[ids[test]],predicted,valid):
                    if good:
                        x,y=observed;u,v=projected;draw.line([x,y,u,v],fill='yellow',width=2);draw.ellipse([x-3,y-3,x+3,y+3],outline='lime',width=2)
                buffer=io.BytesIO();image.save(buffer,format='JPEG',quality=90);payloads['proof/'+im.name]=buffer.getvalue()
        results.append(row)
report={'fixed_map':manifest['artifact_id'],'query_tests':results,'gate':'PASS_QUERY_CHECK_ONLY' if all(r['gate']=='PASS' for r in results) else 'FAIL','map_refitted':False,'query_camera_refined':False,'metric_accuracy':'NOT_EVALUATED','same_recording_development_check':True}
payloads['report.json']=canonical(report)
for name in ['train.db','query.db']:payloads['database-hashes/'+name+'.json']=canonical({'sha256':digest((args.work/name).read_bytes())})
publish(args.output,stage='frozen_map_query_check',payloads=payloads,inputs=[{'role':'frozen_sfm_map','artifact_id':manifest['artifact_id']}],config={'ratio':.75,'min_training_view_votes':2,'p95_gate_px':8,'median_gate_px':3,'min_test_points':30,'reference_window_s':3,'bracketing':True},code_revision=digest(Path(__file__).read_bytes()),runtime={'pycolmap':pc.__version__,'device':'cpu'})
print(json.dumps(report))
