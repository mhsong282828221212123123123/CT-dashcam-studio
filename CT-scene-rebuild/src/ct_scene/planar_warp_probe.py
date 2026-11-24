"""Automatic plane-seeded viewpoint normalization experiment, never a rig fit."""
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .affine_probe import camera_matrix,match_affine
from .static_features import optical_feature_roi
from .artifacts import verify,publish,canonical,digest


def root_sift(image,mask):
    keys,desc=cv2.SIFT_create(nfeatures=4000,contrastThreshold=.01).detectAndCompute(image,mask)
    if desc is None:return image,np.empty((0,2),np.float32),np.empty((0,128),np.float32)
    return image,np.array([k.pt for k in keys],np.float32),np.sqrt(desc/np.maximum(desc.sum(1,keepdims=True),1e-12)).astype(np.float32)


def refine(a,b,mask_a,mask_b,seed):
    if len(seed)<12:raise ValueError('Insufficient automatic seed support')
    H,inliers=cv2.findHomography(seed[:,:2],seed[:,2:],cv2.USAC_MAGSAC,2,maxIters=5000,confidence=.999)
    if H is None or inliers is None or int(inliers.sum())<12:raise ValueError('No supported automatic plane')
    selected=seed[inliers.ravel()>0];h,w=a.shape
    if cv2.contourArea(cv2.convexHull(np.ascontiguousarray(selected[:,:2],np.float32)))<1000:raise ValueError('Plane seed too concentrated')
    domain=np.zeros_like(mask_a);cv2.fillConvexPoly(domain,cv2.convexHull(np.rint(selected[:,:2]).astype(np.int32)),255)
    domain=cv2.dilate(domain,np.ones((73,73),np.uint8))
    inverse=np.linalg.inv(H)
    warped=cv2.warpPerspective(b,inverse,(w,h))
    valid=cv2.bitwise_and(cv2.warpPerspective(mask_b,inverse,(w,h),flags=cv2.INTER_NEAREST),cv2.bitwise_and(domain,mask_a))
    pairs,stats=match_affine(root_sift(a,valid),root_sift(warped,valid))
    if stats['inliers']<12:raise ValueError('Insufficient verified warped matches')
    target=cv2.perspectiveTransform(pairs[:,2:].reshape(-1,1,2),H).reshape(-1,2)
    pairs=np.column_stack([pairs[:,:2],target]).astype(np.float32)
    # De-duplicate physical pixels in BOTH original rectified camera images.
    used_a,used_b=set(),set();unique=[]
    for row in pairs:
        ca,cb=tuple(np.floor(row[:2]/4).astype(int)),tuple(np.floor(row[2:]/4).astype(int))
        if ca in used_a or cb in used_b:continue
        used_a.add(ca);used_b.add(cb);unique.append(row)
    pairs=np.asarray(unique,np.float32).reshape(-1,4)
    if len(pairs)>=4:
        _,ok=cv2.findHomography(pairs[:,:2],pairs[:,2:],cv2.USAC_MAGSAC,2,maxIters=5000,confidence=.999)
        if ok is not None:pairs=pairs[ok.ravel()>0]
    return pairs,{'seed_inliers':int(inliers.sum()),'refined_inliers':len(pairs),'warped_matching':stats,
                  'H_seed':H.tolist(),'warning':'Plane-guided fitted matches are not heldout reprojection or independent 3D evidence.'}


def run(package,masks,probe,lens_config,destination):
    package,masks,probe=Path(package),Path(masks),Path(probe)
    manifests=[verify(p) for p in (package,masks,probe)]
    for m in manifests[1:]:
        if not any(i['artifact_id']==manifests[0]['artifact_id'] for i in m['inputs']):raise ValueError('Source/probe mismatch')
    config=json.loads((probe/'experiment-config.json').read_text())
    profiles=json.loads(Path(lens_config).read_text())['cameras']
    if config['lens_candidates']['cameras']!=profiles or config['roi']!='camera_aware_v1':
        raise ValueError('Cached rectified features use different lens/ROI configuration')
    rows=json.loads((package/'frames.json').read_text())
    observations={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    report=json.loads((probe/'report.json').read_text());payloads={};out={};cv2.setNumThreads(4);cv2.setRNGSeed(20261114)
    for side,pair in report['best_by_side'].items():
        images=[];valid=[];features=[]
        for camera,t,fid in [('front',pair['front_time_s'],pair['from_frame_id']),(side,pair['side_time_s'],pair['to_frame_id'])]:
            f=next(f for f in rows if f['frame_id']==fid)
            image=np.asarray(Image.open(package/f['image']).convert('L'));mask=np.asarray(Image.open(masks/observations[fid]['static_mask']))
            K,D=camera_matrix(profiles[camera]);h,w=image.shape;mx,my=cv2.initUndistortRectifyMap(K,D,None,K,(w,h),cv2.CV_32FC1)
            images.append(cv2.remap(image,mx,my,cv2.INTER_LINEAR))
            valid.append(cv2.remap(optical_feature_roi(mask,camera),mx,my,cv2.INTER_NEAREST))
            with np.load(probe/f'features/{camera}-{t}.npz',allow_pickle=False) as data:
                features.append((images[-1],data['points_rectified'],data['root_descriptors']))
        seed,stats=match_affine(*features)
        try:pairs,diagnostic=refine(*images,*valid,seed)
        except ValueError as exc:
            out[side]={'status':'REJECTED','reason':str(exc),'seed':stats};continue
        out[side]={'source_pair':pair,'seed':stats,**diagnostic}
        proof=Image.new('RGB',(2560,1000),'#111');proof.paste(Image.fromarray(images[0]).convert('RGB'),(0,40));proof.paste(Image.fromarray(images[1]).convert('RGB'),(1280,40));d=ImageDraw.Draw(proof)
        d.text((10,10),f'Plane-seeded automatic proposals: {len(pairs)}; NOT rig/metric acceptance',fill='white')
        for x,y,u,v in pairs[::max(1,len(pairs)//80)]:d.line((x,y+40,u+1280,v+40),fill='yellow',width=1)
        buf=io.BytesIO();proof.save(buf,format='JPEG',quality=92);payloads['proof/'+side+'.jpg']=buf.getvalue()
        buf=io.BytesIO();np.savez_compressed(buf,matches_rectified=pairs);payloads['matches/'+side+'.npz']=buf.getvalue()
    payloads['report.json']=canonical({'results':out,'rig':'NOT_ESTIMATED','metric_geometry':'NOT_EVALUATED'})
    return publish(destination,stage='planar_warp_probe',payloads=payloads,
                   inputs=[{'role':role,'artifact_id':m['artifact_id']} for role,m in zip(['source','masks','affine_probe'],manifests)],
                   config={'lens_candidates':profiles,'domain_dilation_px':36,'homography_px':2},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'device':'cpu'})
