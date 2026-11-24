"""Controlled ASIFT fallback probe for confirmed front/side matching failure."""
from pathlib import Path
import io
import json
import time
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .artifacts import publish,verify,canonical,digest
from .static_features import feature_roi,optical_feature_roi


def camera_matrix(profile):
    expected={'SIMPLE_RADIAL':4,'RADIAL':5}
    if profile.get('model') not in expected or len(profile['params'])!=expected[profile['model']]:
        raise ValueError('Unsupported optical model for rectification')
    if not np.isfinite(profile['params']).all() or profile['params'][0]<=0:
        raise ValueError('Invalid optical candidate')
    f,cx,cy=profile['params'][:3]
    K=np.array([[f,0,cx],[0,f,cy],[0,0,1]],float)
    d=list(profile['params'][3:]);D=np.array([d[0] if d else 0,d[1] if len(d)>1 else 0,0,0,0],float)
    return K,D


def affine_features(gray,mask,profile,camera_id=None):
    K,D=camera_matrix(profile);h,w=gray.shape
    mx,my=cv2.initUndistortRectifyMap(K,D,None,K,(w,h),cv2.CV_32FC1)
    image=cv2.remap(gray,mx,my,cv2.INTER_LINEAR)
    roi=feature_roi(mask) if camera_id is None else optical_feature_roi(mask,camera_id)
    valid=cv2.remap(roi,mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    detector=cv2.AffineFeature_create(cv2.SIFT_create(nfeatures=1200,contrastThreshold=.015),maxTilt=2)
    keypoints,desc=detector.detectAndCompute(image,valid)
    if desc is None:return image,np.empty((0,2),np.float32),np.empty((0,128),np.float32)
    # RootSIFT; multiple tilted descriptors may share one physical image point.
    desc=np.sqrt(desc/np.maximum(desc.sum(axis=1,keepdims=True),1e-12)).astype(np.float32)
    points=np.array([k.pt for k in keypoints],np.float32)
    return image,points,desc


def match_affine(a,b):
    pa,da=a[1:];pb,db=b[1:]
    if len(da)<2 or len(db)<2:return np.empty((0,4),np.float32),{'unique_matches':0,'inliers':0}
    # A pair's result must not depend on which other pairs ran before it.
    cv2.setRNGSeed(20261114)
    matcher=cv2.FlannBasedMatcher(dict(algorithm=1,trees=4),dict(checks=96))
    pairs=matcher.knnMatch(da,db,k=2)
    reverse={m[0].queryIdx:m[0].trainIdx for m in matcher.knnMatch(db,da,k=1) if m}
    candidates=[]
    for pair in pairs:
        if len(pair)!=2:continue
        x,y=pair
        if x.distance<.75*y.distance and reverse.get(x.trainIdx)==x.queryIdx:candidates.append(x)
    seen_a,seen_b=set(),set();matches=[]
    for match in sorted(candidates,key=lambda m:m.distance):
        x,y=pa[match.queryIdx],pb[match.trainIdx]
        ka,kb=tuple(np.floor(x/4).astype(int)),tuple(np.floor(y/4).astype(int))
        if ka in seen_a or kb in seen_b:continue
        seen_a.add(ka);seen_b.add(kb);matches.append(np.r_[x,y])
    matches=np.array(matches,np.float32).reshape(-1,4)
    if len(matches)<16:return matches,{'unique_matches':len(matches),'inliers':0}
    F,inliers=cv2.findFundamentalMat(matches[:,:2],matches[:,2:],cv2.USAC_MAGSAC,2,.999,5000)
    if F is None or inliers is None:return matches,{'unique_matches':len(matches),'inliers':0}
    selected=matches[inliers.ravel()>0]
    return selected,{'unique_matches':len(matches),'inliers':len(selected),'inlier_fraction':len(selected)/len(matches)}


def run(package,masks,lens_config,destination,side_delays=(2,4,6,8),front_times=tuple(range(0,32,4)),roi_mode='camera_aware_v1'):
    if roi_mode not in ('camera_aware_v1','legacy_conservative'):raise ValueError('Unknown feature ROI')
    side_delays=tuple(side_delays)
    if not side_delays or any(not np.isfinite(d) or d<=0 for d in side_delays):
        raise ValueError('Side delays must be finite and positive')
    package,masks,destination=Path(package),Path(masks),Path(destination)
    if destination.exists():raise FileExistsError(destination)
    pm,mm=verify(package),verify(masks)
    if not any(i['artifact_id']==pm['artifact_id'] for i in mm['inputs']):raise ValueError('Source/mask mismatch')
    profiles=json.loads(Path(lens_config).read_text())
    frames=json.loads((package/'frames.json').read_text());obs={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    requests=[]
    if not front_times or any(not np.isfinite(t) or t<0 or t>34 for t in front_times):
        raise ValueError('Front search time outside reviewed 35-second package')
    for t in front_times:
        for delay in side_delays:
            if t+delay>34:continue
            for side in ('left_repeater','right_repeater'):requests.append((('front',t),(side,t+delay)))
    features={};chosen={};payloads={};cv2.setNumThreads(4);cv2.setRNGSeed(20261114);start=time.perf_counter()
    for camera,t in sorted({k for request in requests for k in request}):
        f=min((f for f in frames if f['camera_id']==camera),key=lambda f:abs(f['relative_request_s']-t))
        profile=profiles['cameras'][camera]
        if [f['width'],f['height']]!=profile['resolution']:raise ValueError('Lens/source dimensions differ')
        image=np.asarray(Image.open(package/f['image']).convert('L'));mask=np.asarray(Image.open(masks/obs[f['frame_id']]['static_mask']))
        result=affine_features(image,mask,profile,camera if roi_mode=='camera_aware_v1' else None);features[camera,t]=result;chosen[camera,t]=f
        buf=io.BytesIO();np.savez_compressed(buf,points_rectified=result[1],root_descriptors=result[2]);payloads[f'features/{camera}-{t}.npz']=buf.getvalue()
        print('ASIFT',camera,t,len(result[1]),flush=True)
    rows=[];best={}
    for ka,kb in requests:
        matches,stats=match_affine(features[ka],features[kb]);side=kb[0]
        row={'from_frame_id':chosen[ka]['frame_id'],'to_frame_id':chosen[kb]['frame_id'],
             'front_time_s':ka[1],'side_time_s':kb[1],'side':side,'space_id':'candidate_rectified',**stats}
        rows.append(row)
        if side not in best or stats['inliers']>best[side][0]['inliers']:best[side]=(row,matches,ka,kb)
    for side,(row,matches,ka,kb) in best.items():
        ia,ib=Image.fromarray(features[ka][0]).convert('RGB'),Image.fromarray(features[kb][0]).convert('RGB')
        proof=Image.new('RGB',(ia.width+ib.width,max(ia.height,ib.height)));proof.paste(ia,(0,0));proof.paste(ib,(ia.width,0));draw=ImageDraw.Draw(proof)
        for x,y,u,v in matches[::max(1,len(matches)//50)]:draw.line([x,y,ia.width+u,v],fill='yellow',width=2)
        buf=io.BytesIO();proof.save(buf,format='JPEG',quality=90);payloads['proof/'+side+'.jpg']=buf.getvalue()
    report={'pairs_tested':len(rows),'best_by_side':{k:v[0] for k,v in best.items()},'wall_time_s':time.perf_counter()-start,
            'rig':'NOT_ESTIMATED','metric_geometry':'NOT_EVALUATED','warning':'Deduplicated descriptor/MAGSAC matches are proposals only; inspect physical landmark identity. Lens seeds are estimated.'}
    payloads['report.json']=canonical(report);payloads['pairs.jsonl']=b''.join(canonical(r)+b'\n' for r in rows)
    config={'lens_candidates':profiles,'asift_max_tilt':2,'ratio':.75,'dedupe_cell_px':4,'magsac_px':2,'side_delays_s':list(side_delays),'front_times_s':list(front_times),'roi':roi_mode}
    payloads['experiment-config.json']=canonical(config)
    return publish(destination,stage='affine_cross_camera_probe',payloads=payloads,
                   inputs=[{'role':'source','artifact_id':pm['artifact_id']},{'role':'masks','artifact_id':mm['artifact_id']}],
                   config=config,
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'device':'cpu'})
