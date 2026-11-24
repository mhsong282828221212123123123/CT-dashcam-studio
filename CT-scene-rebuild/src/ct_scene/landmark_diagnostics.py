"""Manual planar controls diagnose pixel/mask/matcher failures, not a 3D rig."""
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .affine_probe import camera_matrix
from .static_features import feature_roi,optical_feature_roi
from .artifacts import verify,publish,canonical,digest


def planar_check(source,target,fit,source_profile=None,target_profile=None):
    source=np.asarray(source,dtype=float);target=np.asarray(target,dtype=float);fit=np.asarray(fit,dtype=bool)
    if source.shape!=target.shape or source.ndim!=2 or source.shape[1]!=2 or len(fit)!=len(source):
        raise ValueError('Invalid manual correspondence shape')
    if not np.isfinite(source).all() or not np.isfinite(target).all() or fit.sum()<4 or (~fit).sum()<1:
        raise ValueError('Need finite independent fit/check points')
    for points in (source[fit],target[fit]):
        if np.linalg.svd(points-points.mean(0),compute_uv=False)[-1]<2:
            raise ValueError('Collinear or insufficiently spread fit controls')
    a,b=source.copy(),target.copy()
    if (source_profile is None)!=(target_profile is None):raise ValueError('Both camera profiles required')
    if source_profile is not None:
        Ks,Ds=camera_matrix(source_profile);Kt,Dt=camera_matrix(target_profile)
        a=cv2.undistortPoints(a.reshape(-1,1,2),Ks,Ds,P=Ks).reshape(-1,2)
        b=cv2.undistortPoints(b.reshape(-1,1,2),Kt,Dt,P=Kt).reshape(-1,2)
    H,_=cv2.findHomography(a[fit],b[fit],0)
    if H is None or not np.isfinite(H).all() or np.linalg.cond(H)>1e12:raise ValueError('Degenerate planar fit')
    prediction=cv2.perspectiveTransform(a.reshape(-1,1,2),H).reshape(-1,2)
    if source_profile is not None:
        rays=np.column_stack([(prediction[:,0]-Kt[0,2])/Kt[0,0],(prediction[:,1]-Kt[1,2])/Kt[1,1],np.ones(len(prediction))])
        prediction=cv2.projectPoints(rays,np.zeros(3),np.zeros(3),Kt,Dt)[0].reshape(-1,2)
    errors=np.linalg.norm(prediction-target,axis=1)
    return {'H':H.tolist(),'predicted_target_raw':prediction.tolist(),'errors_raw_px':errors.tolist(),
            'fit_count':int(fit.sum()),'check_count':int((~fit).sum()),
            'fit_median_px':float(np.median(errors[fit])),
            'check_median_px':float(np.median(errors[~fit])),
            'check_p95_px':float(np.percentile(errors[~fit],95))}


def run(package,masks,lens_config,reference,destination):
    package,masks=Path(package),Path(masks);pm,mm=verify(package),verify(masks)
    if not any(i['artifact_id']==pm['artifact_id'] for i in mm['inputs']):raise ValueError('Source/mask mismatch')
    ref=json.loads(Path(reference).read_text());profiles=json.loads(Path(lens_config).read_text())['cameras']
    if ref['space_id']!='raw':raise ValueError('Manual controls must use original raw pixels')
    frames=json.loads((package/'frames.json').read_text());observations={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    selected=[];images=[];visibility={};payloads={}
    for role in ('source','target'):
        camera=ref[role+'_camera'];t=ref[role+'_request_s']
        f=min((f for f in frames if f['camera_id']==camera),key=lambda f:abs(f['relative_request_s']-t))
        if abs(f['relative_request_s']-t)>.11:raise ValueError('Requested reference frame missing')
        if [f['width'],f['height']]!=profiles[camera]['resolution']:raise ValueError('Lens resolution mismatch')
        selected.append(f);image=Image.open(package/f['image']).convert('RGB');images.append(image)
        mask=np.asarray(Image.open(masks/observations[f['frame_id']]['static_mask']))
        old,new=feature_roi(mask),optical_feature_roi(mask,camera);rows=[]
        for p in ref['points']:
            x,y=np.rint(p[role]).astype(int)
            if not (0<=x<f['width'] and 0<=y<f['height']):raise ValueError('Reference outside image')
            rows.append({'id':p['id'],'dynamic_mask_allows':bool(mask[y,x]),'old_roi_allows':bool(old[y,x]),'new_roi_allows':bool(new[y,x])})
        visibility[role]=rows
    a=[p['source'] for p in ref['points']];b=[p['target'] for p in ref['points']];fit=[p['split']=='fit' for p in ref['points']]
    results={'raw_homography':planar_check(a,b,fit),
             'candidate_rectified':planar_check(a,b,fit,profiles[ref['source_camera']],profiles[ref['target_camera']])}
    for mode,result in results.items():
        proof=Image.new('RGB',(2560,1000),'#151515');proof.paste(images[0],(0,40));proof.paste(images[1],(1280,40));d=ImageDraw.Draw(proof)
        d.text((10,8),f'{mode}: manual controls; cyan=fit, yellow=withheld; red=prediction residual',fill='white')
        for p,pred in zip(ref['points'],result['predicted_target_raw']):
            color='cyan' if p['split']=='fit' else 'yellow'
            x,y=p['source'];u,v=p['target'];pu,pv=pred
            for px,py in [(x,y+40),(u+1280,v+40)]:d.ellipse((px-4,py-4,px+4,py+4),outline=color,width=2)
            d.text((x+5,y+40),p['id'],fill=color);d.text((u+1285,v+40),p['id'],fill=color)
            d.line((u+1280,v+40,pu+1280,pv+40),fill='red',width=2)
        buf=io.BytesIO();proof.save(buf,format='JPEG',quality=92);payloads['proof/'+mode+'.jpg']=buf.getvalue()
    both=lambda key:sum(x[key] and y[key] for x,y in zip(visibility['source'],visibility['target']))
    report={'models':results,'mask_audit':visibility,'both_old_roi_allowed':both('old_roi_allows'),
            'both_new_roi_allowed':both('new_roi_allows'),'selected_frames':selected,
            'rig':'NOT_ESTIMATED','scope':'Manual approximate planar diagnostic only. No metric geometry, optical acceptance or automatic matching acceptance.'}
    payloads['report.json']=canonical(report);payloads['reference.json']=canonical(ref);payloads['lens-candidates.json']=canonical(profiles)
    return publish(destination,stage='manual_planar_diagnostic',payloads=payloads,
                   inputs=[{'role':'source','artifact_id':pm['artifact_id']},{'role':'masks','artifact_id':mm['artifact_id']}],
                   config={'reference':ref,'profiles':profiles},code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'device':'cpu'})
