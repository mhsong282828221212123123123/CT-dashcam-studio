"""Conditional two-view motion experiment; no metric ego or cross-camera rig."""
from pathlib import Path
import json
import cv2
import numpy as np
from .artifacts import canonical,digest,publish,verify


def estimate_pair(matches,lens):
    matches=np.asarray(matches,float)
    if len(matches)<40:return {'status':'rejected','reason':'insufficient_points'}
    K=np.array([[lens['fx'],0,lens['cx']],[0,lens['fy'],lens['cy']],[0,0,1]],float)
    D=np.asarray(lens['coefficients_k1_k2_p1_p2_k3'],float)
    if K[0,0]<=0 or K[1,1]<=0 or not np.isfinite(K).all() or D.shape!=(5,) or not np.isfinite(D).all():raise ValueError('Invalid lens candidate')
    p=cv2.undistortPoints(matches[:,:2].reshape(-1,1,2),K,D).reshape(-1,2)
    q=cv2.undistortPoints(matches[:,2:].reshape(-1,1,2),K,D).reshape(-1,2)
    order=np.random.default_rng(20261114).permutation(len(matches));test=order[::3];train=np.setdiff1d(order,test)
    focal=(K[0,0]+K[1,1])/2
    # Pure rotation admits arbitrary essential-matrix translation solutions.
    # Test a separate bearing-only rotation on points excluded from its fit.
    a=np.column_stack([p,np.ones(len(p))]);a/=np.linalg.norm(a,axis=1,keepdims=True)
    b=np.column_stack([q,np.ones(len(q))]);b/=np.linalg.norm(b,axis=1,keepdims=True)
    keep=train.copy()
    for _ in range(4):
        U,_,Vt=np.linalg.svd(a[keep].T@b[keep]);rotation=Vt.T@U.T
        if np.linalg.det(rotation)<0:Vt[-1]*=-1;rotation=Vt.T@U.T
        rays=a@rotation.T
        rotation_error=np.linalg.norm((rays[:,:2]/rays[:,2:]-q)*[K[0,0],K[1,1]],axis=1)
        keep=train[np.argsort(rotation_error[train])[:max(20,int(len(train)*.8))]]
    rotation_test_p95=float(np.quantile(rotation_error[test],.95))
    if float(np.mean(rotation_error[train]<.7))>=.9 and rotation_test_p95<.7:
        return {'status':'rejected','reason':'rotation_only_translation_unobservable',
                'R_rotation_only_Cnext_Cprevious':rotation.tolist(),
                'rotation_only_test_p95_undistorted_px':rotation_test_p95}
    cv2.setRNGSeed(20261114)
    E,mask=cv2.findEssentialMat(p[train],q[train],np.eye(3),method=cv2.RANSAC,prob=.999,threshold=1.5/focal,maxIters=1500)
    if E is None or mask is None:return {'status':'rejected','reason':'essential_fit_failed'}
    best=None
    for start in range(0,len(E),3):
        e=E[start:start+3]
        if e.shape!=(3,3):continue
        # OpenCV's default 50-baseline triangulation cutoff rejects distant
        # points at small inter-frame motion. Distance here has no metre scale.
        n,R,t,positive,_=cv2.recoverPose(e,p[train],q[train],np.eye(3),distanceThresh=1e6,mask=mask.copy())
        if best is None or n>best[0]:best=(n,e,R,t,positive)
    if best is None:return {'status':'rejected','reason':'no_pose_solution'}
    n,E,R,t,positive=best
    x=np.column_stack([p[test],np.ones(len(test))]);y=np.column_stack([q[test],np.ones(len(test))])
    Ex=x@E.T;Ety=y@E
    error=np.abs(np.sum(y*Ex,axis=1))/np.sqrt(np.maximum(np.sum(Ex[:,:2]**2,axis=1)+np.sum(Ety[:,:2]**2,axis=1),1e-20))*focal
    rx=x@R.T;rx/=np.linalg.norm(rx,axis=1,keepdims=True);ry=y/np.linalg.norm(y,axis=1,keepdims=True)
    parallax=np.degrees(np.arccos(np.clip(np.sum(rx*ry,axis=1),-1,1)))
    angle=float(np.degrees(np.arccos(np.clip((np.trace(R)-1)/2,-1,1))))
    inliers=int(mask.sum());ratio=n/max(inliers,1)
    p95=float(np.quantile(error,.95));medpar=float(np.median(parallax))
    reasons=[]
    if inliers<30:reasons.append('too_few_train_inliers')
    if ratio<.6:reasons.append('cheirality_weak')
    if p95>3:reasons.append('withheld_point_epipolar_residual')
    if medpar<.05:reasons.append('low_parallax')
    if angle>20:reasons.append('large_rotation_requires_review')
    return {'status':'conditional_candidate' if not reasons else 'rejected','reasons':reasons,
            'train_points':len(train),'test_points':len(test),'train_inliers':inliers,'cheirality_fraction':float(ratio),
            'heldout_sampson_p95_undistorted_equivalent_px':p95,'median_derotated_parallax_deg':medpar,
            'rotation_angle_deg':angle,'R_Cnext_Cprevious':R.tolist(),'translation_direction_Cnext':t[:,0].tolist(),
            'translation_units':'unit_vector_only_no_metric_scale','lens_status':lens['status']}


def run(features,lens_path,destination,camera='front'):
    features=Path(features);manifest=verify(features)
    lens=json.loads(Path(lens_path).read_text(encoding='utf-8'))
    if lens['model']!='pinhole_radial_tangential':raise ValueError('Only explicit pinhole candidate supported')
    report=json.loads((features/'report.json').read_text(encoding='utf-8'))
    if report.get('resolutions_by_camera',{}).get(camera)!=lens['resolution']:
        raise ValueError('Lens/source resolution is not verified compatible')
    matches=np.load(features/'matches.npz',allow_pickle=False)['matches']
    pairs=[json.loads(s) for s in (features/'pairs.jsonl').read_text().splitlines()]
    rows=[]
    for row in pairs:
        if row['camera_id']!=camera:continue
        if row['space_id']!='raw':raise ValueError('Raw correspondence space required')
        if row.get('low_motion',False):result={'status':'rejected','reason':'low_image_motion_unobservable'}
        else:
            start,count=row['array_offset'],row['array_count']
            result=estimate_pair(matches[start:start+count],lens)
        rows.append(dict(from_frame_id=row['from_frame_id'],to_frame_id=row['to_frame_id'],
                         from_camera_time_s=row['from_camera_time_s'],to_camera_time_s=row['to_camera_time_s'],**result))
    report={'pairs':len(rows),'conditional_candidates':sum(r['status']=='conditional_candidate' for r in rows),
            'lens_status':lens['status'],'metric_ego':'NOT_ESTIMATED','cross_camera_rig':'NOT_ESTIMATED',
            'warning':'Point holdout within each pair, not independent trajectory ground truth. Candidate intrinsics remain assumed. No world poses composed across rejected intervals.'}
    return publish(destination,stage='two_view_motion_candidates',payloads={'candidates.jsonl':b''.join(canonical(r)+b'\n' for r in rows),'report.json':canonical(report)},
                   inputs=[{'role':'static_candidates','artifact_id':manifest['artifact_id']}],
                   config={'camera':camera,'lens':lens,'holdout_fraction':1/3,'test_p95_px':3,'min_parallax_deg':.05},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__})
