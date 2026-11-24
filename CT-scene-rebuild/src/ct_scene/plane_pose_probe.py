"""Front-only plane-normal consistency followed by split manual side pose check.

No target check pixels participate in plane selection or side pose fitting.
The assumed facade plane is diagnostic, not a road or body-frame prior.
"""
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .affine_probe import camera_matrix
from .manual_rig_probe import project
from .artifacts import verify,publish,canonical,digest


def plane_solutions(a,b,profile):
    a,b=np.asarray(a,float),np.asarray(b,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=2 or len(a)<6:raise ValueError('Need six plane correspondences')
    K,D=camera_matrix(profile)
    ua=cv2.undistortPoints(a.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
    ub=cv2.undistortPoints(b.reshape(-1,1,2),K,D,P=K).reshape(-1,2)
    H,inside=cv2.findHomography(ua,ub,cv2.USAC_MAGSAC,1.5,maxIters=5000,confidence=.999)
    if H is None or inside is None or inside.sum()<6:raise ValueError('Unsupported front plane')
    normal_rays=np.column_stack([(ua[:,0]-K[0,2])/K[0,0],(ua[:,1]-K[1,2])/K[1,1],np.ones(len(ua))])
    count,rotations,translations,normals=cv2.decomposeHomographyMat(H,K);results=[]
    for R,t,n in zip(rotations,translations,normals):
        n=n.reshape(3);t=t.reshape(3);denominator=normal_rays@n
        if np.linalg.norm(t)<1e-4 or np.any(denominator<=1e-6):continue
        xyz=normal_rays/denominator[:,None]
        pose=np.column_stack([R,t])
        try:pred=project(xyz,pose,profile)
        except ValueError:continue
        errors=np.linalg.norm(pred-b,axis=1)
        if np.percentile(errors[inside.ravel()>0],95)>3:continue
        results.append({'normal':n,'rotation':R,'translation_over_distance':t,
                        'inliers':int(inside.sum()),'fit_p95_px':float(np.percentile(errors[inside.ravel()>0],95))})
    return results


def select_normal(candidates,tolerance_deg=5):
    """Choose using FRONT data only; multiple similarly supported clusters fail."""
    clusters=[]
    for candidate in candidates:
        n=candidate['normal'];near=[]
        for view in sorted({c['view'] for c in candidates}):
            options=[c for c in candidates if c['view']==view]
            best=min(options,key=lambda c:np.degrees(np.arccos(np.clip(n@c['normal'],-1,1))))
            angle=float(np.degrees(np.arccos(np.clip(n@best['normal'],-1,1))))
            if angle<=tolerance_deg:near.append(best)
        if len(near)<3:continue
        mean=np.mean([c['normal'] for c in near],axis=0);mean/=np.linalg.norm(mean)
        scatter=float(np.median([np.degrees(np.arccos(np.clip(mean@c['normal'],-1,1))) for c in near]))
        clusters.append((len(near),scatter,mean,near))
    if not clusters:raise ValueError('No plane normal consistent across three front views')
    clusters.sort(key=lambda c:(-c[0],c[1]));best=clusters[0]
    alternatives=[c for c in clusters if c[0]>=best[0]-1 and c[1]<=best[1]+1 and
        np.degrees(np.arccos(np.clip(c[2]@best[2],-1,1)))>10]
    if alternatives:raise ValueError('Ambiguous front plane normals; target pixels cannot choose a solution')
    return best[2],{'supporting_views':best[0],'median_normal_scatter_deg':best[1],'view_ids':[c['view'] for c in best[3]]}


def plane_points(pixels,normal,profile):
    K,D=camera_matrix(profile);r=cv2.undistortPoints(np.asarray(pixels,float).reshape(-1,1,2),K,D).reshape(-1,2)
    rays=np.column_stack([r,np.ones(len(r))]);denominator=rays@normal
    if np.any(denominator<=1e-6):raise ValueError('Plane behind camera')
    return rays/denominator[:,None]


def best_pose(xyz,pixels,profile):
    K,D=camera_matrix(profile)
    _,rv,tv,_=cv2.solvePnPGeneric(np.asarray(xyz,float),np.asarray(pixels,float),K,D,flags=cv2.SOLVEPNP_SQPNP)
    candidates=[]
    for r,t in zip(rv,tv):
        pose=np.column_stack([cv2.Rodrigues(r)[0],t.reshape(3)])
        try:prediction=project(xyz,pose,profile)
        except ValueError:continue
        error=np.linalg.norm(prediction-pixels,axis=1)
        candidates.append((float(np.mean(error**2)),pose))
    if not candidates:raise ValueError('No positive pose')
    return min(candidates,key=lambda p:p[0])


def refine_front_normal(initial,anchor_pixels,views,profile):
    """Variable projection over plane normal using only earlier FRONT images."""
    def cost(q):
        normal=np.r_[q,1.];normal/=np.linalg.norm(normal)
        try:xyz=plane_points(anchor_pixels,normal,profile)
        except ValueError:return float('inf')
        errors=[]
        for active,pixels in views:
            try:_,pose=best_pose(xyz[active],pixels[active],profile)
            except ValueError:return float('inf')
            errors.extend(np.linalg.norm(project(xyz[active],pose,profile)-pixels[active],axis=1))
        e=np.array(errors);return float(np.mean(np.where(e<=2,e*e,4*e-4)))
    if initial[2]<=.2:raise ValueError('Normal parameterization not valid for grazing plane')
    q=initial[:2]/initial[2];before=cost(q)
    for step in (.04,.01,.002):
        choices=[(cost(q+np.array([dx,dy])*step),q+np.array([dx,dy])*step) for dx in range(-2,3) for dy in range(-2,3)]
        _,q=min(choices,key=lambda item:item[0])
    n=np.r_[q,1.];n/=np.linalg.norm(n)
    return n,{'front_fit_huber_cost_before':before,'front_fit_huber_cost_after':cost(q),
              'front_fit_views':len(views),'normal_search_steps':[.04,.01,.002],
              'target_pixels_used':False,'later_front_pixels_used':False}


def fit_target_focal(xyz_fit,pixels_fit,profile):
    """Single scalar candidate fit; caller must withhold all check pixels."""
    width=profile['resolution'][0];lower,upper=.5*width,2*width
    def evaluate(focal):
        candidate=dict(profile,params=[float(focal),*profile['params'][1:]])
        try:cost,_=best_pose(xyz_fit,pixels_fit,candidate)
        except ValueError:return float('inf'),candidate
        return cost,candidate
    candidates=[evaluate(f) for f in np.linspace(lower,upper,33)]+[evaluate(profile['params'][0])]
    cost,candidate=min(candidates,key=lambda c:c[0])
    for step in ((upper-lower)/128,(upper-lower)/512,(upper-lower)/2048):
        f=candidate['params'][0]
        cost,candidate=min([evaluate(v) for v in [f-2*step,f-step,f,f+step,f+2*step] if lower<=v<=upper],key=lambda c:c[0])
    if not np.isfinite(cost) or candidate['params'][0]-lower<2 or upper-candidate['params'][0]<2:
        raise ValueError('Focal candidate at search bound or unsupported')
    curve=[{'focal':float(f),'fit_mean_squared_px':evaluate(f)[0]} for f in np.linspace(lower,upper,9)]
    return candidate,{'fit_count':len(xyz_fit),'fit_mean_squared_px':cost,'bounds':[lower,upper],
        'focal':candidate['params'][0],'curve':curve,'check_pixels_used':False,
        'near_search_bound':min(candidate['params'][0]-lower,upper-candidate['params'][0])<.05*(upper-lower),
        'status':'SINGLE_DEVELOPMENT_FACADE_CANDIDATE_NOT_ACCEPTED_OPTICS'}


def run(package,tracks_artifact,lens_config,destination,refine_target_focal=False):
    package,tracks_artifact=Path(package),Path(tracks_artifact)
    if Path(destination).exists():raise FileExistsError(destination)
    pm,tm=verify(package),verify(tracks_artifact)
    if not any(i['artifact_id']==pm['artifact_id'] for i in tm['inputs']):raise ValueError('Track/source mismatch')
    profiles=json.loads(Path(lens_config).read_text())['cameras'];cached=json.loads((tracks_artifact/'lens-candidates.json').read_text())
    if profiles!=cached:raise ValueError('Track/lens config mismatch')
    ref=json.loads((tracks_artifact/'reference.json').read_text());source=ref['source_camera'];target=ref['target_camera']
    a=np.array([p['source'] for p in ref['points']],float);tracks=json.loads((tracks_artifact/'front-track-proposals.json').read_text())
    candidates=[];rows=[];fit_views=[];check_views=[];cv2.setNumThreads(4);cv2.setRNGSeed(20261114)
    for row in tracks:
        delta=row['frame']['relative_request_s']-ref['source_request_s']
        if abs(delta)<.35:continue
        active=np.asarray(row['active'],bool)
        if active.sum()<6:continue
        if delta>0:
            check_views.append(row);continue
        fit_views.append((active,np.asarray(row['points_raw'],float)))
        try:solutions=plane_solutions(a[active],np.asarray(row['points_raw'])[active],profiles[source])
        except ValueError as exc:rows.append({'frame_id':row['frame']['frame_id'],'reason':str(exc)});continue
        rows.append({'frame_id':row['frame']['frame_id'],'time_s':row['frame']['camera_time_s'],'solutions':len(solutions)})
        for solution in solutions:candidates.append(dict(solution,view=row['frame']['frame_id']))
    report={'gate':'FAIL_FRONT_PLANE','front_view_diagnostics':rows,'rig':'NOT_ACCEPTED','metric_scale':'UNESTABLISHED',
            'assumptions':['Approximate manually identified coplanar facade controls','Estimated lenses','Unverified camera exposure clocks'],
            'target_check_used_for_selection':False};payloads={}
    try:normal,diagnostic=select_normal(candidates)
    except ValueError as exc:report['reason']=str(exc)
    else:
        normal,refinement=refine_front_normal(normal,a,fit_views,profiles[source])
        report['front_normal_refinement']=refinement
        xyz=plane_points(a,normal,profiles[source]);fit=np.array([p['split']=='fit' for p in ref['points']])
        report['later_front_checks']=[]
        for row in check_views:
            active=np.asarray(row['active'],bool);pixels=np.asarray(row['points_raw'],float)
            fit_mask=active&fit;check_mask=active&~fit
            if fit_mask.sum()<4 or check_mask.sum()<3:continue
            try:
                _,pose=best_pose(xyz[fit_mask],pixels[fit_mask],profiles[source])
                err=np.linalg.norm(project(xyz[check_mask],pose,profiles[source])-pixels[check_mask],axis=1)
                report['later_front_checks'].append({'time_s':row['frame']['camera_time_s'],'fit_count':int(fit_mask.sum()),
                    'check_count':int(check_mask.sum()),'check_median_px':float(np.median(err)),'check_p95_px':float(np.percentile(err,95))})
            except ValueError:report['later_front_checks'].append({'time_s':row['frame']['camera_time_s'],'status':'FAIL_NO_POSITIVE_POSE'})
        target_profile=profiles[target]
        if refine_target_focal:
            try:
                target_profile,diagnostic_focal=fit_target_focal(xyz[fit],np.array([p['target'] for p in ref['points']],float)[fit],target_profile)
                report['target_optical_fit']=diagnostic_focal
                payloads['target-optical-candidate.json']=canonical(target_profile)
            except ValueError as exc:
                report['target_optical_fit']={'status':'REJECTED','reason':str(exc)}
        Kt,Dt=camera_matrix(target_profile);count,rv,tv,_=cv2.solvePnPGeneric(xyz[fit],np.array([p['target'] for p in ref['points']],float)[fit],Kt,Dt,flags=cv2.SOLVEPNP_SQPNP)
        poses=[];observed=np.array([p['target'] for p in ref['points']],float)
        for r,t in zip(rv,tv):
            pose=np.column_stack([cv2.Rodrigues(r)[0],t.reshape(3)])
            try:pred=project(xyz,pose,target_profile)
            except ValueError:continue
            poses.append((float(np.median(np.linalg.norm(pred[fit]-observed[fit],axis=1))),pose,pred))
        report.update(front_normal=normal.tolist(),normal_consistency=diagnostic,positive_target_pose_candidates=len(poses))
        if not poses:report['reason']='No positive target pose from fit controls'
        else:
            fit_error,pose,pred=min(poses,key=lambda p:p[0]);errors=np.linalg.norm(pred-observed,axis=1)
            report.update(target_fit_count=int(fit.sum()),target_check_count=int((~fit).sum()),target_fit_median_px=fit_error,
                target_check_median_px=float(np.median(errors[~fit])),target_check_p95_px=float(np.percentile(errors[~fit],95)),
                errors_px=[{'id':p['id'],'split':p['split'],'error_px':float(e)} for p,e in zip(ref['points'],errors)],
                pose_target_from_source_plane_gauge=pose.tolist())
            later_checks=report['later_front_checks']
            front_ok=len(later_checks)>=3 and all(c.get('check_median_px',float('inf'))<=3 and c.get('check_p95_px',float('inf'))<=8 for c in later_checks)
            report['later_front_gate']='PASS_CONDITIONAL_ONLY' if front_ok else 'FAIL'
            target_ok=fit.sum()>=6 and (~fit).sum()>=5 and np.median(errors[~fit])<=3 and np.percentile(errors[~fit],95)<=8
            report['gate']='PASS_PLANE_DIAGNOSTIC_ONLY' if front_ok and target_ok else ('FAIL_LATER_FRONT_CHECK' if not front_ok else 'FAIL_TARGET_REPROJECTION')
            frames=json.loads((package/'frames.json').read_text());f=min((f for f in frames if f['camera_id']==target),key=lambda f:abs(f['relative_request_s']-ref['target_request_s']))
            image=Image.open(package/f['image']).convert('RGB');d=ImageDraw.Draw(image)
            for p,(u,v),(pu,pv) in zip(ref['points'],observed,pred):
                color='cyan' if p['split']=='fit' else 'yellow';d.ellipse((u-4,v-4,u+4,v+4),outline=color,width=2);d.line((u,v,pu,pv),fill='red',width=2)
            buf=io.BytesIO();image.save(buf,format='JPEG',quality=95);payloads['proof/target-check.jpg']=buf.getvalue()
    payloads['report.json']=canonical(report)
    payloads['front-plane-candidates.json']=canonical([{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in c.items()} for c in candidates])
    return publish(destination,stage='plane_pose_probe',payloads=payloads,
        inputs=[{'role':'source','artifact_id':pm['artifact_id']},{'role':'front_track_proposals','artifact_id':tm['artifact_id']}],
        config={'profiles':profiles,'reference':ref,'normal_tolerance_deg':5,'min_views':3,'refine_target_focal':refine_target_focal},
        code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'device':'cpu'})
