"""Offline image-size range refinement for far frozen front anchors.

A frozen anchor is an estimate, not ground truth. This late stage deliberately
leaves ego, road, identity and non-anchor actors alone. It only repairs wholly
far tracks whose contact-derived motion disagrees with clear body-size evidence.
Absolute scale still depends on an estimated optical rig and class-height prior.
"""
import math
import cv2
import numpy as np
from .affine_probe import camera_matrix
from .collision_constraints import audit as collision_audit


def smooth_curve(times, values, query, tau_s=.6):
    """Robust offline second-derivative regularization, using both time directions."""
    times=np.asarray(times,float);values=np.asarray(values,float)
    grid=np.linspace(times[0],times[-1],max(4,int(math.ceil((times[-1]-times[0])/.2))+1))
    dt=grid[1]-grid[0]
    idx=np.clip(np.searchsorted(grid,times)-1,0,len(grid)-2)
    alpha=(times-grid[idx])/dt
    A=np.zeros((len(times),len(grid)));A[np.arange(len(times)),idx]=1-alpha;A[np.arange(len(times)),idx+1]=alpha
    D=np.diff(np.eye(len(grid)),n=2,axis=0)
    penalty=(tau_s/dt)**4*(D.T@D)+np.eye(len(grid))*1e-9
    weights=np.ones(len(times))
    for _ in range(4):
        fitted=np.linalg.solve(A.T@(weights[:,None]*A)+penalty,A.T@(weights*values))
        residual=A@fitted-values
        scale=max(1e-5,1.4826*float(np.median(abs(residual-np.median(residual)))))
        weights=np.minimum(1.,2.5*scale/np.maximum(abs(residual),1e-12))
    return np.interp(query,grid,fitted)


def refine_far_leads(states,actors,profile,dimensions,mount):
    """Modify only accepted front-anchor positions in final render states.

    No cap or lane clamp makes a bad estimate pass. Candidates must disagree
    with old contact range and its rate, cover the full displayed lifespan, and
    introduce no new body overlap. Missing/near/ambiguous tracks abstain.
    """
    K,D=camera_matrix(profile);reviews=[]
    state_times=np.array([s['time_s'] for s in states],float)
    for actor in actors:
        if not actor.get('frozen_lead') or actor.get('camera_id')!='front':continue
        key=actor.get('global_id',actor.get('id'));model=actor['model']
        review=dict(id=key,status='abstained',absolute_metric_accuracy='NOT_EVALUATED')
        reviews.append(review)
        rows=[]
        for r in actor['samples']:
            b=r.get('bbox_xyxy')
            if b is None or r.get('support')!='detector_observation':continue
            w,h=b[2]-b[0],b[3]-b[1]
            width=r.get('image_width',1280);height=r.get('image_height',960)
            if min(w,h)<20 or min(b[0],b[1],width-b[2],height-b[3])<8 or w/h>2.2:continue
            cx=(b[0]+b[2])/2
            rays=cv2.undistortPoints(np.array([[[cx,b[1]]],[[cx,b[3]]]],float),K,D).reshape(2,2)
            angular_height=rays[1,1]-rays[0,1]
            depth=float(dimensions[model][2]/angular_height) if angular_height>0 else float('nan')
            if not np.isfinite(depth) or not 3<depth<140:continue
            rows.append((float(r['time_s']),math.log(angular_height),float(np.mean(rays[:,0])),depth))
        rows=sorted({r[0]:r for r in rows}.values())
        if len(rows)<12 or rows[-1][0]-rows[0][0]<3:
            review['reason']='insufficient_clear_body_size_evidence';continue
        obs=np.array(rows);ts=obs[:,0]
        # Near targets and side-on/crossing targets retain their original estimate.
        # Do not blend a large far correction into an unchanged near track.
        if np.min(obs[:,3])<35 or np.mean(obs[:,3]>45)<.75:
            review['reason']='not_a_wholly_far_front_track';continue
        entries=[(i,a) for i,s in enumerate(states) for a in s['actors'] if a['id']==key]
        if not entries:
            review['reason']='not_displayed';continue
        query=np.array([state_times[i] for i,a in entries])
        if query[0]<ts[0]-.15 or query[-1]>ts[-1]+.15 or np.max(np.diff(ts))>2.5:
            review['reason']='size_evidence_does_not_cover_display_lifespan';continue
        depths=dimensions[model][2]/np.exp(smooth_curve(ts,obs[:,1],query))
        bearings=smooth_curve(ts,obs[:,2],query)
        old=[];new=[]
        half_length=float(dimensions[model][0])/2
        mount_yaw=math.radians(float(mount.get('yaw_deg',0)))
        offset=np.asarray(mount.get('offset_W_ego_m',[0,0,0]),float)
        for (i,a),depth,bearing in zip(entries,depths,bearings):
            s=states[i];yaw=s['ego_yaw_rad'];c,sn=math.cos(yaw),math.sin(yaw)
            R=np.array([[c,-sn,0],[sn,c,0],[0,0,1]])
            cm,sm=math.cos(mount_yaw),math.sin(mount_yaw)
            RM=np.array([[cm,-sm,0],[sm,cm,0],[0,0,1]])
            rel=RM@np.array([depth+half_length,-bearing*depth,0])+offset
            proposed=np.asarray(s['ego_position_W'],float)+R@rel;proposed[2]=a['position_W'][2]
            old.append(float((np.asarray(a['position_W'])-s['ego_position_W'])@R[:,0]-half_length))
            new.append(proposed.tolist())
        old=np.array(old)
        valid=np.r_[False,np.diff(query)<.11]
        rate_delta=np.abs(np.diff(old)-np.diff(depths))/np.maximum(np.diff(query),1e-6)
        mismatch=float(np.median(abs(old-depths)))
        rate_p95=float(np.percentile(rate_delta[valid[1:]],95)) if valid[1:].any() else 0.
        review.update(clear_size_observations=len(rows),median_contact_size_disagreement_m=mismatch,range_rate_disagreement_p95_mps=rate_p95)
        if mismatch<8 or rate_p95<6:
            review['reason']='no_sustained_far_contact_instability';continue
        # Transactional application: a candidate must not add new overlap events.
        before_pairs={(tuple(x['ids']),x['start_s'],x['end_s']) for x in collision_audit(states,dimensions)['body_overlap_intervals']}
        original=[dict(a) for i,a in entries]
        for (i,a),pos in zip(entries,new):a['position_W']=pos
        after_pairs={(tuple(x['ids']),x['start_s'],x['end_s']) for x in collision_audit(states,dimensions)['body_overlap_intervals']}
        if after_pairs-before_pairs:
            for (i,a),old_row in zip(entries,original):a.clear();a.update(old_row)
            review['reason']='candidate_introduces_body_overlap';continue
        trace=[]
        for (i,a),old_row,depth in zip(entries,original,depths):
            a['position_support']='offline_far_front_body_size_refinement'
            trace.append(dict(time_s=states[i]['time_s'],before_position_W=list(old_row['position_W']),after_position_W=a['position_W'],size_depth_m=float(depth)))
        review.update(status='applied',reason='far_contact_motion_disagrees_with_clear_size_evidence',changed_frames=len(entries),trace=trace,
                      scope='position_only; yaw_identity_ego_road_other_actors_unchanged; no_speed_cap',
                      uncertainty='class_height_and_estimated_intrinsics_scale_not_surveyed_3D_truth')
    return dict(policy='offline_far_front_size_refinement_v1',actors=reviews,raw_observations_preserved=True,ego_and_road_unchanged=True,
                applied_actor_ids=[r['id'] for r in reviews if r['status']=='applied'])
