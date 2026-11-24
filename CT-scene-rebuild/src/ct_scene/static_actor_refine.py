"""Conditional parked-car pose from source mesh reprojection across cameras.

No rig, ego, scale or identity fitting. A later source interval is withheld;
failed hypotheses never modify the actor. Frozen lead/lane priors are excluded.
"""
import numpy as np
import cv2
from .prototype_front import rotation
from .affine_probe import camera_matrix


def projected_box(vertices,position,yaw,camera_position,camera_yaw,K,D):
    phi=camera_yaw;basis=np.array([[np.sin(phi),-np.cos(phi),0],[0,0,-1],[np.cos(phi),np.sin(phi),0]])
    v=(vertices@rotation(yaw).T+position-camera_position)@basis.T
    if v[:,2].min()<=.1:return None
    uv=cv2.projectPoints(v,np.zeros(3),np.zeros(3),K,D)[0][:,0,:]
    return np.r_[uv.min(0),uv.max(0)]


def fit_pose(vertices,samples,seed,yaw):
    if len(samples)<15:return dict(accepted=False,reason='insufficient_source_boxes')
    if len({r['camera_id'] for r in samples})<2:return dict(accepted=False,reason='no_two_camera_mesh_baseline')
    end=samples[0]['time_s']+.73*(samples[-1]['time_s']-samples[0]['time_s']);train=[r for r in samples if r['time_s']<end];test=[r for r in samples if r['time_s']>=end]
    if len(test)<5 or any(sum(r['camera_id']==c for r in train)<3 for c in {r['camera_id'] for r in samples}):return dict(accepted=False,reason='insufficient_independent_later_interval_or_camera_baseline')
    def residual(x,rows,censored=True):
        out=[]
        for r in rows:
            b=np.array(r['box']);q=projected_box(vertices,np.r_[x[:2],0],x[2],r['camera_position'],r['camera_yaw'],r['K'],r['D'])
            error=(q-b)/np.tile(np.maximum(b[2:]-b[:2],1),2) if q is not None else np.full(4,100.)
            # A curb/foreground can hide the lower body. The mesh must reach
            # every visible bottom pixel, but a detector box ending above the
            # hidden wheels is not a measured ground-contact point. Keep roof
            # and BOTH horizontal extents as equalities at every source time.
            if censored and q is not None:error[3]=min(error[3],0.)
            out.append(error)
        return np.array(out)
    def cost(x):
        r=residual(x,train);return np.mean(np.sqrt(1+(r/.1)**2)-1)
    x=np.r_[seed[:2],yaw];lo=x-np.array([3,3,np.radians(15)]);hi=x+np.array([3,3,np.radians(15)])
    # Joint robust least squares avoids a coordinate-descent pose/yaw trap.
    # Only three object parameters are optimized; the fixed rig is untouched.
    for _ in range(30):
        r=residual(x,train).ravel();J=[]
        for k,h in enumerate([.001,.001,.0001]):
            offset=np.eye(3)[k]*h;J.append((residual(x+offset,train).ravel()-residual(x-offset,train).ravel())/(2*h))
        J=np.array(J).T;weight=(1+(r/.1)**2)**(-.25)
        step=np.linalg.lstsq(np.r_[J*weight[:,None],np.eye(3)*.01],np.r_[-r*weight,np.zeros(3)],rcond=None)[0];step=np.clip(step,[-1,-1,-.1],[1,1,.1])
        choices=[x]+[np.clip(x+fraction*step,lo,hi) for fraction in [1,.5,.25,.125]];best=min(choices,key=cost)
        if np.linalg.norm(best-x)<1e-6:break
        x=best
    for size in [1,.5,.2,.1,.05,.02]:
        step=np.array([size,size,size*.1])
        for _ in range(20):
            candidates=[x]+[np.clip(x+sign*step*np.eye(3)[i],lo,hi) for i in range(3) for sign in [-1,1]];best=min(candidates,key=cost)
            if np.array_equal(best,x):break
            x=best
    tr,te=residual(x,train),residual(x,test)
    # No error threshold is chosen from a filename, plate, map or target car.
    metrics=dict(train_median=float(np.median(abs(tr))),held_out_median=float(np.median(abs(te))),held_out_p90=float(np.quantile(abs(te),.9)),held_out_worst_mean=float(np.max(np.mean(abs(te),axis=1))))
    full=residual(x,test,censored=False);metrics['uncensored_full_box_held_out_p90']=float(np.quantile(abs(full),.9))
    accepted=metrics['train_median']<=.12 and metrics['held_out_median']<=.12 and metrics['held_out_p90']<=.25 and metrics['held_out_worst_mean']<=.25
    return dict(accepted=bool(accepted),reason='conditional_static_mesh_reprojection' if accepted else 'later_source_reprojection_conflicts_with_static_pose',
                position_W=[float(x[0]),float(x[1]),0.],yaw_rad=float(x[2]),train_frame_ids=[r['frame_id'] for r in train],held_out_frame_ids=[r['frame_id'] for r in test],
                valid_time_range_s=[samples[0]['time_s'],samples[-1]['time_s']],metrics=metrics,ego_or_rig_or_scale_modified=False,
                measurement_model='roof_and_both_horizontal_extents; visible_bottom_is_censored_by_possible_foreground; conditional_not_metric')


def refine_actor(actor,heading_review,frames,profiles,mounts,ego_times,ego_positions,ego_yaws,vertices,gap_budget_s=4.):
    if actor.get('frozen_lead') or actor.get('lane_index')!=-2 or actor.get('model') not in ('sedan','suv','van'):return dict(accepted=False,reason='protected_actor_or_lane_geometry')
    rows=actor['samples'];stationary=[r for r in rows if r.get('stationary_handoff_support')]
    anchors=heading_review.get('anchors',[])
    if len(stationary)<3 or len(anchors)<3:return dict(accepted=False,reason='no_independent_stationary_handoff_and_image_body_heading')
    angles=np.array([r['yaw_rad'] for r in anchors]);angle=float(np.angle(np.mean(np.exp(1j*angles))))
    if np.quantile(abs(np.angle(np.exp(1j*(angles-angle)))),.9)>np.radians(15):return dict(accepted=False,reason='source_body_angle_changes')
    samples=[];last_by_camera={};times=np.asarray(ego_times);own=np.asarray(ego_positions);yaws=np.unwrap(ego_yaws)
    for r in rows:
        b=r.get('bbox_xyxy');f=frames.get(r.get('frame_id'));c=r.get('camera_id')
        if not f or c not in mounts or r['support'] not in ('detector_observation','offline_bidirectional_clear_geometry') or not b:continue
        if min(b[0],b[1],f['width']-b[2],f['height']-b[3])<=5 or b[2]-b[0]<80 or b[3]-b[1]<60:continue
        if r['time_s']-last_by_camera.get(c,-np.inf)<.5:continue
        last_by_camera[c]=r['time_s'];t=r['time_s'];ey=np.interp(t,times,yaws);ep=np.array([np.interp(t,times,own[:,k]) for k in range(3)]);m=mounts[c]
        cp=ep+rotation(ey)@np.array(m['offset_W_ego_m']);cp[2]+=m['height_m'];K,D=camera_matrix(profiles[c])
        samples.append(dict(frame_id=f['frame_id'],time_s=t,camera_id=c,box=b,camera_position=cp,camera_yaw=ey+np.radians(m['yaw_deg']),K=K,D=D))
    if len({r['camera_id'] for r in samples})<2:return dict(accepted=False,reason='no_two_camera_mesh_baseline')
    seed=np.median([r['position_W'] for r in stationary],axis=0);review=fit_pose(vertices,samples,seed,angle)
    if not review['accepted']:return review
    lo,hi=review['valid_time_range_s'];changed=0;additional=[]
    for row in rows:
        weight=1.
        if not lo-gap_budget_s<=row['time_s']<=hi+gap_budget_s:
            # A distant small but still observed body can precede the clear
            # fitting window. Extend only when its OWN source pixels agree;
            # never extend a blind interval beyond the ordinary gap budget.
            b=row.get('bbox_xyxy');f=frames.get(row.get('frame_id'));c=row.get('camera_id')
            if not f or not b or row['support'] not in ('detector_observation','flow_observation') or c not in mounts:continue
            t=row['time_s'];ey=np.interp(t,times,yaws);ep=np.array([np.interp(t,times,own[:,k]) for k in range(3)]);m=mounts[c];cp=ep+rotation(ey)@np.array(m['offset_W_ego_m']);cp[2]+=m['height_m'];K,D=camera_matrix(profiles[c])
            q=projected_box(vertices,np.array(review['position_W']),review['yaw_rad'],cp,ey+np.radians(m['yaw_deg']),K,D)
            if q is None:continue
            error=q-np.array(b);error[3]=min(error[3],0.);tolerance=5+np.tile(np.maximum(np.array(b[2:])-b[:2],1),2)*.25
            if np.any(abs(error)>tolerance):continue
            additional.append(f['frame_id'])
        else:
            # The finite inference window must not introduce a position jump
            # at its boundary. Clear fitted times retain the exact static pose.
            edge=min(row['time_s']-(lo-gap_budget_s),(hi+gap_budget_s)-row['time_s']);u=float(np.clip(edge/min(1.,gap_budget_s),0,1)) if gap_budget_s>0 else 1.;weight=u*u*(3-2*u)
        if weight<=0:continue
        old=np.array(row['position_W']);angle=float(np.angle(np.exp(1j*(review['yaw_rad']-row['yaw_rad']))))
        row.update(position_before_static_mesh_fit=row['position_W'],yaw_before_static_mesh_fit=row['yaw_rad'],position_W=(old+weight*(np.array(review['position_W'])-old)).tolist(),yaw_rad=row['yaw_rad']+weight*angle,
                   geometry_reconstruction='conditional_static_mesh; later_source_reprojection_validated',static_mesh_weight=weight,heading_support='conditional_static_mesh_orientation',uses_future_observations=True)
        changed+=1
    review['changed_samples']=changed;review['additional_source_reprojection_supported_frame_ids']=additional;return review
