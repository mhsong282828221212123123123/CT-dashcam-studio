"""Conditional side-ground alignment using road paint, never vehicle occupancy."""
from collections import defaultdict
from pathlib import Path
import cv2,numpy as np
from .road_frame import frame_for,coordinates,world
from .road_geometry import boundaries_at,lane_at,lane_bounds,supported


def fit_range_shear(records):
    """One stripe at different ranges can constrain a small yaw-like error.

    Keep height scale fixed; range variation, separate later-time validation
    and bounded angular shear are required. Cars are never fitting controls.
    """
    result=dict(accepted=False,status='insufficient_range_separated_paint',records=records)
    if len(records)<40 or len({r['frame_id'] for r in records})<12 or not all('relative_station_m' in r for r in records):return result
    rows=sorted(records,key=lambda r:r['time_s']);cut=rows[int(len(rows)*.67)]['time_s']
    train=[r for r in rows if r['time_s']<cut];test=[r for r in rows if r['time_s']>=cut]
    if len(train)<20 or len(test)<10:return result
    distances=np.array([r['relative_station_m'] for r in train])
    # At least one nominal lane width of longitudinal lever arm is required.
    if np.ptp(distances)<3.5:return result
    X=np.c_[distances,np.ones(len(train))];target=np.array([r['target_d']-r['measured_d'] for r in train]);coeff=np.linalg.lstsq(X,target,rcond=None)[0]
    for _ in range(10):
        err=target-X@coeff;weight=np.minimum(1,.2/np.maximum(abs(err),1e-6));coeff=np.linalg.lstsq(X*weight[:,None],target*weight,rcond=None)[0]
    shear,offset=map(float,coeff);errors=np.array([abs(r['measured_d']+shear*r['relative_station_m']+offset-r['target_d']) for r in test]);raw=np.array([abs(r['measured_d']-r['target_d']) for r in test])
    metrics=dict(held_out_median_m=float(np.median(errors)),held_out_p90_m=float(np.quantile(errors,.9)),uncorrected_held_out_median_m=float(np.median(raw)))
    accepted=abs(shear)<=.25 and abs(offset)<=2. and np.median(errors)<=.35 and np.quantile(errors,.9)<=.7 and np.median(errors)<np.median(raw)*.75
    result.update(accepted=bool(accepted),status='range_supported_camera_ground_shear' if accepted else 'range_shear_held_out_gate_failed',
                  model_type='relative_station_shear',scale=1.,range_shear=shear,offset_m=offset,metrics=metrics,
                  held_out_start_s=cut,relative_station_training_range_m=[float(min(distances)),float(max(distances))],
                  vehicle_positions_used=False,metric_calibration=False)
    return result


def fit(records):
    result=dict(accepted=False,status='insufficient_independent_paint',records=records)
    if len(records)<40 or len({r['frame_id'] for r in records})<12:return result
    if any(len({r['frame_id'] for r in records if r['kind']==kind})<6 for kind in ('near','far')):return fit_range_shear(records)
    rows=sorted(records,key=lambda r:r['time_s']);cut=rows[int(len(rows)*.67)]['time_s']
    train=[r for r in rows if r['time_s']<cut];test=[r for r in rows if r['time_s']>=cut]
    if len(train)<20 or len(test)<10:return result
    X=np.array([[r['measured_d'],1.] for r in train]);y=np.array([r['target_d'] for r in train])
    if np.ptp(X[:,0])<2.:return result
    coeff=np.linalg.lstsq(X,y,rcond=None)[0]
    for _ in range(10):
        err=y-X@coeff;weight=np.minimum(1,.2/np.maximum(abs(err),1e-6))
        coeff=np.linalg.lstsq(X*weight[:,None],y*weight,rcond=None)[0]
    a,b=map(float,coeff);held=np.array([[r['measured_d'],r['target_d']] for r in test])
    error=abs(held[:,0]*a+b-held[:,1]);raw=abs(held[:,0]-held[:,1])
    metrics=dict(held_out_median_m=float(np.median(error)),held_out_p90_m=float(np.quantile(error,.9)),
                 uncorrected_held_out_median_m=float(np.median(raw)))
    accepted=.8<=a<=1.25 and abs(b)<=2. and metrics['held_out_median_m']<=.35 and metrics['held_out_p90_m']<=.7 and np.median(error)<np.median(raw)*.75
    result.update(accepted=bool(accepted),status='camera_paint_relative_alignment' if accepted else 'held_out_paint_or_parameter_gate_failed',
                  scale=a,offset_m=b,metrics=metrics,held_out_start_s=cut,vehicle_positions_used=False,
                  metric_calibration=False,scope='estimated side ground to existing front road; not measured rig calibration')
    return result


def extract(package,frames,observations,times,ego,yaws,profiles,mounts,road,project_contact,rotation):
    if not road.get('reference_line') or road.get('lane_geometry_status')=='unknown_route_display_only':return {}
    rf=frame_for(road);out={}
    for camera in ('left_repeater','right_repeater'):
        records=[];left=camera=='left_repeater'
        for f in sorted([f for f in frames if f['camera_id']==camera],key=lambda f:f['camera_time_s'])[::2]:
            t=f['camera_time_s'];yaw=np.interp(t,times,yaws);ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)])
            q=rf.project(ep);bounds=boundaries_at(road,float(q[0]));indices=[i for i,b in enumerate(bounds) if b>q[1]+.7] if left else [i for i,b in enumerate(bounds) if b<q[1]-.7]
            if len(indices)<2 or min(abs(np.array(bounds)-q[1]))<.7 or lane_at(road,float(q[0]),float(q[1])) is None:continue
            near=max(indices) if left else min(indices);far=near-1 if left else near+1
            if not 0<=far<len(bounds):continue
            image=cv2.imread(str(Path(package)/f['image']));h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);v=hsv[:,:,2]
            contrast=cv2.subtract(v,cv2.GaussianBlur(v,(0,0),7));mask=((hsv[:,:,1]<60)&(v>115)&(contrast>14)).astype('uint8')*255
            mask[:round(h*.52)]=0;mask[round(h*.94):]=0
            if left:mask[:,:55]=0
            else:mask[:,w-55:]=0
            for r in observations.get(f['frame_id'],[]):
                x,y,X,Y=np.rint(r['bbox_xyxy']).astype(int);mask[max(0,y-5):min(h,Y+5),max(0,x-5):min(w,X+5)]=0
            lines=cv2.HoughLinesP(mask,1,np.pi/180,35,minLineLength=90,maxLineGap=18)
            if lines is None:continue
            for line in lines[:,0,:]:
                dx,dy=line[2]-line[0],line[3]-line[1]
                if dx==0:continue
                slope=dy/dx*(1 if left else -1)
                if slope<.07 or .3<slope<.4:continue
                kind='near' if slope>=.4 else 'far';idx=near if kind=='near' else far;points=[]
                try:
                    for x,y in (line[:2],line[2:]):points.append(ep+rotation(yaw)@project_contact([x,y,x,y],profiles[camera],mounts[camera]))
                except ValueError:continue
                local=rf.project(points);delta=local[1]-local[0]
                if abs(delta[1])>.2*abs(delta[0]) or abs(delta[0])<2:continue
                targets=[]
                for station in local[:,0]:
                    bb=boundaries_at(road,float(station))
                    if idx>=len(bb):break
                    targets.append(bb[idx])
                if len(targets)!=2:continue
                measured=float(np.mean(local[:,1]));target=float(np.mean(targets))
                if abs(measured-target)>2.:continue
                records.append(dict(frame_id=f['frame_id'],time_s=t,kind=kind,measured_d=measured,target_d=target,
                                    relative_station_m=float(np.mean(local[:,0])-q[0]),pixels=line.tolist()))
        out[camera]=fit(records)
        if out[camera].get('accepted') and out[camera].get('model_type')=='relative_station_shear':
            out[camera]['ego_station_trace']=dict(times_s=np.asarray(times).tolist(),stations_m=rf.project(ego)[:,0].tolist())
    return out


def apply(actors,road,models):
    review=[];original_by_id={a['id']:list(a['samples']) for a in actors}
    for actor in actors:
        model=models.get(actor['camera_id'],{})
        if actor.get('frozen_lead') or not model.get('accepted'):continue
        original_samples=actor['samples'];rows=[r for r in original_samples if r.get('side_ground_contact_W') is not None]
        if len(rows)<12 or rows[-1]['time_s']-rows[0]['time_s']<2.4:continue
        local=coordinates(road,[r['side_ground_contact_W'] for r in rows]);local[:,1]=local[:,1]*model['scale']+model['offset_m']
        if model.get('model_type')=='relative_station_shear':
            trace=model['ego_station_trace'];own_station=np.interp([r['time_s'] for r in rows],trace['times_s'],trace['stations_m'])
            local[:,1]+=model['range_shear']*(local[:,0]-own_station)
        votes=[lane_at(road,float(s),float(d)) for s,d,_ in local];known=[v for v in votes if v is not None and v>=0]
        if not known:continue
        lane=max(set(known),key=known.count)
        if known.count(lane)<len(rows)*.8:continue
        accepted=[]
        for r,(s,d,_),vote in zip(rows,local,votes):
            upper,lower=lane_bounds(road,float(s),lane)
            if not lower-.5<=d<=upper+.5:continue
            point=np.r_[world(road,float(s),float(d))[:2],0.].tolist()
            accepted.append(dict(r,position_before_side_paint_W=r['position_W'],position_W=point,unrefined_position_W=point,
                                 paint_lane_index=lane,paint_lane_supported=True,
                                 projection_support='direct_side_ground_with_held_out_paint_alignment'))
        if len(accepted)<12:continue
        if model.get('model_type')=='relative_station_shear':
            # A clear front/side handoff may include clipped prefixes and tiny
            # later boxes whose bottom is unusable. Reconstruct only short
            # missing-contact portions of the SAME tracked camera identity.
            accepted_local=coordinates(road,[r['position_W'] for r in accepted]);tt=np.array([r['time_s'] for r in accepted]);design=np.c_[tt-tt[0],np.ones(len(tt))]
            coef=np.linalg.lstsq(design,accepted_local[:,0],rcond=None)[0]
            for _ in range(8):
                err=accepted_local[:,0]-design@coef;ww=np.minimum(1,.5/np.maximum(abs(err),1e-8));coef=np.linalg.lstsq(design*ww[:,None],accepted_local[:,0]*ww,rcond=None)[0]
            if 5<=coef[0]<=55 and np.quantile(abs(accepted_local[:,0]-design@coef),.9)<=2.:
                known={r['frame_id'] for r in accepted};extra=[]
                for r in original_samples:
                    t=r['time_s']
                    if r['frame_id'] in known or max(tt[0]-t,t-tt[-1],0)>4.:continue
                    station=float(np.interp(t,tt,accepted_local[:,0])) if tt[0]<=t<=tt[-1] else float(np.array([t-tt[0],1.])@coef)
                    upper,lower=lane_bounds(road,station,lane);point=np.r_[world(road,station,(upper+lower)/2)[:2],0.].tolist()
                    extra.append(dict(r,position_before_side_paint_W=r['position_W'],position_W=point,unrefined_position_W=point,
                                      paint_lane_index=lane,paint_lane_supported=True,support='offline_occluded_contact_reconstruction',
                                      projection_support='calibrated_same_camera_short_contact_gap',uses_future_observations=True))
                accepted=sorted(accepted+extra,key=lambda r:r['time_s'])
        actor['samples']=accepted;actor['side_paint_geometry']=dict(camera=actor['camera_id'],lane=lane,observations=len(accepted),
            scale=model['scale'],offset_m=model['offset_m'],vehicle_used_to_fit_camera=False,
            model_type=model.get('model_type','two_stripe_affine'),range_shear=model.get('range_shear',0.))
        review.append(dict(id=actor['id'],**actor['side_paint_geometry']))
    # A foreground body can hide a following truck's wheels. Its detector
    # bottom then gives a one-sided, outward-biased contact near a divider.
    # Use repeated same-image occlusion plus an independently paint-supported
    # foreground lane, only inside a sub-metre boundary uncertainty band.
    for rear in actors:
        if rear.get('model') not in ('truck','box_truck','heavy_truck') or not rear.get('side_paint_geometry'):continue
        votes=[];samples=rear['samples'];side=rear['camera_id'];censored={}
        for front in actors:
            if front is rear or front['camera_id']!=side or not front.get('side_paint_geometry'):continue
            lane=front['side_paint_geometry']['lane']
            if abs(lane-rear['side_paint_geometry']['lane'])!=1:continue
            front_by_frame={r['frame_id']:r for r in front['samples']}
            for r in samples:
                f=front_by_frame.get(r['frame_id'])
                if not f:continue
                a,b=np.array(r['bbox_xyxy']),np.array(f['bbox_xyxy']);width=a[2]-a[0]
                overlap=min(a[2],b[2])-max(a[0],b[0])
                if overlap<width*.25 or b[3]<a[3]-2:continue
                censored[r['frame_id']]=dict(lane=lane,overlap_fraction=float(overlap/width),bottom_gap=float(b[3]-a[3]))
                if b[3]<a[3]+5:continue
                q=coordinates(road,r['unrefined_position_W']);upper,lower=lane_bounds(road,float(q[0]),lane)
                if not lower-.75<=q[1]<=upper+.75:continue
                votes.append(dict(lane=lane,frame_id=r['frame_id'],time_s=r['time_s'],foreground_id=front['id']))
        if len(votes)<6:continue
        lane=max(set(v['lane'] for v in votes),key=lambda k:sum(v['lane']==k for v in votes));valid=[v for v in votes if v['lane']==lane]
        if len({v['frame_id'] for v in valid})<6 or max(v['time_s'] for v in valid)-min(v['time_s'] for v in valid)<1.2 or len(valid)/len(votes)<.9:continue
        compatible=[];anchors=[]
        for r in samples:
            q=coordinates(road,r['unrefined_position_W']);upper,lower=lane_bounds(road,float(q[0]),lane)
            if lower-.75<=q[1]<=upper+.75:
                compatible.append(r)
                c=censored.get(r['frame_id'],{})
                if c.get('lane')==lane and c.get('overlap_fraction',1)<.6 and c.get('bottom_gap',0)>=5:anchors.append((r,float(q[0])))
        if len(compatible)<len(samples)*.8:
            # Stronger later overlap may censor the contact completely. Only
            # short, source-visible occlusions can use the earlier stable lane
            # and robust longitudinal motion; never extend an unseen long gap.
            if len(anchors)<8 or anchors[-1][0]['time_s']-anchors[0][0]['time_s']<1.2:continue
            t0=anchors[0][0]['time_s'];X=np.array([[r['time_s']-t0,1.] for r,s in anchors]);y=np.array([s for r,s in anchors]);coef=np.linalg.lstsq(X,y,rcond=None)[0]
            for _ in range(8):
                err=y-X@coef;w=np.minimum(1,.5/np.maximum(abs(err),1e-8));coef=np.linalg.lstsq(X*w[:,None],y*w,rcond=None)[0]
            # Same 2m longitudinal residual budget as side-contact recovery;
            # no metric precision claim for a monocular, partly hidden body.
            if not 5<=coef[0]<=55 or np.quantile(abs(y-X@coef),.9)>2.:continue
            known={r['frame_id'] for r in compatible};extended=[]
            for r in samples:
                c=censored.get(r['frame_id']);time=r['time_s']
                if r['frame_id'] in known and (not c or c['overlap_fraction']<.6):extended.append(r);continue
                if not c or c['lane']!=lane or max(t0-time,time-anchors[-1][0]['time_s'],0)>4.:continue
                station=float(np.array([time-t0,1.])@coef);upper,lower=lane_bounds(road,station,lane);point=np.r_[world(road,station,(upper+lower)/2)[:2],0.].tolist()
                extended.append(dict(r,position_W=point,unrefined_position_W=point,position_before_occluded_contact=r['position_W'],
                                     support='offline_occluded_contact_reconstruction',uses_future_observations=True,
                                     projection_support='paint_lane_and_short_source_visible_occlusion_motion'))
            if len(extended)<len(samples)*.9:continue
            compatible=extended
        rear['samples']=[dict(r,paint_lane_index=lane,paint_contact_censored=True) for r in compatible]
        rear['side_paint_geometry'].update(lane=lane,occluded_contact_lane_supported=True,occlusion_evidence=valid)
        for record in review:
            if record['id']==rear['id']:record.update(rear['side_paint_geometry'])
    for actor in actors:
        count=complete_near_camera_edges(actor,road,original_by_id[actor['id']])
        if count:
            for record in review:
                if record['id']==actor['id']:record['near_edge_completed_samples']=count
    return review


def complete_near_camera_edges(actor,road,original_samples,max_gap_s=2.):
    """Keep a large, visibly clipped SAME camera identity through its edge.

    No new detection/identity is created. Distant small boxes, free lane changes,
    uncalibrated cameras and long unknown gaps are explicitly outside this path.
    """
    evidence=actor.get('side_paint_geometry',{})
    if actor.get('frozen_lead') or evidence.get('model_type')!='two_stripe_affine' or evidence.get('occluded_contact_lane_supported'):return 0
    rows=actor['samples'];clear=[r for r in rows if r.get('support')=='detector_observation' and r.get('paint_lane_supported') and not r.get('paint_contact_censored')]
    if len(clear)<5:return 0
    lane=evidence['lane'];known={r['frame_id'] for r in rows};added=[]
    for before in (True,False):
        window=clear[:6] if before else clear[-6:];anchor=window[0] if before else window[-1]
        tt=np.array([r['time_s'] for r in window]);local=coordinates(road,[r['position_W'] for r in window])
        if len(tt)<5 or np.ptp(tt)<.6:continue
        X=np.c_[tt-anchor['time_s'],np.ones(len(tt))];fit=np.linalg.lstsq(X,local[:,:2],rcond=None)[0];speed=float(fit[0,0])
        if not -.5<=speed<=55 or abs(fit[0,1])>.5 or np.quantile(abs(local[:,0]-(X@fit)[:,0]),.9)>.5:continue
        origin=coordinates(road,anchor['position_W']);speed=0. if abs(speed)<.5 else speed
        for r in original_samples:
            t=r['time_s'];dt=t-anchor['time_s']
            if r['frame_id'] in known or (dt>=0 if before else dt<=0) or abs(dt)>max_gap_s:continue
            box=r.get('bbox_xyxy');w=r.get('image_width',1280);h=r.get('image_height',960)
            if not box or box[2]-box[0]<180 or box[3]-box[1]<100 or min(box[0],box[1],w-box[2],h-box[3])>8:continue
            s=float(origin[0]+speed*dt);d=float(origin[1])
            if not supported(road,s):continue
            bounds=boundaries_at(road,s)
            if not 0<=lane<len(bounds)-1:continue
            upper,lower=lane_bounds(road,s,lane)
            if not lower<=d<=upper:continue
            point=np.r_[world(road,s,d)[:2],0.].tolist()
            added.append(dict(r,position_before_camera_edge_completion_W=r['position_W'],position_W=point,unrefined_position_W=point,
                              paint_lane_index=lane,paint_lane_supported=True,support='offline_camera_edge_prediction',
                              projection_support='short_large_clipped_body_from_same_camera_identity',
                              geometry_anchor_frame_ids=[q['frame_id'] for q in window],uses_future_observations=before))
            known.add(r['frame_id'])
    if added:
        actor['samples']=sorted(rows+added,key=lambda r:r['time_s'])
        actor['near_camera_edge_completion']=dict(added_samples=len(added),max_gap_s=max_gap_s,source_visible_large_body_only=True,new_identity_created=False)
    return len(added)


def extend_calibrated_tail(actor,road,end_time,max_tail_s=.8):
    """Brief post-observation coast from a paint-calibrated side identity."""
    if actor.get('frozen_lead') or not actor.get('calibrated_tail_camera_ids'):return False
    rows=actor['samples'];last=rows[-1];gap=end_time-last['time_s']
    if last.get('camera_id') not in actor['calibrated_tail_camera_ids'] or not 0<gap<=max_tail_s:return False
    recent=[r for r in rows if r['time_s']>=last['time_s']-1.2]
    if len(recent)<5:return False
    local=coordinates(road,[r['position_W'] for r in recent]);t=np.array([r['time_s']-last['time_s'] for r in recent]);X=np.c_[t,np.ones(len(t))];coef=np.linalg.lstsq(X,local[:,0],rcond=None)[0]
    if not 5<=coef[0]<=55 or np.quantile(abs(local[:,0]-X@coef),.9)>.5:return False
    lane=lane_at(road,float(local[-1,0]),float(local[-1,1]))
    if lane is None or lane<0:return False
    new=[]
    for now in [*np.arange(last['time_s']+.2,end_time,.2),end_time]:
        s=float(local[-1,0]+coef[0]*(now-last['time_s']))
        if not supported(road,s):return False
        b=boundaries_at(road,s)
        if lane>=len(b)-1:return False
        d=float(local[-1,1]);upper,lower=lane_bounds(road,s,lane)
        if not lower<=d<=upper:return False
        point=np.r_[world(road,s,d)[:2],0.].tolist();yaw=float(coordinates(road,point)[2])
        new.append(dict(last,time_s=float(now),frame_id=f'prediction:{actor["global_id"]}:{now:.6f}',
                        position_W=point,unrefined_position_W=point,yaw_rad=yaw,
                        support='offline_camera_tail_prediction',projection_support='short_calibrated_identity_coast',
                        evidence_frame_id=last['frame_id'],uses_future_observations=False))
    rows.extend(new);actor['end_s']=float(end_time)
    actor['tail_prediction']=dict(start_s=last['time_s'],end_s=end_time,max_budget_s=max_tail_s,source_observation_extended=False)
    return True
