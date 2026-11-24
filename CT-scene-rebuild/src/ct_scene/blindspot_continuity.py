"""Short display-only coasts at a pillar exit; never merge object identities.

Measured/estimated observations remain unchanged. A finite hold is drawn as a
transparent prediction and stops when a nearby real observation reappears.
"""
import math
import numpy as np
from .road_frame import coordinates, world
from .road_geometry import supported, lane_at
from .pillar_bridges import build as build_pillar_bridges


def _ego(times,positions,yaws,t):
    p=np.array([np.interp(t,times,positions[:,k]) for k in range(3)])
    return p,float(np.interp(t,times,np.unwrap(yaws)))


def build(actors,road,times,ego,yaws,policy=None):
    policy=policy or {};budget=float(policy.get('max_hold_s',2.5));result={};review=[]
    times=np.asarray(times);ego=np.asarray(ego);yaws=np.asarray(yaws)
    if not road.get('reference_line') or budget<=0:return result,review
    ego_station=coordinates(road,ego)[:,0]
    for actor in actors:
        if actor.get('frozen_lead'):continue
        rows=actor['samples'];observed=[r for r in rows if r.get('support') in ('detector_observation','flow_observation')]
        if len(observed)<5:continue
        anchor=observed[-1];camera=anchor.get('camera_id');last=rows[-1];t0=float(anchor['time_s']);end=min(t0+budget,float(times[-1]))
        if camera not in ('front','left_repeater','right_repeater') or end<=last['time_s']+.15:continue
        box=anchor.get('bbox_xyxy');width=anchor.get('image_width',1280)
        if box is None or min(box[0],width-box[2])>max(8.,width*.05):continue
        recent=[r for r in observed if r.get('camera_id')==camera and t0-1.4<=r['time_s']<=t0]
        if len(recent)<5 or t0-recent[0]['time_s']<.65:continue
        stamp=np.array([r['time_s']-t0 for r in recent]);local=coordinates(road,[r['position_W'] for r in recent]);X=np.c_[stamp,np.ones(len(stamp))]
        fit=np.linalg.lstsq(X,local[:,:2],rcond=None)[0];residual=float(np.quantile(np.linalg.norm(X@fit-local[:,:2],axis=1),.9));speed=float(fit[0,0])
        if residual>.8 or not 0<=speed<=55 or abs(fit[0,1])>.8:continue
        ep,eyaw=_ego(times,ego,yaws,t0);delta=np.asarray(anchor['position_W'])-ep
        longitudinal=float(np.cos(eyaw)*delta[0]+np.sin(eyaw)*delta[1]);lateral=float(-np.sin(eyaw)*delta[0]+np.cos(eyaw)*delta[1])
        if not -5<=longitudinal<=14 or not 1.4<=abs(lateral)<=8.5:continue
        future_yaw=np.interp([max(times[0],t0-1),end],times,np.unwrap(yaws))
        if abs(future_yaw[1]-future_yaw[0])>math.radians(8):continue
        ego_speed=(float(np.interp(t0,times,ego_station))-float(np.interp(max(times[0],t0-1),times,ego_station)))/max(.1,t0-max(times[0],t0-1))
        relative_speed=speed-ego_speed
        # A repeater exit approaches the pillar from behind. A front exit
        # approaches it from ahead as the ego passes. Do not retain a car
        # simply because it is moving out of the rear/distant field of view.
        if (camera=='front' and relative_speed>=-.3) or (camera!='front' and relative_speed<=.3):continue
        origin=coordinates(road,last['position_W']);lane=lane_at(road,float(origin[0]),float(origin[1]))
        if lane is None:continue
        predictions=[];stop_reason='hold_budget'
        coverage_end=float(last['time_s'])+float(policy.get('observation_half_step_s',.1))
        for now in np.arange(coverage_end,end+.00001,.05):
            station=float(origin[0]+speed*(now-coverage_end));point=np.r_[world(road,station,float(origin[1]))[:2],last['position_W'][2]]
            if not supported(road,station) or lane_at(road,station,float(origin[1]))!=lane:stop_reason='road_or_lane_support_ends';break
            ep,eyaw=_ego(times,ego,yaws,now);delta=point-ep
            rx=np.cos(eyaw)*delta[0]+np.sin(eyaw)*delta[1];ry=-np.sin(eyaw)*delta[0]+np.cos(eyaw)*delta[1]
            if not -6<=rx<=16 or not 1.2<=abs(ry)<=9:stop_reason='left_pillar_region';break
            replacement=False
            for other in actors:
                if other is actor:continue
                options=[r for r in other['samples'] if abs(r['time_s']-now)<=.12 and r.get('support') in ('detector_observation','flow_observation')]
                if any(np.linalg.norm(point[:2]-np.asarray(r['position_W'])[:2])<2.5 for r in options):replacement=True;break
            if replacement:stop_reason='nearby_real_observation';break
            predictions.append(dict(time_s=float(now),position_W=point.tolist(),yaw_rad=float(last['yaw_rad']),
                                    elapsed_since_observation_s=float(now-t0)))
        if predictions:
            result[actor['global_id']]=dict(model=actor['model'],samples=predictions)
            review.append(dict(global_id=actor['global_id'],camera=camera,anchor_frame_id=anchor['frame_id'],
                               last_observation_s=t0,start_s=predictions[0]['time_s'],end_s=predictions[-1]['time_s'],
                               budget_since_observation_s=budget,speed_mps=speed,relative_speed_mps=relative_speed,
                               fit_residual_p90_m=residual,stop_reason=stop_reason,identity_merged=False,
                               observations_modified=False,render_state='ghost'))
    bridges,bridge_review=build_pillar_bridges(actors,road,times,ego,yaws,policy)
    if bridges:
        result.update(bridges);review=[r for r in review if r['global_id'] not in bridges]+bridge_review
    return result,review


def at(coasts,t):
    result=[]
    for key,coast in coasts.items():
        rows=coast['samples'];ts=[r['time_s'] for r in rows]
        if not ts[0]-.025<=t<=ts[-1]+.025:continue
        p=[float(np.interp(t,ts,[r['position_W'][k] for r in rows])) for k in range(3)]
        yaw=float(np.interp(t,ts,np.unwrap([r['yaw_rad'] for r in rows])))
        result.append(dict(id=key,position_W=np.array(p),yaw_rad=yaw,model=coast['model'],
                           ghost=True,source_visible=False,override_unobserved=coast.get('override_unobserved',False),allow_visible_tail=bool(t>coast.get('visible_tail_after_s',float('inf'))),position_support='bounded_pillar_blindspot_prediction',
                           rear_appearance=dict(brake=None,left=None,right=None,rear_red_visible=None),collision_cap_m=0.))
    return result
