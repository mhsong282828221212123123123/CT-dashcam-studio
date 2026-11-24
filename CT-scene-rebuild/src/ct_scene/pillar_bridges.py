"""Future-supported display bridges across the unsensed pillar region.

Keep all source observations and object IDs. Only the absent interval and old
one-sided predictions may be replaced by a bounded two-endpoint path.
"""
import math
import numpy as np
from .clear_handoff import _window
from .identity_graph import select_handoffs
from .tracking import appearance_distance
from .road_frame import coordinates,world
from .road_geometry import lane_at,supported


def usable(row,w,h):
    box=row.get('bbox_xyxy')
    return bool(box and row.get('support')=='detector_observation' and
                min(box[0],box[1],w-box[2],h-box[3])>max(2,min(w,h)*.005) and
                box[2]-box[0]>=80 and box[3]-box[1]>=60)


def hermite(a,b,va,vb,dt,u):
    return (2*u**3-3*u**2+1)*a+(u**3-2*u**2+u)*va*dt+(-2*u**3+3*u**2)*b+(u**3-u**2)*vb*dt


def build(actors,road,times,ego,yaws,policy=None):
    policy=policy or {};maximum=float(policy.get('future_bridge_max_gap_s',9.));half=float(policy.get('observation_half_step_s',.1))
    times=np.asarray(times);ego=np.asarray(ego);yaws=np.unwrap(yaws);edges=[];paths={};review=[]
    proxies=[dict(a,id=a['global_id']) for a in actors]
    for first in actors:
        if first.get('frozen_lead'):continue
        observed=[r for r in first['samples'] if r.get('support') in ('detector_observation','flow_observation')]
        if len(observed)<8:continue
        anchor=observed[-1];camera=anchor.get('camera_id');box=anchor.get('bbox_xyxy');w=anchor.get('image_width',1280)
        if camera not in ('front','left_repeater','right_repeater') or not box:continue
        if camera=='left_repeater' and box[2]<w*.95:continue
        if camera=='right_repeater' and box[0]>w*.05:continue
        if camera=='front' and min(box[0],w-box[2])>w*.05:continue
        past=_window(first,True,usable)
        if past is None or anchor['time_s']-past['time_s']>.5:continue
        ep=np.array([np.interp(anchor['time_s'],times,ego[:,k]) for k in range(3)]);yaw=float(np.interp(anchor['time_s'],times,yaws));delta=np.asarray(anchor['position_W'])-ep
        rx=math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1];ry=-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1]
        if not (-7<rx<6 if camera!='front' else 0<rx<16) or not 1.4<abs(ry)<8.5:continue
        for second in actors:
            if second is first or second.get('frozen_lead') or first['end_s']>=second['start_s']:continue
            entry=second['samples'][0];gap=entry['time_s']-anchor['time_s']
            entry_camera=entry.get('camera_id')
            if not 3<gap<=maximum:continue
            if camera!='front' and entry_camera!='front':continue
            if camera=='front' and entry_camera not in ('left_repeater','right_repeater'):continue
            box2=entry.get('bbox_xyxy');w2=entry.get('image_width',1280)
            if not box2:continue
            if camera=='left_repeater' and box2[0]>w2*.02:continue
            if camera=='right_repeater' and box2[2]<w2*.98:continue
            if camera=='front' and entry_camera=='left_repeater' and (box[0]>w*.05 or box2[2]<w2*.98):continue
            if camera=='front' and entry_camera=='right_repeater' and (box[2]<w*.95 or box2[0]>w2*.02):continue
            if first['model']!=second['model']:continue
            app=appearance_distance(first.get('appearance',[]),second.get('appearance',[]))
            if app>=.65:continue
            future=_window(second,False,usable)
            if future is None or future['time_s']-entry['time_s']>3.2:continue
            if abs(float(np.interp(future['time_s'],times,yaws)-np.interp(anchor['time_s'],times,yaws)))>math.radians(10):continue
            a,b=coordinates(road,anchor['position_W']),coordinates(road,entry['position_W']);la,lb=lane_at(road,*map(float,a[:2])),lane_at(road,*map(float,b[:2]))
            if la is None or la<0 or la!=lb or abs(a[1]-b[1])>1.2 or b[0]-a[0]<8:continue
            va=float(np.array(past['velocity_W'][:2])@np.array([math.cos(a[2]),math.sin(a[2])]))
            vb=float(np.array(future['velocity_W'][:2])@np.array([math.cos(b[2]),math.sin(b[2])]))
            if not 2<min(va,vb)<=max(va,vb)<55 or abs(va-vb)>6:continue
            begin=float(anchor['time_s']+half);end=float(entry['time_s']-half);dt=end-begin
            stamps=np.r_[np.arange(begin,end,.05),end];u=(stamps-begin)/dt
            stations=hermite(a[0],b[0],va,vb,dt,u);lateral=a[1]+(b[1]-a[1])*(3*u*u-2*u*u*u)
            speed=np.gradient(stations,stamps);acceleration=np.gradient(speed,stamps)
            peak=float(max(abs(acceleration)))
            if min(speed)<0 or max(speed)>55 or peak>4.5:continue
            if any(not supported(road,float(s)) or lane_at(road,float(s),float(d))!=la for s,d in zip(stations,lateral)):continue
            points=np.array([np.r_[world(road,float(s),float(d))[:2],0.] for s,d in zip(stations,lateral)])
            start_body=float(anchor['yaw_rad']);delta_yaw=math.atan2(math.sin(entry['yaw_rad']-start_body),math.cos(entry['yaw_rad']-start_body))
            if abs(delta_yaw)>math.radians(25):continue
            samples=[dict(time_s=float(t),position_W=p.tolist(),yaw_rad=float(start_body+delta_yaw*v)) for t,p,v in zip(stamps,points,u)]
            cost=.5*app/.65+.3*peak/4.5+.2*gap/maximum
            edge=dict(from_id=first['global_id'],to_id=second['global_id'],cost=cost)
            edge.update({'from':edge.pop('from_id'),'to':edge.pop('to_id'),'identity_eligible':True})
            edges.append(edge);paths[first['global_id'],second['global_id']]=dict(model=first['model'],samples=samples,override_unobserved=True,visible_tail_after_s=float(anchor['time_s']+.12))
            review.append(dict(global_id=first['global_id'],future_global_id=second['global_id'],kind='two_sided_pillar_bridge',from_camera=camera,to_camera=entry_camera,
                               last_observation_s=anchor['time_s'],future_entry_s=entry['time_s'],start_s=begin,end_s=end,
                               anchor_frame_id=anchor['frame_id'],future_frame_id=entry['frame_id'],appearance_distance=app,
                               max_longitudinal_acceleration_mps2=peak,max_speed_mps=float(max(speed)),lane=la,
                               observations_modified=False,identity_merged=False,render_state='ghost',cost=cost))
    select_handoffs(edges,proxies,margin=.15)
    selected={(e['from'],e['to']) for e in edges if e['selected']};result={};accepted=[]
    for row in review:
        pair=row['global_id'],row['future_global_id']
        if pair in selected:result[pair[0]]=paths[pair];accepted.append(row)
    return result,accepted
