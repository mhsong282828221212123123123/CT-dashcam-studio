"""Two clear camera windows constrain clipped exit/entry contacts offline.

Identity selection still requires FOV, appearance and exclusive matching.
Fits are evaluated before geometry changes; no camera offset is estimated.
"""
import numpy as np


def _window(actor,past,usable):
    clear=[r for r in actor['samples'] if usable(r,r.get('image_width',1280),r.get('image_height',960)) and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=80 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=60]
    if not clear:return None
    boundary=clear[-1]['time_s'] if past else clear[0]['time_s']
    rows=[r for r in clear if abs(r['time_s']-boundary)<=1.2]
    if len({r['frame_id'] for r in rows})<5 or rows[-1]['time_s']-rows[0]['time_s']<.8:return None
    t=np.array([r['time_s']-boundary for r in rows]);p=np.array([r['position_W'] for r in rows]);X=np.c_[t,np.ones(len(t))];fit=np.linalg.lstsq(X,p,rcond=None)[0]
    residual=float(np.quantile(np.linalg.norm(X@fit-p,axis=1),.9));speed=float(np.linalg.norm(fit[0,:2]))
    if residual>.75 or speed>45:return None
    return dict(time_s=boundary,position_W=fit[1].tolist(),velocity_W=fit[0].tolist(),residual_p90_m=residual,
                source_frame_ids=[r['frame_id'] for r in rows],span_s=[rows[0]['time_s'],rows[-1]['time_s']])


def fit(first,second,usable):
    pair={first['camera_id'],second['camera_id']}
    if 'front' not in pair or not pair.intersection({'left_repeater','right_repeater'}):return None
    if not -.4<=second['start_s']-first['end_s']<=4:return None
    past,future=_window(first,True,usable),_window(second,False,usable)
    if not past or not future:return None
    if first['end_s']-past['time_s']>3.2 or future['time_s']-second['start_s']>3.2 or future['time_s']-past['time_s']>6:return None
    va,vb=np.array(past['velocity_W']),np.array(future['velocity_W']);dv=float(np.linalg.norm(va-vb))
    if dv>max(3.,.25*max(np.linalg.norm(va),np.linalg.norm(vb))):return None
    distances=[]
    for t in [first['end_s'],(first['end_s']+second['start_s'])/2,second['start_s']]:
        a=np.array(past['position_W'])+va*(t-past['time_s']);b=np.array(future['position_W'])+vb*(t-future['time_s']);distances.append(float(np.linalg.norm(a-b)))
    distance=max(distances)
    return dict(kind='bidirectional_clear_windows',past=past,future=future,distance_m=distance,velocity_disagreement_mps=dv,
                stationary=bool(np.linalg.norm(va[:2])<.7 and np.linalg.norm(vb[:2])<.7 and distance<1.5),camera_offset_fit=False)


def repair(first,second,evidence):
    """Preserve every clear sample and frozen anchor; repair only clipped ends."""
    for actor,fit,past in [(first,evidence['past'],True),(second,evidence['future'],False)]:
        if actor.get('frozen_lead'):continue
        boundary=fit['time_s'];anchor=next(r for r in actor['samples'] if abs(r['time_s']-boundary)<1e-8);origin=np.array(anchor['position_W']);velocity=np.array(fit['velocity_W'])
        for row in actor['samples']:
            if not (row['time_s']>boundary if past else row['time_s']<boundary):continue
            target=origin+velocity*(row['time_s']-boundary);old=np.array(row['position_W'])
            if actor.get('lane_index')!=-2:
                tangent=np.array([np.cos(row['yaw_rad']),np.sin(row['yaw_rad']),0.]);target=old+tangent*np.dot(target-old,tangent)
            row.update(position_W=target.tolist(),support='offline_bidirectional_clear_geometry',uses_future_observations=True,
                       geometry_reconstruction='past_clear_suffix' if past else 'future_clear_prefix',clear_handoff_anchor_frame_id=anchor['frame_id'],stationary_handoff_support=bool(evidence['stationary']))
