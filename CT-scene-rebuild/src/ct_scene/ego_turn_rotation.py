"""Bounded own-turn timing from independent static-background camera rotations.

No actor pose, bounding-box centre, GPS position, map or fitted camera offset
enters the correction. Own SEI sets both endpoint headings and wheel distance.
"""
import numpy as np


def corrected_yaw(times,yaws,camera_pairs):
    t=np.asarray(times,float);yaw=np.unwrap(np.asarray(yaws,float));result=yaw.copy();reviews=[]
    if len(t)<3 or len(t)!=len(yaw) or np.any(np.diff(t)<=0):raise ValueError('Ordered own motion required')
    active=np.flatnonzero(abs(np.gradient(yaw,t))>np.radians(4.))
    groups=[]
    for i in active:
        if not groups or t[i]-t[groups[-1][-1]]>1.5:groups.append([])
        groups[-1].append(i)
    windows=[]
    for g in groups:
        start=max(float(t[0]),float(t[g[0]]-2.5));end=min(float(t[-1]),float(t[g[-1]]+2.5))
        if windows and start<=windows[-1][1]:windows[-1][1]=end
        else:windows.append([start,end])
    for start,end in windows:
        ids=np.flatnonzero((t>=start)&(t<=end));review=dict(start_s=start,end_s=end,accepted=False)
        if len(ids)<10 or abs(yaw[ids[-1]]-yaw[ids[0]])<np.radians(20):
            review['reason']='not_a_large_own_turn';reviews.append(review);continue
        increments=[];supported=0.;duration=t[ids[-1]]-t[ids[0]]
        for i,j in zip(ids[:-1],ids[1:]):
            lo,hi=t[i],t[j];rates=[]
            for camera,rows in camera_pairs.items():
                if camera not in ('front','left_repeater','right_repeater'):continue
                overlap=0.;angle=0.
                for r in rows:
                    if not r.get('accepted'):continue
                    dt=r['end_s']-r['start_s'];weight=max(0.,min(hi,r['end_s'])-max(lo,r['start_s']))
                    if dt<=0 or dt>.35:continue
                    overlap+=weight;angle+=np.radians(r['visual_delta_deg'])*weight/dt
                if overlap>=(hi-lo)*.8:rates.append(angle/overlap)
            selected=[]
            if len(rates)>=2:
                pair=min(((a,b) for k,a in enumerate(rates) for b in rates[k+1:]),key=lambda pair:abs(pair[0]-pair[1]))
                if abs(pair[0]-pair[1])<=np.radians(4):selected=[r for r in rates if abs(r-np.mean(pair))<=np.radians(4)]
            if len(selected)>=2:
                increments.append(float(np.median(selected))*(hi-lo));supported+=hi-lo
            else:increments.append(yaw[j]-yaw[i])
        visual=np.r_[0,np.cumsum(increments)];drift=float(visual[-1]-(yaw[ids[-1]]-yaw[ids[0]]));review.update(consensus_fraction=supported/duration,endpoint_drift_deg=float(np.degrees(drift)))
        if supported/duration<.85 or abs(drift)>np.radians(8):
            review['reason']='insufficient_independent_rotation_consensus';reviews.append(review);continue
        local=yaw[ids[0]]+visual-drift*(t[ids]-t[ids[0]])/duration;correction=local-yaw[ids];review['max_correction_deg']=float(np.degrees(np.max(abs(correction))))
        if np.max(abs(correction))>np.radians(25):
            review['reason']='correction_outside_bounded_turn_timing';reviews.append(review);continue
        result[ids]=local;review.update(accepted=True,reason='two_camera_background_rotation; own_SEI_endpoint_headings_preserved');reviews.append(review)
    return result,reviews
