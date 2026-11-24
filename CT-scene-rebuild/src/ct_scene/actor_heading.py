"""Two-sided motion heading. A stop does not reset a car to the observer yaw."""
import numpy as np


def resolve_axis_flips(times,headings,max_bridge_s=1.,visible_times=None,max_visible_bridge_s=8.):
    """Resolve instantaneous velocity-sign ambiguity without changing positions.

    A brief backward contact fit at a stop does not rotate the physical body
    180 degrees. Use the nearest continuous orientation axis only for adjacent
    observations; a long unknown interval cannot establish front/back identity.
    Gradual turns, including a U-turn, retain their cumulative rotation.
    """
    t=np.asarray(times,float);raw=np.unwrap(np.asarray(headings,float));result=raw.copy()
    if len(t)!=len(raw) or not np.isfinite(raw).all() or np.any(np.diff(t)<=0):raise ValueError('Ordered finite headings required')
    visible=np.asarray(visible_times,float) if visible_times is not None else np.array([])
    if not np.isfinite(visible).all() or np.any(np.diff(visible)<=0):raise ValueError('Ordered visible times required')
    for i in range(1,len(t)):
        gap=t[i]-t[i-1]
        if gap>max_bridge_s:
            seen=visible[(visible>=t[i-1])&(visible<=t[i])]
            # A failed ground projection is not a disappearance: the same local
            # identity can remain continuously visible between metric anchors.
            if gap>max_visible_bridge_s or len(seen)<3 or seen[0]>t[i-1]+1e-6 or seen[-1]<t[i]-1e-6 or np.max(np.diff(seen))>max_bridge_s:continue
        delta=(raw[i]-result[i-1]+np.pi)%(2*np.pi)-np.pi
        if abs(delta)>np.pi/2:delta-=np.sign(delta)*np.pi
        result[i]=result[i-1]+delta
    return result


def estimate(times,positions,fallback=0.,window_s=1.,minimum_speed=1.5):
    t=np.asarray(times,float);p=np.asarray(positions,float)
    if len(t)<2 or np.any(np.diff(t)<=0) or not np.isfinite(p).all():raise ValueError('Ordered finite actor positions required')
    headings=np.full(len(t),np.nan)
    for i,now in enumerate(t):
        use=abs(t-now)<=window_s
        if sum(use)<3 or np.ptp(t[use])<.4:continue
        X=np.c_[t[use]-now,np.ones(sum(use))];fit=np.linalg.lstsq(X,p[use,:2],rcond=None)[0]
        residual=np.quantile(np.linalg.norm(X@fit-p[use,:2],axis=1),.8)
        if np.linalg.norm(fit[0])<minimum_speed or residual>.8:continue
        headings[i]=np.arctan2(fit[0,1],fit[0,0])
    valid=np.flatnonzero(np.isfinite(headings))
    if not len(valid):return np.full(len(t),fallback),np.zeros(len(t),bool)
    measured=np.unwrap(headings[valid]);result=np.interp(t,t[valid],measured)
    return result,np.isfinite(headings)
