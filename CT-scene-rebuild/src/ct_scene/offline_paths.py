"""Two-ended world-path completion for already associated object identities.

Camera association and world observations are inputs, not invented here.
Estimated geometry is allowed; unverified exposure clocks are not silently
treated as a shared timeline. Rear-reference anchors may supply lane only.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class WorldAnchor:
    object_id: str
    time_s: float
    position_W: tuple | None
    velocity_W: tuple | None = None
    lane_key: tuple | None = None
    frame_ids: tuple = ()
    clock_status: str = 'estimated'

    def __post_init__(self):
        if not self.object_id or not math.isfinite(self.time_s):raise ValueError('Invalid world anchor')
        if self.clock_status not in ('estimated','verified'):raise ValueError('Common timeline must be explicitly estimated or verified')
        for value in (self.position_W,self.velocity_W):
            if value is not None and (len(value)!=3 or not np.isfinite(value).all()):raise ValueError('Expected finite world xyz')


def smooth_positions(times,positions,weights=None,strength=.2):
    """Whole-interval acceleration fit; later observations revise earlier states."""
    t=np.asarray(times,float);p=np.asarray(positions,float)
    if p.shape!=(len(t),3) or len(t)<3 or not np.isfinite(p).all() or not np.isfinite(t).all() or np.any(np.diff(t)<=0):
        raise ValueError('Need finite ordered 3D observations')
    w=np.ones(len(t)) if weights is None else np.asarray(weights,float)
    if w.shape!=t.shape or not np.isfinite(w).all() or np.any(w<=0) or not np.isfinite(strength) or strength<0:raise ValueError('Invalid smoother weights')
    D=np.zeros((len(t)-2,len(t)))
    for i in range(1,len(t)-1):
        a,b=t[i]-t[i-1],t[i+1]-t[i]
        D[i-1,i-1:i+2]=[2/(a*(a+b)),-2/(a*b),2/(b*(a+b))]
    H=np.diag(w)+strength*float(np.median(np.diff(t)))*(D.T@D)
    return np.linalg.solve(H,w[:,None]*p)


def augment_frozen_geometry(trusted,candidates):
    """Fill only missing observed geometry; trusted rows remain byte-equivalent."""
    known={r['frame_id'] for r in trusted};new=[r for r in candidates if r['frame_id'] not in known]
    if len(new)<3:return list(trusted)
    rows=sorted(list(trusted)+new,key=lambda r:r['time_s'])
    times=np.array([r['time_s'] for r in rows]);positions=np.array([r['position_W'] for r in rows])
    weights=[1e6 if r['frame_id'] in known else .2 for r in rows]
    fitted=smooth_positions(times,positions,weights,strength=.5)
    from .actor_heading import estimate
    angles,support=estimate(times,fitted,fallback=trusted[0]['yaw_rad'] if trusted else 0.)
    if trusted:
        # Walk through each missing interval from a frozen neighbour. Comparing
        # every row to one fixed heading would flip a gradual >90-degree turn.
        first=next(i for i,r in enumerate(rows) if r['frame_id'] in known)
        previous=rows[first]['yaw_rad']
        for i in range(first-1,-1,-1):
            delta=(angles[i]-previous+np.pi/2)%np.pi-np.pi/2
            angles[i]=previous+delta;previous=angles[i]
        previous=rows[first]['yaw_rad']
        for i in range(first+1,len(rows)):
            if rows[i]['frame_id'] in known:previous=rows[i]['yaw_rad'];continue
            delta=(angles[i]-previous+np.pi/2)%np.pi-np.pi/2
            angles[i]=previous+delta;previous=angles[i]
        anchors=sorted(trusted,key=lambda r:r['time_s'])
        interior=(times>=anchors[0]['time_s'])&(times<=anchors[-1]['time_s'])
        # The height-based depth prior can loop at a stop. Its velocity cannot
        # overrule two-sided body observations. Only missing rows use this fit.
        angles[interior]=np.interp(times[interior],[r['time_s'] for r in anchors],np.unwrap([r['yaw_rad'] for r in anchors]))
    return [r if r['frame_id'] in known else dict(r,position_W=fitted[i].tolist(),yaw_rad=float(angles[i]),
                    geometry_completion='observed_size_prior_with_frozen_neighbours',uses_future_observations=True) for i,r in enumerate(rows)]


def complete_gap(before,after,times,max_gap_s=8):
    if before.object_id!=after.object_id:raise ValueError('Different vehicle identities')
    dt=after.time_s-before.time_s
    if not 0<dt<=max_gap_s:raise ValueError('Gap outside reconstruction budget')
    times=np.asarray(times,float)
    if not np.isfinite(times).all() or np.any(times<=before.time_s) or np.any(times>=after.time_s):raise ValueError('Only interior times can be reconstructed')
    same_lane=before.lane_key is not None and before.lane_key==after.lane_key
    lane=before.lane_key if same_lane else None;rows=[]
    usable=before.position_W is not None and after.position_W is not None
    if usable:
        a,b=np.array(before.position_W,float),np.array(after.position_W,float);delta=b-a
        slopes=[]
        for v in (before.velocity_W,after.velocity_W):
            slope=delta.copy() if v is None else np.asarray(v,float)*dt
            for k,d in enumerate(delta):
                if abs(d)<1e-9:slope[k]=0
                elif slope[k]/d<0:slope[k]=0
            slopes.append(slope)
        # Limit Hermite tangents: a fast endpoint cannot launch a vehicle beyond
        # both observed positions during the blind interval.
        for k,d in enumerate(delta):
            if abs(d)<1e-9:continue
            alpha,beta=slopes[0][k]/d,slopes[1][k]/d;radius=math.hypot(alpha,beta)
            if radius>3:
                slopes[0][k]*=3/radius;slopes[1][k]*=3/radius
    for t in times:
        row={'object_id':before.object_id,'time_s':float(t),'lane_key':lane,
             'lane_support':'both_endpoint_observations' if same_lane else 'unresolved_or_lane_change',
             'uses_future_observation':True,'endpoint_frame_ids':list(before.frame_ids+after.frame_ids),
             'position_support':'temporally_reconstructed' if usable else 'unresolved','position_W':None,'yaw_rad':None,
             'render_state':'ghost' if usable else 'unlocalized'}
        if usable:
            u=(t-before.time_s)/dt
            p=(2*u**3-3*u**2+1)*a+(u**3-2*u**2+u)*slopes[0]+(-2*u**3+3*u**2)*b+(u**3-u**2)*slopes[1]
            derivative=(6*u*u-6*u)*a+(3*u*u-4*u+1)*slopes[0]+(-6*u*u+6*u)*b+(3*u*u-2*u)*slopes[1]
            row['position_W']=p.tolist()
            if np.linalg.norm(derivative[:2])>1e-6:row['yaw_rad']=math.atan2(derivative[1],derivative[0])
        rows.append(row)
    return rows
