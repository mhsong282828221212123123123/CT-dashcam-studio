"""Lane regularization cannot establish road membership for an observation."""
import numpy as np
from .offline_paths import smooth_positions
from .road_frame import coordinates,world
from .road_geometry import lane_at,lane_bounds,boundaries_at,pocket_width,supported
from .actor_lane_motion import observation_usable


def confirmed_handoff_targets(rows,road,lane):
    local=coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in rows])
    proposed=[];targets=[]
    for s,d,_ in local:
        valid=0<=lane<len(boundaries_at(road,float(s)))-1
        proposed.append(lane if valid else -2)
        targets.append(sum(lane_bounds(road,float(s),lane))/2 if valid else float(d))
    return proposed,targets


def bounded_smooth(times,values,weights,lower,upper):
    """Acceleration fit with observation-side bounds, never a lane fallback."""
    t=np.asarray(times);w=np.asarray(weights);n=len(t)
    if n<3:return np.clip(values,lower,upper)
    D=np.zeros((n-2,n))
    for i in range(1,n-1):
        a,b=t[i]-t[i-1],t[i+1]-t[i]
        D[i-1,i-1:i+2]=[2/(a*(a+b)),-2/(a*b),2/(b*(a+b))]
    H=np.diag(w)+.5*float(np.median(np.diff(t)))*(D.T@D);rhs=w*np.asarray(values)
    free_solution=np.linalg.solve(H,rhs);x=np.clip(free_solution,lower,upper)
    active=np.where(free_solution<lower,-1,np.where(free_solution>upper,1,0))
    for _ in range(4*n+20):
        free=np.flatnonzero(active==0);fixed=np.flatnonzero(active!=0);candidate=x.copy()
        if len(free):candidate[free]=np.linalg.solve(H[np.ix_(free,free)],rhs[free]-H[np.ix_(free,fixed)]@x[fixed])
        direction=candidate-x;alpha=1.;block=None;side=0
        for i in free:
            if candidate[i]<lower[i]-1e-9 and direction[i]<0:
                step=(lower[i]-x[i])/direction[i]
                if step<alpha:alpha=step;block=i;side=-1
            elif candidate[i]>upper[i]+1e-9 and direction[i]>0:
                step=(upper[i]-x[i])/direction[i]
                if step<alpha:alpha=step;block=i;side=1
        x+=max(0.,alpha)*direction
        if block is not None:
            active[block]=side;x[block]=lower[block] if side==-1 else upper[block];continue
        gradient=H@x-rhs
        bad=np.where(active==-1,-gradient,np.where(active==1,gradient,0.))
        if np.max(bad)<=1e-7:return np.clip(x,lower,upper)
        active[int(np.argmax(bad))]=0
    raise ValueError('Bounded temporal position fit did not converge')


def refine(rows,road,proposed_lanes,lateral_targets,weights,confirmed_front_handoff=False):
    positions=np.asarray([r['position_W'] for r in rows],float)
    local=coordinates(road,positions)
    measured=[lane_at(road,float(s),float(d)) for s,d,_ in local]
    for i,(row,(s,d,_)) in enumerate(zip(rows,local)):
        lane=row.get('paint_lane_index')
        if not row.get('paint_lane_supported') or lane is None or not supported(road,float(s)) or not 0<=lane<len(boundaries_at(road,float(s)))-1:continue
        upper,lower=lane_bounds(road,float(s),lane)
        allowance=.75 if row.get('paint_contact_censored') else .5
        if lower-allowance<=d<=upper+allowance:measured[i]=lane
    if confirmed_front_handoff:
        for i,(row,(s,d,_),proposed) in enumerate(zip(rows,local,proposed_lanes)):
            if measured[i] is not None or row.get('camera_id') not in ('left_repeater','right_repeater') or not supported(road,float(s)):continue
            if not 0<=proposed<len(boundaries_at(road,float(s)))-1:continue
            upper,lower=lane_bounds(road,float(s),int(proposed))
            # An exclusive identity link to the front anchor supports a
            # modest side-rig error. It cannot recover a distant outside car.
            if lower-1.<=d<=upper+1.:measured[i]=int(proposed)
    eligible=np.asarray([lane is not None and lane==int(proposed)
                         for lane,proposed in zip(measured,proposed_lanes)],bool)
    w=np.asarray(weights,float)
    # Small/unclear image boxes cannot establish a lane-centre target. Their
    # weak projection weight is not a reason to strengthen a fallback lane.
    usable=[bool(r.get('paint_lane_supported')) or (observation_usable(r) if r.get('camera_id') is None or {'support','bbox_xyxy','image_width'}<=r.keys() else False) for r in rows]
    prior=np.where(usable,np.where(w<.5,3.,.3),0.)
    target=(w*local[:,1]+prior*np.asarray(lateral_targets))/(w+prior)
    result=positions.copy()
    if np.any(eligible):
        corrected=world(road,local[eligible,0],target[eligible])
        result[eligible,:2]=corrected[:,:2]
    # Temporal denoising remains necessary without a lane prior. It must not
    # turn a known outside contact into an inside contact at a track boundary.
    quality=np.maximum([r['quality'] for r in rows],.1);times=[r['time_s'] for r in rows]
    if len(rows)>=3:result=smooth_positions(times,result,quality,strength=.5)
    fitted=coordinates(road,result)
    lo=np.full(len(rows),-np.inf);hi=np.full(len(rows),np.inf)
    for i,(s,d,_) in enumerate(local):
        fs=float(fitted[i,0])
        if not supported(road,float(s)) or not supported(road,fs):continue
        if measured[i] is not None:
            if measured[i]>=len(boundaries_at(road,fs))-1:continue
            upper,lower=lane_bounds(road,fs,measured[i]);lo[i]=lower;hi[i]=upper;continue
        b=boundaries_at(road,float(s));upper=b[0]+pocket_width(road.get('left_pockets',[]),float(s))
        fb=boundaries_at(road,fs)
        if d<b[-1]:hi[i]=fb[-1]-.05
        elif d>upper:lo[i]=fb[0]+pocket_width(road.get('left_pockets',[]),fs)+.05
    desired=np.where(eligible,target,local[:,1])
    lateral=bounded_smooth(times,desired,quality,lo,hi)
    result[:,:2]=world(road,fitted[:,0],lateral)
    return result,eligible,measured
