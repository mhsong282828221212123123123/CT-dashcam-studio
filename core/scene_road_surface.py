"""Temporal road footprint consensus, not a lane-count or wall estimator."""
import numpy as np


def _intervals(polygons,z):
    intervals=[]
    for polygon in polygons:
        p=np.asarray(polygon,float)
        if p.shape!=(4,2) or not np.isfinite(p).all():continue
        xs=[]
        for a,b in zip(p,np.roll(p,-1,axis=0)):
            if abs(a[1]-b[1])<1e-9:
                if abs(z-a[1])<1e-8:xs.extend([a[0],b[0]])
            elif min(a[1],b[1])<=z<=max(a[1],b[1]):
                xs.append(a[0]+(z-a[1])*(b[0]-a[0])/(b[1]-a[1]))
        if len(xs)>=2 and max(xs)-min(xs)>.05:intervals.append((min(xs),max(xs)))
    merged=[]
    for left,right in sorted(intervals):
        if merged and left<=merged[-1][1]+1e-6:merged[-1]=(merged[-1][0],max(right,merged[-1][1]))
        else:merged.append((left,right))
    return merged


def consensus_surface(frames,step=.5):
    """Use one vote per camera time rather than the union of all noisy edges.

    Each frame contains ground-plane quads in the same coordinate system.
    Unsupported longitudinal bands stay empty; disconnected lateral components
    stay separate. This estimates visible road area and never creates walls.
    """
    if not frames:return []
    all_points=[np.asarray(p,float) for frame in frames for p in frame]
    if not all_points:return []
    points=np.concatenate(all_points)
    near=max(-15.,float(points[:,1].min()));far=min(65.,float(points[:,1].max()))
    output=[];previous=[];previous_z=None
    for z in np.arange(np.ceil(near/step)*step,far+1e-8,step):
        votes=[(left,right,i) for i,frame in enumerate(frames) for left,right in _intervals(frame,z)]
        groups=[]
        for vote in sorted(votes):
            # Overlap groups cannot bridge a lateral hole within one frame.
            compatible=[g for g in groups if vote[0]<np.median([v[1] for v in g])
                        and vote[1]>np.median([v[0] for v in g])
                        and vote[2] not in {v[2] for v in g}]
            if len(compatible)==1:compatible[0].append(vote)
            else:groups.append([vote])
        current=[]
        active=len({v[2] for v in votes})
        for group in groups:
            if len(group)<max(1,int(np.ceil(active/2))):continue
            current.append(tuple(np.median([(v[0],v[1]) for v in group],axis=0)))
        if previous_z is not None:
            for left,right in current:
                matches=[p for p in previous if min(right,p[1])>max(left,p[0])]
                if len(matches)!=1:continue
                a,b=matches[0]
                # At split/merge topology, don't span across an opening.
                if sum(min(r,b)>max(l,a) for l,r in current)!=1:continue
                output.append([[a,previous_z],[b,previous_z],[right,float(z)],[left,float(z)]])
        previous=current;previous_z=float(z)
    return output
