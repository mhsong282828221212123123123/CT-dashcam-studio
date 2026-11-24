"""Robust camera-road curves in a shared ego-compensated coordinate frame."""
import numpy as np


def retain_unmatched_observations(observed, fitted, tolerance=.6):
    """A stable path must not erase a newly visible branch or adjacent boundary."""
    retained=[]
    for lane in observed:
        points=np.asarray(lane['points'],float)
        if len(points)<2:continue
        explained=np.zeros(len(points),bool)
        for path in fitted:
            curve=np.asarray(path['points'],float)
            if len(curve)<2:continue
            order=np.argsort(curve[:,1]);curve=curve[order]
            inside=(points[:,1]>=curve[0,1])&(points[:,1]<=curve[-1,1])
            expected=np.interp(points[:,1],curve[:,1],curve[:,0])
            explained|=inside&(abs(points[:,0]-expected)<=tolerance)
        # Split the unsupported portion. Never redraw a supported section or
        # join across a fitted section to create a false intersection line.
        indices=np.flatnonzero(~explained)
        for run in np.split(indices,np.flatnonzero(np.diff(indices)>1)+1):
            if len(run)<2:continue
            retained.append(dict(lane,points=points[run].tolist(),
                source='current_camera_unmatched_temporal_fit',temporally_confirmed=False))
    return retained


def robust_curve(points):
    points=np.asarray(points,float)
    if len(points)<6 or not np.isfinite(points).all() or np.ptp(points[:,1])<3:return None
    z=points[:,1]/20.;x=points[:,0]
    A=np.column_stack([np.ones(len(z)),z,z*z]);weights=np.ones(len(z))
    for _ in range(8):
        coefficient=np.linalg.lstsq(A*weights[:,None]**.5,x*weights**.5,rcond=None)[0]
        errors=abs(A@coefficient-x)
        weights=np.minimum(1.,.20/np.maximum(errors,1e-8))
    valid=errors<.5
    if valid.mean()<.75 or np.ptp(points[valid,1])<3:return None
    return coefficient,float(points[valid,1].min()),float(points[valid,1].max())


def coherent_paths(segments):
    candidates=[]
    for segment in segments:
        points=np.asarray(segment['points'],float)
        fit=robust_curve(points)
        if fit is None:continue
        coefficients,near,far=fit
        if near>18 or far<0:continue
        at8=float(np.array([1,.4,.16])@coefficients)
        if abs(at8)>25:continue
        candidates.append((at8,segment))
    clusters=[]
    for at8,segment in sorted(candidates,key=lambda item:item[0]):
        if not clusters or at8-np.median([p[0] for p in clusters[-1]])>.7:
            clusters.append([])
        clusters[-1].append((at8,segment))
    output=[]
    for cluster in clusters:
        points=np.concatenate([np.asarray(item[1]['points']) for item in cluster])
        fit=robust_curve(points)
        if fit is None:continue
        coefficient,near,far=fit
        near=max(-15.,near);far=min(65.,far)
        if far-near<3:continue
        zs=np.linspace(near,far,max(10,int((far-near)*4)))
        xs=np.column_stack([np.ones(len(zs)),zs/20,(zs/20)**2])@coefficient
        if np.max(abs(np.diff(xs)/np.diff(zs)))>1.:continue
        edge_votes=sum(item[1].get('edge_role')=='estimated_road_edge' for item in cluster)
        edge=edge_votes>len(cluster)/2
        observed_z=np.sort(points[:,1]);indices=np.searchsorted(observed_z,zs)
        lower=observed_z[np.clip(indices-1,0,len(observed_z)-1)]
        upper=observed_z[np.clip(indices,0,len(observed_z)-1)]
        supported=np.minimum(abs(zs-lower),abs(zs-upper))<=1.5
        # A polynomial must not paint across a large unsupported intersection.
        runs=np.split(np.flatnonzero(supported),np.flatnonzero(np.diff(np.flatnonzero(supported))>1)+1)
        for run in runs:
            if len(run)<4:continue
            output.append(dict(points=np.column_stack([xs[run],zs[run]]).tolist(),dashed=not edge,
                               edge_role='estimated_road_edge' if edge else 'camera_paint',
                               observed=False,inferred=True,source='robust_temporal_camera_curve',
                               extrapolated=False,observation_segments=len(cluster)))
    return output
