"""Piecewise road coordinates. Camera road geometry never modifies ego poses.

Polyline stations are metric arc length, lateral positive to the left. Explicit
branch selection is required at junctions; nearest-road projection cannot assign
a lane from a GPS position. Legacy straight corridors retain their old units.
"""
import numpy as np


class RoadFrame:
    def __init__(self, points, station_origin=0.):
        self.points=np.asarray(points,float)
        if self.points.ndim!=2 or self.points.shape[1]!=2 or len(self.points)<2 or not np.isfinite(self.points).all():
            raise ValueError('Finite XY polyline with at least two vertices required')
        delta=np.diff(self.points,axis=0);self.lengths=np.linalg.norm(delta,axis=1)
        if np.any(self.lengths<1e-5):raise ValueError('Repeated road vertices')
        self.tangents=delta/self.lengths[:,None]
        self.normals=np.c_[-self.tangents[:,1],self.tangents[:,0]]
        self.stations=np.r_[0,np.cumsum(self.lengths)]+station_origin

    def project(self, points):
        p=np.asarray(points,float);single=p.ndim==1;p=np.atleast_2d(p)[:,:2]
        if not np.isfinite(p).all():raise ValueError('Finite positions required')
        delta=p[:,None,:]-self.points[None,:-1,:]
        along=np.einsum('nki,ki->nk',delta,self.tangents)
        bounded=np.clip(along,0,self.lengths)
        # Only the two end segments may extrapolate. Interior corners must
        # not acquire infinite extensions that steal points from other roads.
        if len(self.lengths)==1:bounded=along.copy()
        else:
            bounded[:,0]=np.minimum(along[:,0],self.lengths[0])
            bounded[:,-1]=np.maximum(along[:,-1],0)
        residual=delta-bounded[:,:,None]*self.tangents
        idx=np.argmin(np.sum(residual**2,axis=2),axis=1);rows=np.arange(len(p))
        s=self.stations[idx]+bounded[rows,idx]
        d=np.einsum('ni,ni->n',delta[rows,idx],self.normals[idx])
        heading=np.arctan2(self.tangents[idx,1],self.tangents[idx,0])
        result=np.c_[s,d,heading]
        return result[0] if single else result

    def world(self, station, lateral=0.):
        s,d=np.broadcast_arrays(np.asarray(station,float),np.asarray(lateral,float))
        idx=np.clip(np.searchsorted(self.stations,s,side='right')-1,0,len(self.lengths)-1)
        return self.points[idx]+(s-self.stations[idx])[...,None]*self.tangents[idx]+d[...,None]*self.normals[idx]


def frame_for(road):
    data=road.get('reference_line')
    return RoadFrame(data['points_W_xy'],data.get('station_origin_m',0.)) if data else None


def coordinates(road,positions):
    frame=frame_for(road)
    if frame is not None:return frame.project(positions)
    p=np.asarray(positions,float);slope=road.get('heading_evidence',{}).get('slope',0.)
    return np.stack([p[...,0],p[...,1]-slope*p[...,0],np.full(p.shape[:-1],np.arctan(slope))],axis=-1)


def world(road,station,lateral):
    frame=frame_for(road)
    if frame is not None:return frame.world(station,lateral)
    s,d=np.broadcast_arrays(np.asarray(station,float),np.asarray(lateral,float))
    return np.stack([s,d+road.get('heading_evidence',{}).get('slope',0.)*s],axis=-1)
