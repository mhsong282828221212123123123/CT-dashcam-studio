"""Explicit pixel transforms. Synthetic lens math is not a vehicle calibration."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class CropResize:
    x: float
    y: float
    width: float
    height: float
    output_width: float
    output_height: float
    pad_x: float = 0
    pad_y: float = 0

    def __post_init__(self):
        values = np.asarray(list(vars(self).values()),float)
        if not np.isfinite(values).all() or min(self.width,self.height,self.output_width,self.output_height)<=0:
            raise ValueError('Invalid pixel transform')

    def forward(self, pixels):
        return (np.asarray(pixels,float)-[self.x,self.y])*[self.output_width/self.width,self.output_height/self.height]+[self.pad_x,self.pad_y]

    def backward(self, pixels):
        return (np.asarray(pixels,float)-[self.pad_x,self.pad_y])*[self.width/self.output_width,self.height/self.output_height]+[self.x,self.y]


@dataclass(frozen=True)
class RadialTangentialLens:
    fx: float
    fy: float
    cx: float
    cy: float
    coefficients: tuple = (0,0,0,0,0)  # k1,k2,p1,p2,k3

    def __post_init__(self):
        if len(self.coefficients)!=5 or not np.isfinite([self.fx,self.fy,self.cx,self.cy,*self.coefficients]).all() or min(self.fx,self.fy)<=0:
            raise ValueError('Invalid pinhole lens')

    def distort_normalized(self, xy):
        xy=np.asarray(xy,float)
        x,y=xy[...,0],xy[...,1]
        k1,k2,p1,p2,k3=self.coefficients
        r2=x*x+y*y
        radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        dx=2*p1*x*y+p2*(r2+2*x*x)
        dy=p1*(r2+2*y*y)+2*p2*x*y
        return np.stack([x*radial+dx,y*radial+dy],axis=-1)

    def raw_pixels(self, xy):
        return self.distort_normalized(xy)*[self.fx,self.fy]+[self.cx,self.cy]

    def rays(self, raw_pixels, tolerance_px=.01):
        target=(np.asarray(raw_pixels,float)-[self.cx,self.cy])/[self.fx,self.fy]
        xy=target.copy()
        # Newton iterations with numerical Jacobian; failures remain invalid rays.
        for _ in range(20):
            f=self.distort_normalized(xy)-target
            eps=1e-6
            jx=(self.distort_normalized(xy+[eps,0])-self.distort_normalized(xy-[eps,0]))/(2*eps)
            jy=(self.distort_normalized(xy+[0,eps])-self.distort_normalized(xy-[0,eps]))/(2*eps)
            det=jx[...,0]*jy[...,1]-jy[...,0]*jx[...,1]
            safe=np.where(np.abs(det)>1e-10,det,np.nan)
            delta=np.stack([(jy[...,1]*f[...,0]-jy[...,0]*f[...,1])/safe,
                            (-jx[...,1]*f[...,0]+jx[...,0]*f[...,1])/safe],axis=-1)
            xy-=delta
        error=np.linalg.norm(self.raw_pixels(xy)-raw_pixels,axis=-1)
        valid=np.isfinite(xy).all(axis=-1)&(error<=tolerance_px)
        return xy,valid
