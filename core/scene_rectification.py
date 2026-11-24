"""Calibrated lens rectification before inference; never invent lens constants."""
import json
from pathlib import Path
import cv2
import numpy as np


class CameraRectifier:
    def __init__(self,profile,width,height):
        self.profile=dict(profile);self.width=width;self.height=height
        model=profile.get('model')
        if model not in ('opencv_pinhole','opencv_fisheye','division1'):raise ValueError('Unknown lens model')
        source=np.asarray(profile['image_size'],float)
        if source.shape!=(2,) or np.any(source<=0):raise ValueError('Invalid calibration image size')
        if abs(width/height-source[0]/source[1])>1e-6:raise ValueError('Calibration aspect/crop mismatch')
        scale=width/source[0]
        K=np.asarray(profile['camera_matrix'],float);D=np.asarray(profile['distortion_coefficients'],float)
        newK=np.asarray(profile['rectified_camera_matrix'],float)
        if K.shape!=(3,3) or newK.shape!=(3,3) or not np.isfinite(np.r_[K.ravel(),D.ravel(),newK.ravel()]).all():raise ValueError('Invalid lens calibration')
        expected=(1,) if model=='division1' else (4,) if model=='opencv_fisheye' else (4,5,8)
        if D.ndim!=1 or len(D) not in expected:raise ValueError('Invalid distortion coefficient count')
        for matrix in (K,newK):
            if matrix[0,0]<=0 or matrix[1,1]<=0 or not np.allclose(matrix[2],[0,0,1]) or abs(matrix[0,1])>1e-8:raise ValueError('Invalid camera matrix')
        K=K.copy();newK=newK.copy();K[:2]*=scale;newK[:2]*=scale
        if abs(newK[0,0]-newK[1,1])>1e-6 or abs(newK[1,2]-height/2)>1e-6:raise ValueError('Rectified geometry requires square pixels and centred vertical principal point')
        self.K=K;self.newK=newK
        if model=='division1':
            yy,xx=np.indices((height,width),dtype=float)
            ux=(xx-newK[0,2])/newK[0,0];uy=(yy-newK[1,2])/newK[1,1]
            discriminant=1-4*D[0]*(ux*ux+uy*uy)
            factor=2/(1+np.sqrt(np.maximum(discriminant,0)))
            self.mapx=(K[0,2]+K[0,0]*ux*factor).astype(np.float32)
            self.mapy=(K[1,2]+K[1,1]*uy*factor).astype(np.float32)
            self.mapx[discriminant<0]=-1;self.mapy[discriminant<0]=-1
        else:
            fn=cv2.fisheye.initUndistortRectifyMap if model=='opencv_fisheye' else cv2.initUndistortRectifyMap
            self.mapx,self.mapy=fn(K,D,np.eye(3),newK,(width,height),cv2.CV_32FC1)
        self.valid=cv2.remap(np.full((height,width),255,np.uint8),self.mapx,self.mapy,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)>0

    def apply(self,frame):
        if frame.shape[:2]!=(self.height,self.width):raise ValueError('Frame/calibration size mismatch')
        return cv2.remap(frame,self.mapx,self.mapy,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)

    def metadata(self):
        return dict(status='applied',coordinate_space='rectified_pixels',profile=self.profile,
                    camera_matrix=self.newK.tolist(),source_camera_matrix=self.K.tolist(),
                    valid_fraction=float(self.valid.mean()))


def load_rectification(path,width,height):
    return CameraRectifier(json.loads(Path(path).read_text(encoding='utf-8-sig')),width,height)
