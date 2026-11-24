"""Deterministic, orthographic GLB ego preview. No inferred road geometry."""
import json
import struct
from functools import lru_cache
from pathlib import Path
import cv2
import numpy as np
from core.scene_telemetry import odometry_speed_kmh
try:
    from numba import njit
except ImportError:
    # Keep the rest of the app importable without optional preview dependencies.
    def njit(**kwargs):
        def decorate(fn):
            def unavailable(*args, **kwargs):
                raise RuntimeError('Scene preview requires: pip install -r requirements-scene-preview.txt')
            return unavailable
        return decorate

ASSET = Path(__file__).resolve().parents[1] / 'assets/scene/tesla_ego_model3.glb'


class EgoTimeline:
    def __init__(self, rows, max_gap=.5):
        self.states = []
        distance = 0.
        previous = None
        for row in rows:
            t = float(row['time_s'])
            data = (row.get('ego') or {}).get('data')
            speed = odometry_speed_kmh(data)
            if previous is not None:
                pt, ps = previous
                if speed is not None and ps is not None and 0 < t-pt <= max_gap:
                    distance += (float(speed)+float(ps)) / 7.2 * (t-pt)
            self.states.append((data, t, distance/.343))
            previous = t, speed


@njit(cache=True)
def raster(canvas, depth, points, triangles, colors):
    h, w = depth.shape
    for tri in triangles:
        a,b,c = tri
        x0,y0,z0=points[a]; x1,y1,z1=points[b]; x2,y2,z2=points[c]
        den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
        if abs(den)<1e-8: continue
        for y in range(max(0,int(np.floor(min(y0,y1,y2)))),min(h,int(np.ceil(max(y0,y1,y2)))+1)):
            for x in range(max(0,int(np.floor(min(x0,x1,x2)))),min(w,int(np.ceil(max(x0,x1,x2)))+1)):
                u=((y1-y2)*(x+.5-x2)+(x2-x1)*(y+.5-y2))/den
                v=((y2-y0)*(x+.5-x2)+(x0-x2)*(y+.5-y2))/den
                q=1-u-v
                if u<0 or v<0 or q<0: continue
                z=u*z0+v*z1+q*z2
                if z<=depth[y,x]: continue
                depth[y,x]=z
                for k in range(3): canvas[y,x,k]=min(255,max(0,u*colors[a,k]+v*colors[b,k]+q*colors[c,k]))


class EgoModel:
    """Loader for the bundled opaque, triangle-only, untextured GLB asset."""
    def __init__(self, path=ASSET):
        raw=Path(path).read_bytes()
        magic,version,length=struct.unpack_from('<4sII',raw)
        if magic!=b'glTF' or version!=2 or length!=len(raw): raise ValueError('Invalid GLB')
        size,kind=struct.unpack_from('<II',raw,12)
        if kind!=0x4e4f534a: raise ValueError('Missing GLB JSON')
        self.g=json.loads(raw[20:20+size]); offset=20+size
        bs,bk=struct.unpack_from('<II',raw,offset)
        if bk!=0x004e4942: raise ValueError('Missing GLB buffer')
        self.binary=raw[offset+8:offset+8+bs]
        self.parts={}
        for i,node in enumerate(self.g['nodes']):
            if any(k in node for k in ('rotation','scale','matrix')): raise ValueError('Unsupported asset transform')
            parts=[]
            if 'mesh' in node:
                for p in self.g['meshes'][node['mesh']]['primitives']:
                    if p.get('mode',4)!=4: raise ValueError('Triangle mesh required')
                    pos=self.accessor(p['attributes']['POSITION'])
                    normals=self.accessor(p['attributes']['NORMAL'])
                    ids=self.accessor(p['indices']).reshape(-1,3).astype(np.int64)
                    if ids.min()<0 or ids.max()>=len(pos): raise ValueError('Invalid mesh indices')
                    parts.append((pos,normals,ids,self.g['materials'][p['material']]))
            self.parts[i]=parts

    def accessor(self, index):
        a=self.g['accessors'][index]; v=self.g['bufferViews'][a['bufferView']]
        if 'sparse' in a: raise ValueError('Sparse accessor unsupported')
        dtype=np.dtype({5126:'<f4',5125:'<u4',5123:'<u2'}[a['componentType']])
        count={'SCALAR':1,'VEC3':3}[a['type']]
        result=np.ndarray((a['count'],count),dtype=dtype,buffer=self.binary,
            offset=v.get('byteOffset',0)+a.get('byteOffset',0),
            strides=(v.get('byteStride',dtype.itemsize*count),dtype.itemsize)).copy()
        if not np.isfinite(result).all(): raise ValueError('Non-finite model geometry')
        return result

    def transforms(self, spin):
        out={}
        def walk(i,parent):
            node=self.g['nodes'][i]; local=np.eye(4); local[:3,3]=node.get('translation',[0,0,0])
            if node.get('name','').startswith('Wheel_'):
                c,s=np.cos(spin),np.sin(spin)
                local[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]]
            out[i]=parent@local
            for child in node.get('children',[]): walk(child,out[i])
        for root in self.g['scenes'][self.g.get('scene',0)]['nodes']: walk(root,np.eye(4))
        return out


@lru_cache(maxsize=1)
def model(): return EgoModel()


def render_ego(telemetry, width=320, height=240, *, time_s=0., spin=0.):
    asset=model()
    canvas=np.full((height,width,3),(30,28,25),np.uint8)
    depth=np.full((height,width),-np.inf)
    toward=np.array([.408248,.707107,-.577350])
    right=np.cross(np.array([0.,1.,0.]),toward); right/=np.linalg.norm(right)
    up=np.cross(toward,right)
    basis=np.column_stack((right,-up,toward))
    scale=min(width/6.6,height/5.6)
    origin=np.array([width*.5,height*.59,0.])
    light=np.array([-.35,.85,-.4]); light/=np.linalg.norm(light)
    blink=(time_s % .66)<.33
    transforms=asset.transforms(spin)
    for i,parts in asset.parts.items():
        name=asset.g['nodes'][i].get('name','')
        active=False
        if name.startswith('BrakeLight'): active=bool(telemetry and telemetry.get('brake_applied'))
        elif name.startswith('TurnSignal'):
            left=name.endswith(('FL','RL','SideL'))
            active=bool(telemetry and telemetry.get('left_blinker' if left else 'right_blinker')) and blink
        transform=transforms[i]
        for pos,normals,tri,mat in parts:
            world=pos@transform[:3,:3].T+transform[:3,3]
            projected=world@basis
            projected[:,:2]*=scale; projected+=origin
            shade=.4+.6*np.maximum(0,(normals@transform[:3,:3].T)@light)
            base=np.array(mat['pbrMetallicRoughness']['baseColorFactor'][:3])
            if 'emissiveFactor' in mat:
                base=base if active else base*.18
                if active: shade=np.ones_like(shade)
            colors=np.ascontiguousarray(shade[:,None]*base[::-1]*255)
            raster(canvas,depth,projected,tri,colors)
    cv2.putText(canvas,'MODEL 3 / SEI',(10,20),0,.45,(225,225,225),1,cv2.LINE_AA)
    label='Telemetry unavailable' if telemetry is None else f"{telemetry.get('speed_kmh','?')} km/h | Steer {telemetry.get('steering_deg','?')} deg"
    cv2.putText(canvas,label,(10,height-25),0,.37,(230,230,230),1,cv2.LINE_AA)
    cv2.putText(canvas,'Steering ratio / surroundings: uncalibrated',(10,height-9),0,.30,(120,170,220),1,cv2.LINE_AA)
    return canvas


def overlay_ego(frame, ego):
    result=frame.copy()
    w=min(240,result.shape[1]//3); h=round(w*ego.shape[0]/ego.shape[1])
    tile=cv2.resize(ego,(w,h),interpolation=cv2.INTER_AREA)
    result[-h:,-w:]=tile
    return result
