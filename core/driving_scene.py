"""Light driving scene: explicit synthetic demo and uncalibrated observations.

Coordinates: +X left, +Y up, +Z forward, metres in the renderer.
The default projection is an illustrative ground-plane assumption, NOT calibration.
"""
from dataclasses import dataclass, field, replace
from bisect import bisect_right
from functools import lru_cache
import math
import cv2
import numpy as np
import copy
from core.scene_preview import ASSET, EgoModel, EgoTimeline, model, raster


@dataclass
class Actor:
    key: str
    kind: str
    x: float
    z: float
    yaw: float = 0.
    confidence: float = 1.
    estimated: bool = False
    sources: tuple = ()
    width: float = 1.9
    length: float = 4.7
    tracking_state: str = 'observed'
    prediction_age: float = 0.
    position_uncertainty: float = 0.
    geometry_kind: str = ''
    prediction_horizon: float = 1.5
    height: float | None = None
    image_observed: bool = False
    position_hypothesis: bool = False


def actor_is_ghost(actor):
    """Transparency represents missing presence, not uncertain metric depth."""
    return actor.position_hypothesis or (actor.tracking_state!='observed' and not actor.image_observed)


@dataclass
class Scene:
    time: float
    telemetry: dict | None
    distance: float = 0.
    actors: list = field(default_factory=list)
    lanes: list = field(default_factory=list)
    path: list = field(default_factory=list)
    mode: str = 'observed'
    note: str = ''
    rejected: int = 0


def road_display_scene(scene):
    """Apply ONE rigid transform to road, host, objects and signals for display."""
    view=getattr(scene,'road_view',None)
    if not view:return scene
    result=copy.copy(scene);R=np.asarray(view['rotation']);offset=np.asarray(view['offset'])
    point=lambda p:(R@np.asarray(p)+offset).tolist()
    yaw=math.atan2(R[0,1],R[1,1])
    result.ego_display=(*offset,yaw)
    if hasattr(scene,'ego_display_override'):
        result.ego_display=scene.ego_display_override
    result.actors=[replace(a,x=point([a.x,a.z])[0],z=point([a.x,a.z])[1],yaw=a.yaw+yaw) for a in scene.actors]
    result.lanes=[dict(lane,points=[point(p) for p in lane['points']]) for lane in scene.lanes]
    result.path=[point(p) for p in scene.path]
    result.road_surface=[point(p) for p in getattr(scene,'road_surface',[])]
    result.road_surfaces=[[point(p) for p in surface] for surface in getattr(scene,'road_surfaces',[])]
    result.signals=[]
    for signal in getattr(scene,'signals',[]):
        item=dict(signal)
        if 'position_estimate' in item:
            x,y,z=item['position_estimate'];x,z=point([x,z]);item['position_estimate']=[x,y,z]
        result.signals.append(item)
    return result


@dataclass(frozen=True)
class GroundAssumption:
    """Editable preview assumptions; deliberately not named a camera calibration."""
    height: float = 1.2
    focal_normalized: float = .65
    horizon_normalized: float = .50
    yaw: float = 0.
    offset_x: float = 0.
    offset_z: float = 0.

    def project(self, u, v, width, height):
        if width <= 0 or height <= 0: return None
        dy = v / height - self.horizon_normalized
        if dy <= .025 or v >= height * .985 or not 0 <= u <= width:
            return None
        forward = self.height * self.focal_normalized / dy
        left = -(u / width - .5) * forward / self.focal_normalized
        x = left * math.cos(self.yaw) + forward * math.sin(self.yaw) + self.offset_x
        z = forward * math.cos(self.yaw) - left * math.sin(self.yaw) + self.offset_z
        if not np.isfinite([x, z]).all() or math.hypot(x, z) > 45:
            return None
        return float(x), float(z)


ASSUMPTIONS = {
    'front': GroundAssumption(offset_z=1.2),
    'back': GroundAssumption(yaw=math.pi, offset_z=-2.),
    'left_repeater': GroundAssumption(yaw=math.radians(120), offset_x=.9),
    'right_repeater': GroundAssumption(yaw=math.radians(-120), offset_x=-.9),
}


class ObservedSceneTimeline:
    def __init__(self, rows, sample_hz=5., camera='front', image_size=(1280, 960), assumption=None):
        # One camera at a time until cross-camera calibration/fusion is available.
        if not math.isfinite(sample_hz) or sample_hz <= 0: raise ValueError('Invalid sampling rate')
        times=[float(row['time_s']) for row in rows]
        if not np.isfinite(times).all() or any(b<=a for a,b in zip(times,times[1:])):
            raise ValueError('Observation times must be finite and increasing')
        self.max_gap=1.5/sample_hz
        self.rows = rows
        self.camera = camera
        self.assumption = assumption or ASSUMPTIONS[camera]
        self.ego = EgoTimeline(rows, max_gap=1.5/sample_hz)
        self.scenes = []
        previous = {}
        width, height = image_size
        for row, (data, t, spin) in zip(rows, self.ego.states):
            actors, current, rejected = [], {}, 0
            camera_data = row.get('cameras', {}).get(camera, {})
            objects = camera_data.get('objects', []) if camera_data.get('available') else []
            for obj in objects:
                if obj.get('track_status') != 'confirmed':
                    continue
                x, y, w, h = obj['bbox']
                if not np.isfinite([x,y,w,h]).all() or w <= 0 or h <= 0:
                    rejected += 1
                    continue
                ground = self.assumption.project(x+w/2, y+h, width, height)
                if ground is None:
                    rejected += 1
                    continue
                gx, gz = ground
                key = obj['track_id']
                old = previous.get(key)
                if old and 0 < t-old[0] <= 1.5/sample_hz and math.hypot(gx-old[1],gz-old[2]) < 8:
                    blend = 1-math.exp(-(t-old[0])/.25)
                    gx, gz = old[1]+blend*(gx-old[1]), old[2]+blend*(gz-old[2])
                current[key] = (t,gx,gz)
                # No snapping to lanes, no inferred direction, no occlusion extrapolation.
                actors.append(Actor(key,obj['kind'],gx,gz,confidence=obj['confidence'],estimated=True))
            previous = current
            self.scenes.append(Scene(t,data,spin*.343,actors,mode='observed',
                note=f'{camera} / approximate placement / heading unknown',rejected=rejected))

    def at(self, index):
        return self.scenes[index]

    def sample(self, t):
        """Interpolate only IDs observed at both bracketing samples; never extrapolate."""
        if not self.scenes: raise ValueError('Empty observation timeline')
        times=[s.time for s in self.scenes]
        i=max(0,bisect_right(times,t)-1)
        a=self.scenes[i]
        if i+1>=len(times) or t<=a.time:
            return a
        b=self.scenes[i+1]
        if b.time-a.time>self.max_gap:
            return Scene(t,None,a.distance,mode=a.mode,note='Observation gap / no extrapolation')
        q=(t-a.time)/(b.time-a.time)
        next_actors={actor.key:actor for actor in b.actors}
        actors=[]
        for actor in a.actors:
            other=next_actors.get(actor.key)
            if other is None:
                continue
            actors.append(replace(actor,x=actor.x+(other.x-actor.x)*q,
                z=actor.z+(other.z-actor.z)*q,
                yaw=actor.yaw+((other.yaw-actor.yaw+math.pi)%(2*math.pi)-math.pi)*q))
        return Scene(t,a.telemetry,a.distance+(b.distance-a.distance)*q,actors,
                     mode=a.mode,note=a.note,rejected=a.rejected)


def demo_scene(t):
    """Synthetic motion only. Never inserted into a real observation timeline."""
    speed = 54. if t < 6 else max(18.,54.-(t-6)*9)
    distance = 15*t if t < 6 else 90+15*(t-6)-1.25*min(t-6,4)**2
    curve = lambda z: .0024*z*z
    lanes = []
    for x in (-5.4,-1.8,1.8,5.4):
        lanes.append({'points':[(x+curve(z),z) for z in np.linspace(-13,55,100)],
                      'dashed': abs(x)<5, 'observed':False})
    change = np.clip((t-2)/4,0,1)
    change = float(change*change*(3-2*change))
    z = 17-1.1*t
    actors = [Actor('lead','car',curve(z)+3.6*(1-change),z,yaw=math.atan(.0048*z)-.12*math.sin(change*math.pi)),
              Actor('passing','suv',3.6+curve(-8+t*3),-8+t*3,yaw=math.atan(.0048*(-8+t*3))),
              Actor('truck','truck',-3.6+curve(32-t*.7),32-t*.7),
              Actor('far','car',curve(44-t),44-t),
              Actor('walker','person',7.3,11-t*.45,math.pi/2)]
    for i in range(4):
        actors.append(Actor(f'cone{i}','cone',-6.2+curve(7+i*4),7+i*4))
    return Scene(t,{'speed_kmh':round(speed),'brake_applied':t>=6,
                   'left_blinker':2<=t<6,'right_blinker':False},distance,actors,lanes,
                 [(curve(z),z) for z in np.linspace(1,40,70)],'demo','SYNTHETIC DEMO / not reconstructed footage')


@lru_cache(maxsize=24)
def fleet(kind,ghost=False):
    names={'car':'sedan','compact_car':'sedan','suv':'suv','van':'van','compact_van':'van',
           'truck':'box_truck','truck_box':'box_truck','truck_heavy':'box_truck','truck_medium':'box_truck',
           'small_truck':'small_truck','bus':'bus','person':'pedestrian',
           'bicycle':'cyclist','motorcycle':'motorcycle','cone':'traffic_cone','traffic_light':'traffic_light_head'}
    return EgoModel(ASSET.parent/f'tesla_fsd_{names.get(kind,"sedan")}{"_ghost" if ghost else ""}.glb')


@lru_cache(maxsize=48)
def asset_geometry_bounds(asset):
    transforms=asset.transforms(0.)
    points=np.concatenate([pos@transforms[i][:3,:3].T+transforms[i][:3,3]
                           for i,parts in asset.parts.items() for pos,_,_,_ in parts])
    return points.min(axis=0),points.max(axis=0)


def actor_mesh_transform(asset,size):
    """Ground and scale a rendered asset to the geometry used for placement."""
    low,high=asset_geometry_bounds(asset)
    target=np.asarray([size[0],size[2],size[1]],float)
    if not np.isfinite(target).all() or np.any(target<=0):raise ValueError('Invalid actor dimensions')
    origin=np.array([(low[0]+high[0])/2,low[1],(low[2]+high[2])/2])
    return origin,target/np.maximum(high-low,1e-8)


class DrivingRenderer:
    def __init__(self, width=960, height=640):
        self.width,self.height = width,height
        # Fixed rear upper camera, approximately 45 degrees above the ego.
        self.eye=np.array([5.,20.,-20.])
        target=np.array([0.,0.,2.])
        forward=target-self.eye; forward/=np.linalg.norm(forward)
        right=np.cross(forward,np.array([0.,1.,0.])); right/=np.linalg.norm(right)
        up=np.cross(right,forward)
        self.basis=np.column_stack((right,-up,forward))
        self.focal=min(width*.95,height*1.0)

    def project(self, points):
        camera=(np.asarray(points)-self.eye)@self.basis
        z=camera[:,2]
        return np.column_stack((self.width*.5+self.focal*camera[:,0]/z,
                                self.height*.58+self.focal*camera[:,1]/z,1/z))

    def polygon(self, canvas, depth, points, color):
        points=np.asarray(points,dtype=float)
        if np.any(((points-self.eye)@self.basis)[:,2]<=.1): return
        projected=self.project(points)
        tris=np.array([[0,i,i+1] for i in range(1,len(points)-1)],dtype=np.int64)
        raster(canvas,depth,projected,tris,np.tile(np.array(color,dtype=float),(len(points),1)))

    def strip(self, canvas, depth, points, color, width=.10, dashed=False, distance=0., elevation=.015):
        for a,b in zip(points,points[1:]):
            a,b=np.array(a,dtype=float),np.array(b,dtype=float)
            if dashed and ((a[1]+distance)%7)>3.5: continue
            d=b-a; norm=np.linalg.norm(d)
            if norm<1e-6: continue
            n=np.array([d[1],-d[0]])/norm*width/2
            corners=[a+n,a-n,b-n,b+n]
            self.polygon(canvas,depth,[(p[0],elevation,p[1]) for p in corners],color)

    def mesh(self, canvas, depth, asset, x=0.,z=0.,yaw=0.,spin=0.,telemetry=None,t=0.,neutral=False,y=0.,signal_state=None,size=None):
        c,s=math.cos(yaw),math.sin(yaw)
        rotate=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        transforms=asset.transforms(spin)
        mesh_origin,mesh_scale=actor_mesh_transform(asset,size) if size is not None else (np.zeros(3),np.ones(3))
        light=np.array([-.4,.86,-.32]); light/=np.linalg.norm(light)
        for i,parts in asset.parts.items():
            name=asset.g['nodes'][i].get('name','')
            active=False
            if name.startswith('BrakeLight'): active=bool(telemetry and telemetry.get('brake_applied'))
            elif name.startswith('TurnSignal'):
                side='left_blinker' if name.endswith(('FL','RL','SideL')) else 'right_blinker'
                active=bool(telemetry and telemetry.get(side)) and t%.66<.33
            transform=transforms[i]
            for pos,normals,tri,mat in parts:
                local=(pos@transform[:3,:3].T+transform[:3,3]-mesh_origin)*mesh_scale
                world=local@rotate.T+[x,y,z]
                if np.any(((world-self.eye)@self.basis)[:,2]<=.1): continue
                normal=(normals@transform[:3,:3].T/mesh_scale)@rotate.T
                normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-8)
                shade=.48+.52*np.maximum(0,normal@light)
                base=np.array(mat['pbrMetallicRoughness']['baseColorFactor'][:3])
                if neutral:
                    # FSD-style neutral surrounding actors; do not imply observed paint or lights.
                    grey=.55 if base.mean()>.35 else .30
                    base=np.array([grey,grey+.015,grey+.025])
                elif asset is model() and 'emissiveFactor' in mat:
                    base=base if active else base*.20
                    if active: shade=np.ones_like(shade)
                colors=shade[:,None]*base[::-1]*255
                if signal_state is not None and 'Signal' in mat['name']:
                    colors=np.tile(base[::-1]*255*.10,(len(pos),1))
                    active=np.zeros(len(pos),dtype=bool)
                    if signal_state=='red' and mat['name']=='FSDRedSignal':active[:]=True
                    if signal_state=='amber' and mat['name']=='FSDSignalAmber':active[:]=True
                    if signal_state=='green' and mat['name']=='FSDSignalGreen':active=pos[:,0]>.35
                    colors[active]=base[::-1]*255
                # Distance fade is visual only; all actors retain actual scene coordinates.
                fade=np.clip((world[:,2]-45)/100,0,.35)[:,None]
                colors=colors*(1-fade)+np.array([245,245,243])*fade
                raster(canvas,depth,self.project(world),tri,np.ascontiguousarray(colors))

    def render(self, scene):
        scene=road_display_scene(scene)
        canvas=np.full((self.height,self.width,3),(247,247,245),np.uint8)
        depth=np.full((self.height,self.width),-np.inf)
        self.polygon(canvas,depth,[(-45,-.03,-15),(45,-.03,-15),(45,-.03,85),(-45,-.03,85)],(237,237,235))
        road=getattr(scene,'road_surface',[])
        for surface in getattr(scene,'road_surfaces',[]):
            self.polygon(canvas,depth,[(x,-.01,z) for x,z in surface],(229,228,226))
        # Curved corridor polygons need strips, not a concave fan triangulation.
        if road:
            half=len(road)//2;left=road[:half];right=list(reversed(road[half:]))
            for i in range(half-1):
                self.polygon(canvas,depth,[(p[0],-.01,p[1]) for p in (left[i],right[i],right[i+1],left[i+1])],(229,228,226))
        if scene.path:
            self.strip(canvas,depth,scene.path,(247,202,149),width=1.5,elevation=.003)
            self.strip(canvas,depth,scene.path,(233,147,42),width=.12,elevation=.006)
        for lane in scene.lanes:
            self.strip(canvas,depth,lane['points'],(180,177,171) if lane.get('inferred') else ((174,176,176) if lane.get('observed') else (255,255,255)),
                       width=.12,dashed=lane.get('dashed',False),distance=scene.distance)
        # Soft ground contact shadows, drawn before geometry; all meshes share a depth buffer.
        ex,ez,eyaw=getattr(scene,'ego_display',(0.,0.,0.))
        actors=[Actor('ego','ego',ex,ez,eyaw)]+scene.actors
        for actor in actors:
            if actor_is_ghost(actor):continue
            radius=.35 if actor.kind in ('cone','person') else 1.05
            length=.35 if actor.kind in ('cone','person') else 2.3
            for expansion,shade in [(1.2,226),(1.,217),(.8,208)]:
                pts=[(actor.x+radius*expansion*math.cos(a),.001,actor.z+length*expansion*math.sin(a)) for a in np.linspace(0,2*math.pi,25)]
                self.polygon(canvas,depth,pts,(shade,shade,shade))
        for actor in scene.actors:
            if not actor_is_ghost(actor):
                size=(actor.width,actor.length,actor.height) if actor.height is not None else None
                self.mesh(canvas,depth,fleet(actor.kind),actor.x,actor.z,actor.yaw,neutral=actor.kind!='cone',size=size)
        self.mesh(canvas,depth,model(),ex,ez,eyaw,spin=scene.distance/.343,telemetry=scene.telemetry,t=scene.time)
        ghosts=sorted((a for a in scene.actors if actor_is_ghost(a)),
                      key=lambda a:((np.array([a.x,0,a.z])-self.eye)@self.basis)[2],reverse=True)
        for actor in ghosts:
            asset=fleet(actor.kind,ghost=True)
            layer=canvas.copy();ghost_depth=depth.copy()
            size=(actor.width,actor.length,actor.height) if actor.height is not None else None
            self.mesh(layer,ghost_depth,asset,actor.x,actor.z,actor.yaw,size=size)
            mask=ghost_depth>depth+1e-8
            alphas=[m.get('pbrMetallicRoughness',{}).get('baseColorFactor',[1,1,1,.35])[3] for m in asset.g['materials']]
            age=np.clip(actor.prediction_age/max(.1,actor.prediction_horizon),0,1)
            opacity=float(np.clip(np.median(alphas),.25,.55))*(1-.75*age)
            canvas[mask]=(canvas[mask]*(1-opacity)+layer[mask]*opacity).astype(np.uint8)
            # Transparent geometry does not occlude other transparent surfaces;
            # opaque scene depth continues to hide ghosts behind real objects.
        # Fade into a clean horizon instead of a hard rectangular ground plane.
        fade=np.clip((self.height*.30-np.arange(self.height))/(self.height*.24),0,1)[:,None,None]
        canvas=(canvas*(1-fade)+np.array([247,247,245])*fade).astype(np.uint8)
        # Elevated signal heads must stay legible above the road's horizon fade.
        # They still use the shared depth buffer and the same fixed camera.
        for signal in getattr(scene,'signals',[]):
            if 'position_estimate' in signal:
                sx,sy,sz=signal['position_estimate']
                self.mesh(canvas,depth,fleet('traffic_light'),sx,sz,math.pi,y=sy-.25,signal_state=signal['state'])
        speed='--' if scene.telemetry is None else str(scene.telemetry.get('speed_kmh','--'))
        cv2.putText(canvas,speed,(30,75),cv2.FONT_HERSHEY_SIMPLEX,1.8,(54,54,50),3,cv2.LINE_AA)
        cv2.putText(canvas,'km/h',(33,101),0,.45,(127,127,120),1,cv2.LINE_AA)
        mode={'demo':'DESIGN DEMO','fused':'MULTI-CAMERA FUSION / ESTIMATED'}.get(scene.mode,'OBSERVED / ESTIMATED SPACE')
        cv2.putText(canvas,mode,(self.width-300,33),0,.48,(125,125,117),1,cv2.LINE_AA)
        cv2.putText(canvas,f'{scene.time:05.2f} s',(self.width-95,57),0,.44,(145,145,135),1,cv2.LINE_AA)
        predictions=sum(actor_is_ghost(a) for a in scene.actors)
        uncertain=sum(a.tracking_state!='observed' and a.image_observed for a in scene.actors)
        if predictions or uncertain:
            parts=[]
            if predictions:parts.append(f'{predictions} ghost')
            if uncertain:parts.append(f'{uncertain} visible / position estimated')
            cv2.putText(canvas,' | '.join(parts),(24,self.height-70),0,.45,(135,100,65),1,cv2.LINE_AA)
        if getattr(scene,'road_view',None):
            label='ESTIMATED TURN CORRIDOR' if getattr(scene,'ego_turn_path_estimated',False) else 'ESTIMATED TRAVEL LANES'
            if getattr(scene,'junction_surface_estimated',False):label='ESTIMATED JUNCTION'
            cv2.putText(canvas,label,(24,125),0,.43,(135,110,85),1,cv2.LINE_AA)
            if getattr(scene,'ego_position_state','')=='held_uncertain':
                cv2.putText(canvas,'EGO LATERAL HELD / ROAD UNOBSERVED',(24,145),0,.4,(90,110,155),1,cv2.LINE_AA)
        elif road:
            cv2.putText(canvas,'CAMERA ROAD / ESTIMATED EXTENT',(24,125),0,.43,(135,110,85),1,cv2.LINE_AA)
        prior=getattr(scene,'road_prior',None)
        if prior:
            # Separate schematic: unregistered map lines must not masquerade as
            # camera-derived lane markings in the reconstructed 3D scene.
            w,h=min(180,self.width//3),110;x0=self.width-w-14;y0=80
            tile=np.full((h,w,3),(241,240,237),np.uint8)
            project=lambda p:(round(w/2-p[0]*1.15),round(h-24-p[1]*.9))
            for path in prior['paths']:
                a,b=map(project,path['points'])
                cv2.line(tile,a,b,(155,151,140) if path['matched_way'] else (195,192,185),2,cv2.LINE_AA)
            for point in prior.get('junctions',[]):
                cv2.drawMarker(tile,project(point),(65,135,200),cv2.MARKER_DIAMOND,10,2,cv2.LINE_AA)
            cv2.circle(tile,project([0,0]),3,(95,125,155),-1)
            cv2.putText(tile,'OSM SHAPE / GPS',(5,13),0,.33,(100,100,95),1,cv2.LINE_AA)
            cv2.putText(tile,'NOT LANE GEOMETRY',(5,h-5),0,.29,(100,100,95),1,cv2.LINE_AA)
            canvas[y0:y0+h,x0:x0+w]=tile
            indication=(getattr(scene,'road_context',None) or {}).get('junction_indication')
            if indication:
                label='OSM + CAMERA JUNCTION' if indication['camera_confirmed'] else 'OSM JUNCTION CANDIDATE'
                cv2.putText(canvas,label,(24,245),0,.43,(65,120,170),1,cv2.LINE_AA)
                cv2.putText(canvas,indication['state'].upper()+' / GPS APPROXIMATE',(24,264),0,.35,(100,120,140),1,cv2.LINE_AA)
            cv2.putText(canvas,'Map: OpenStreetMap contributors',(24,self.height-8),0,.36,(135,110,85),1,cv2.LINE_AA)
        if getattr(scene,'review_assisted',False):
            cv2.putText(canvas,'VISUAL REVIEW ASSISTED',(self.width-260,80),0,.42,(130,100,60),1,cv2.LINE_AA)
        for i,signal in enumerate(getattr(scene,'signals',[])[:3]):
            left=30+i*65; top=155
            cv2.rectangle(canvas,(left,top),(left+48,top+28),(80,80,80),-1)
            for j,name in enumerate(('red','amber','green')):
                color={'red':(30,30,235),'amber':(0,180,245),'green':(50,190,50)}[name] if signal['state']==name else (135,135,135)
                cv2.circle(canvas,(left+9+j*15,top+14),5,color,-1)
            label='?' if signal['state']=='unknown' else signal['state']
            cv2.putText(canvas,label,(left+16,top+45),0,.45,(90,90,90),1,cv2.LINE_AA)
        if getattr(scene,'signals',[]):
            cv2.putText(canvas,'Signal candidates / ? = unknown',(30,222),0,.35,(110,110,110),1,cv2.LINE_AA)
        cv2.putText(canvas,scene.note,(24,self.height-20),0,.43,(135,135,123),1,cv2.LINE_AA)
        if scene.mode=='fused':
            coverage=getattr(scene,'coverage',{})
            cameras=sum(state=='ok' for state in coverage.values())
            cv2.putText(canvas,f'{cameras} cameras | {len(scene.actors)} vehicles/objects | {scene.rejected} unplaced',
                        (24,self.height-40),0,.40,(135,135,123),1,cv2.LINE_AA)
        elif scene.mode!='demo':
            cv2.putText(canvas,'No lane / path inferred. One camera; no cross-camera fusion.',(24,self.height-40),0,.40,(135,135,123),1,cv2.LINE_AA)
        if scene.telemetry:
            if scene.telemetry.get('brake_applied'):
                cv2.putText(canvas,'BRAKE',(33,127),0,.42,(75,75,205),1,cv2.LINE_AA)
            if scene.time%.66<.33:
                if scene.telemetry.get('left_blinker'): cv2.arrowedLine(canvas,(self.width//2-35,42),(self.width//2-65,42),(90,160,70),3,tipLength=.4)
                if scene.telemetry.get('right_blinker'): cv2.arrowedLine(canvas,(self.width//2+35,42),(self.width//2+65,42),(90,160,70),3,tipLength=.4)
        return canvas
