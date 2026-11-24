"""Camera geometry, cuboid fitting and conservative multi-camera association.

Profiles are explicitly estimated, not factory calibrations. Never merge two
simultaneous detections from one camera, or move vehicles apart to hide bad fits.
"""
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import numpy as np
import cv2
from core.driving_scene import Actor, Scene, ObservedSceneTimeline
from core.scene_preview import EgoTimeline
from core.scene_analysis import iou
from core.scene_geometry import footprints_overlap

VEHICLES={'car','truck','bus'}
PROFILE_DIR=Path(__file__).resolve().parents[1]/'assets/scene/profiles'


@dataclass
class CameraGeometry:
    width: int = 1280
    height: int = 960
    focal: float = 850.
    horizon: float = 474.
    yaw: float = 0.
    camera_height: float = 1.28
    x: float = 0.
    z: float = 1.2
    principal_x: float | None = None
    rig_pitch: float = 0.

    @property
    def rotation(self):
        pitch=math.atan((self.height/2-self.horizon)/self.focal)
        c,s=math.cos(pitch),math.sin(pitch)
        a,b=math.cos(self.yaw),math.sin(self.yaw)
        result=np.array([[a,0,b],[0,1,0],[-b,0,a]])@np.array([[1,0,0],[0,c,-s],[0,s,c]])
        if self.rig_pitch:
            c,s=math.cos(self.rig_pitch),math.sin(self.rig_pitch)
            result=np.array([[1,0,0],[0,c,-s],[0,s,c]])@result
        return result

    @property
    def origin(self): return np.array([self.x,self.camera_height,self.z])

    def ground(self,u,v):
        cx=self.width/2 if self.principal_x is None else self.principal_x
        ray=self.rotation@np.array([-(u-cx)/self.focal,-(v-self.height/2)/self.focal,1.])
        if not np.isfinite(ray).all() or ray[1]>=-.015: return None
        point=self.origin-ray*self.camera_height/ray[1]
        if np.linalg.norm(point[[0,2]])>65: return None
        return point[[0,2]]

    def pixels(self,world):
        local=(np.asarray(world)-self.origin)@self.rotation
        if np.any(local[:,2]<=.10): return None
        cx=self.width/2 if self.principal_x is None else self.principal_x
        return np.column_stack((cx-self.focal*local[:,0]/local[:,2],
                               self.height/2-self.focal*local[:,1]/local[:,2]))

    def visible_bounds(self,world):
        """Clip a convex vehicle envelope at the near plane and image boundary.

        A nearby vehicle can straddle the camera plane; its visible part must
        not disappear merely because another corner is behind the camera.
        """
        local=(np.asarray(world)-self.origin)@self.rotation
        if not np.isfinite(local).all():return None
        near=.1
        visible=[p for p in local if p[2]>=near]
        if not visible:return None
        if len(visible)!=len(local):
            # All-pair crossings contain the actual clipped hull edges and only
            # interior extra points. Their convex hull is the same clipped hull.
            for i,a in enumerate(local):
                for b in local[i+1:]:
                    if (a[2]-near)*(b[2]-near)<0:
                        visible.append(a+(near-a[2])/(b[2]-a[2])*(b-a))
        points=np.asarray(visible);cx=self.width/2 if self.principal_x is None else self.principal_x
        pixels=np.column_stack([cx-self.focal*points[:,0]/points[:,2],self.height/2-self.focal*points[:,1]/points[:,2]])
        lo,hi=pixels.min(0),pixels.max(0)
        if np.all(lo>=0) and np.all(hi<=[self.width,self.height]):return np.r_[lo,hi]
        hull=cv2.convexHull(pixels.astype(np.float32))
        rectangle=np.array([[0,0],[self.width,0],[self.width,self.height],[0,self.height]],np.float32)
        area,polygon=cv2.intersectConvexConvex(hull,rectangle)
        if polygon is None or area<=1:return None
        p=polygon.reshape(-1,2)
        return np.r_[p.min(0),p.max(0)].astype(float)


def default_profile():
    # Repeater horizons are clip-balanced development-track estimates, not
    # factory intrinsics. The former 585/545 values put visible road contacts
    # above the assumed horizon and dropped confirmed approaching vehicles.
    # Provenance and independent evaluation: SCENE_ACCEPTANCE.md.
    return {'name':'HW3 preview geometry — empirical horizon prior','status':'estimated',
            'lane_width':None,'cameras':{
        'front':dict(focal=850.,horizon=474.,yaw=0.,camera_height=1.28,x=0.,z=1.2),
        'back':dict(focal=650.,horizon=300.,yaw=180.,camera_height=.8,x=0.,z=-2.2),
        'left_repeater':dict(focal=800.,horizon=504.,yaw=146.,camera_height=.9,x=.9,z=.8),
        'right_repeater':dict(focal=800.,horizon=486.,yaw=-145.,camera_height=.9,x=-.9,z=.8),
        'left_pillar':dict(focal=800.,horizon=480.,yaw=65.,camera_height=1.1,x=.9,z=.3),
        'right_pillar':dict(focal=800.,horizon=480.,yaw=-65.,camera_height=1.1,x=-.9,z=.3)}}


def load_profile(manifest):
    """Clip-specific reference observations must not silently apply to other clips."""
    filename=manifest.get('cameras',{}).get('front',{}).get('file','')
    if PROFILE_DIR.is_dir():
        for path in sorted(PROFILE_DIR.glob('*.json')):
            profile=json.loads(path.read_text(encoding='utf-8'))
            if filename in profile.get('front_files',[]): return profile
    return default_profile()


def cameras_for(manifest,profile):
    result={}
    for name,metadata in manifest.get('cameras',{}).items():
        values=dict(profile.get('cameras',{}).get(name,{}))
        if not values: continue
        width,height=metadata.get('width',1280),metadata.get('height',960)
        if width<=0 or height<=0: continue
        # Profile pixels are expressed in 1280x960 reference resolution.
        sx,sy=width/1280,height/960
        if abs(sx-sy)>1e-3: continue  # Reject unknown crop/aspect ratio instead of warping calibration.
        values['focal']*=sx; values['horizon']*=sy
        if 'principal_x' in values: values['principal_x']*=sx
        values['yaw']=math.radians(values['yaw'])
        rectification=metadata.get('rectification',{})
        if rectification.get('status')=='applied':
            matrix=np.asarray(rectification['camera_matrix'],float)
            if matrix.shape!=(3,3) or not np.isfinite(matrix).all():raise ValueError('Invalid rectified camera matrix')
            if abs(matrix[0,0]-matrix[1,1])>1e-6 or abs(matrix[1,2]-height/2)>1e-6:raise ValueError('Unsupported rectified camera geometry')
            pitch_tangent=(height/2-values['horizon'])/values['focal']
            values['focal']=float(matrix[0,0])
            values['horizon']=height/2-values['focal']*pitch_tangent
            values['principal_x']=float(matrix[0,2])
        result[name]=CameraGeometry(width=width,height=height,**values)
    return result


def dimensions(kind):
    # Representative shape priors, not a claim about the detected vehicle model.
    if kind=='person': return .55,.6,1.7
    if kind in ('bicycle','motorcycle'): return .7,1.9,1.65
    # COCO cycle boxes describe the vehicle; riders are separate person labels.
    # Rendering assets include a rider, so keep their display height separate.
    if kind in ('bicycle_body','motorcycle_body'): return .7,1.9,1.1
    if kind=='bus': return 2.5,10.,3.1
    if kind=='truck': return 2.,5.2,2.0
    if kind=='truck_box': return 2.1,5.5,2.7
    if kind=='truck_medium': return 2.3,7.,3.3
    if kind=='truck_heavy': return 2.5,10.,3.8
    if kind=='suv': return 1.98,4.95,1.8
    if kind=='compact_car':return 1.65,3.6,1.55
    if kind=='compact_van':return 1.65,3.6,1.75
    if kind=='small_truck':return 1.9,5.1,2.05
    if kind=='van':return 2.,5.1,2.0
    return 1.9,4.7,1.65


def corners(x,z,size,yaw=0.,kind=None):
    w,l,h=size
    if kind in (None,'car','suv','compact_car') and 3<l<7:
        # A full-height rectangular box exaggerates the near roof/bumper silhouette,
        # biasing adjacent cars outward and farther away. Use a sloped vehicle envelope.
        roof=.38 if kind=='suv' else .22
        points=np.array([(a,b,c) for a in (-w/2,w/2) for b in (0,h*.55) for c in (-l/2,l/2)]+
                        [(a,h,c) for a in (-w*.4,w*.4) for c in (-l*roof,l*roof)])
    elif kind=='small_truck':
        # An open load bed is not a full-height box. The high cab is at +Z.
        # A box envelope lets yaw compensate for an imaginary tall rear cargo.
        points=np.array([(a,b,c) for a in (-w*.48,w*.48) for b in (0,h*.45) for c in (-l/2,l*.45)]+
                        [(a,h*.72,c) for a in (-w/2,w/2) for c in (l*.16,l*.49)]+
                        [(a,h,c) for a in (-w*.43,w*.43) for c in (l*.17,l*.37)])
    elif kind in ('bicycle','motorcycle','bicycle_body','motorcycle_body'):
        # Tyre contacts are narrow; handlebars and rider are wider above them.
        # A full-width rectangular ground footprint biases grazing bike boxes.
        points=np.array([(a,0,c) for a in (-w*.12,w*.12) for c in (-l/2,l/2)]+
                        [(a,h*.55,c) for a in (-w/2,w/2) for c in (-l*.3,l*.3)]+
                        [(a,h,c) for a in (-w*.25,w*.25) for c in (-l*.15,l*.2)])
    else:
        points=np.array([(a,b,c) for a in (-w/2,w/2) for b in (0,h) for c in (-l/2,l/2)])
    cs,sn=math.cos(yaw),math.sin(yaw)
    points=points@np.array([[cs,0,-sn],[0,1,0],[sn,0,cs]])
    return points+[x,0,z]


def box_fit_scales(camera,w,h,weighted=True):
    scale=np.array([max(w,60),max(h,60)]*2,dtype=float)
    if weighted and abs(camera.yaw)<math.radians(15):
        # Small distant boxes have uncertain pixel contacts. Ramp precision
        # instead of turning a one-pixel detection fluctuation into metres.
        strength=float(np.clip((h-40.)/60.,0.,1.))**2
        contact_sigma=max(3.,min(8.,h*.03))
        scale[3]/=1.+strength*(scale[3]/contact_sigma-1.)
    return scale


def _fit_box(camera,box,kind,prioritize_contact=True,yaw=0.):
    """Fit a whole grounded cuboid; image-clipped edges are one-sided constraints."""
    x,y,w,h=map(float,box)
    if not np.isfinite(box).all() or w<8 or h<12: return None
    if y+h>=camera.height*.99: return None
    ground=camera.ground(x+w/2,y+h)
    if ground is None: return None
    size=dimensions(kind)
    direction=ground-camera.origin[[0,2]]
    direction/=max(.01,np.linalg.norm(direction))
    lateral=np.array([math.cos(yaw),-math.sin(yaw)])
    forward=np.array([math.sin(yaw),math.cos(yaw)])
    support=abs(direction@lateral)*size[0]/2+abs(direction@forward)*size[1]/2
    start=ground+direction*support
    target=np.array([x,y,x+w,y+h])
    scale=box_fit_scales(camera,w,h,weighted=False)
    # Body dimensions are class priors; the visible box bottom is our image
    # contact proxy (not a verified tyre keypoint). Equal relative edge weights let a wider/longer
    # prior float above that contact and push adjacent vehicles outward.
    # Keep the original unweighted fit/IoU gates; prioritize contact only in
    # the optimizer, never use a lane centre as a position constraint.
    # Repeaters' grazing/partly hidden tyre contacts and estimated distortion
    # do not support the same vertical precision as the forward view.
    optimization_scale=box_fit_scales(camera,w,h,weighted=prioritize_contact)

    def residual(position,weighted=True):
        prediction=camera.visible_bounds(corners(*position,size,yaw=yaw,kind=kind))
        if prediction is None: return np.ones(4)*20
        r=prediction-target
        if x<3: r[0]=max(0,r[0])
        if x+w>camera.width-3: r[2]=min(0,r[2])
        if y<3: r[1]=max(0,r[1])
        return r/(optimization_scale if weighted else scale)

    position=start.copy()
    for _ in range(15):
        r=residual(position)
        jac=np.column_stack([(residual(position+offset)-r)/.03 for offset in ([.03,0],[0,.03])])
        step=np.linalg.solve(jac.T@jac+np.eye(2)*.002,jac.T@r)
        step=np.clip(step,-1.5,1.5)
        candidate=position-step
        if np.dot(residual(candidate),residual(candidate))>np.dot(r,r): candidate=position-step*.25
        position=candidate
        if np.linalg.norm(step)<.003: break
    error=float(np.sqrt(np.mean(residual(position,weighted=False)**2)))
    clipped=x<3 or x+w>camera.width-3
    if error>(.18 if clipped else .24) or np.linalg.norm(position)>60 or not np.isfinite(position).all(): return None
    bounds=camera.visible_bounds(corners(*position,size,yaw=yaw,kind=kind))
    if bounds is None or iou(np.r_[bounds[:2],bounds[2:]-bounds[:2]],box)<.5:return None
    if abs(position[0])<1.4 and abs(position[1])<3.: return None  # Host/body/invalid near-ground fit.
    return position,size,error


def fit_box(camera,box,kind,yaw=0.):
    result=_fit_box(camera,box,kind,yaw=yaw)
    if result is None:
        # An imperfect/occluded contact must not discard an otherwise supported
        # detection. Preserve the original box-fit gates and fallback geometry.
        result=_fit_box(camera,box,kind,prioritize_contact=False,yaw=yaw)
    return result


@dataclass
class Candidate:
    camera: str
    local_id: str
    kind: str
    x: float
    z: float
    confidence: float
    size: tuple
    error: float
    clipped: bool = False
    render_kind: str | None = None
    bbox: tuple | None = None
    geometry_kind: str | None = None
    association_only: bool = False
    camera_geometry: CameraGeometry | None = None
    estimated_pitch: bool = False
    yaw: float = 0.


def compatible(a,b):
    return a==b or (a in VEHICLES and b in VEHICLES)


def fuse_candidates(candidates):
    """Complete-link spatial association, one observation per camera per cluster.

    Camera-local track histories supply temporal IDs separately. No appearance
    identity is claimed; ambiguous associations are retained as separate candidates.
    """
    clusters=[]
    for candidate in sorted(candidates,key=lambda c:(c.error,-c.confidence,c.camera,c.local_id)):
        choices=[]
        for i,cluster in enumerate(clusters):
            if any(c.camera==candidate.camera for c in cluster): continue
            if not all(compatible(c.kind,candidate.kind) for c in cluster): continue
            # Tight anisotropic gate: never join cars in adjacent 3m+ lanes.
            scores=[((c.x-candidate.x)/.85)**2+((c.z-candidate.z)/1.8)**2 for c in cluster]
            if max(scores)<1: choices.append((max(scores),i))
        choices.sort()
        if choices and (len(choices)==1 or choices[1][0]-choices[0][0]>.3):
            clusters[choices[0][1]].append(candidate)
        else: clusters.append([candidate])
    return clusters


def joint_position(cluster,cameras):
    """One 3D position must explain every camera, not just average their positions."""
    # Rear imagery is association evidence only. It must never change a pose
    # supported by front/repeater/pillar cameras, or seed a standalone pose.
    cluster=[c for c in cluster if c.camera!='back']
    if not cluster:return None
    weights=np.array([c.confidence/max(.035,c.error) for c in cluster]);weights/=weights.sum()
    position=np.average([(c.x,c.z) for c in cluster],axis=0,weights=weights)
    if len(cluster)<2 or any(c.bbox is None for c in cluster):return position
    best=min(cluster,key=lambda c:c.error)
    shape=best.geometry_kind or best.render_kind or best.kind;size=dimensions(shape)
    def residual(p,weighted=True):
        result=[]
        for c in cluster:
            camera=c.camera_geometry or cameras[c.camera];bounds=camera.visible_bounds(corners(*p,size,yaw=best.yaw,kind=shape))
            if bounds is None:return np.full(4*len(cluster),20.)
            x,y,w,h=c.bbox;r=bounds-np.array([x,y,x+w,y+h])
            if x<3:r[0]=max(0,r[0])
            if x+w>camera.width-3:r[2]=min(0,r[2])
            if y<3:r[1]=max(0,r[1])
            scale=box_fit_scales(camera,w,h,weighted)
            result.extend(r/scale)
        return np.array(result)
    for _ in range(12):
        r=residual(position)
        J=np.column_stack([(residual(position+d)-r)/.03 for d in ([.03,0],[0,.03])])
        step=np.clip(np.linalg.solve(J.T@J+np.eye(2)*.002,J.T@r),-.5,.5)
        proposed=position-step
        if np.linalg.norm(residual(proposed))>=np.linalg.norm(r):break
        position=proposed
        if np.linalg.norm(step)<.002:break
    per_camera=np.sqrt(np.mean(residual(position,weighted=False).reshape(-1,4)**2,axis=1))
    if max(per_camera)>.24:return None
    return position


class FusedSceneTimeline(ObservedSceneTimeline):
    def __init__(self,rows,manifest,profile=None,postprocess=True,enable_handoffs=True,select_truck_shapes=True,motion_progress=None):
        sample_hz=float(manifest['sample_hz'])
        if sample_hz<=0 or not math.isfinite(sample_hz): raise ValueError('Invalid sample rate')
        times=[float(row['time_s']) for row in rows]
        if not np.isfinite(times).all() or any(b<=a for a,b in zip(times,times[1:])):
            raise ValueError('Invalid observation timestamps')
        self.max_gap=1.5/sample_hz
        self.profile=profile or load_profile(manifest)
        self.truck_shape_priors={}
        if select_truck_shapes and any(o.get('kind')=='truck' for row in rows
                for data in row.get('cameras',{}).values() for o in data.get('objects',[])):
            # Offline prepass retains the same per-frame pitch estimates. No
            # image inference is repeated and source observations are unchanged.
            preliminary=FusedSceneTimeline(rows,manifest,self.profile,postprocess=False,
                enable_handoffs=False,select_truck_shapes=False)
            from core.scene_truck_shapes import infer_truck_shapes
            self.truck_shape_priors=infer_truck_shapes(rows,preliminary.camera_frames)
        self.cameras=cameras_for(manifest,self.profile)
        base_front=self.cameras.get('front')
        from core.scene_stopped_motion import stopped_motion_priors
        stopped_horizons,stopped_directions,stopped_reports=stopped_motion_priors(rows,base_front)
        reviewed=manifest.get('camera_calibration_review') or {}
        reviewed_bias=None;reviewed_trace=None
        if reviewed:
            front_name=manifest.get('cameras',{}).get('front',{}).get('file')
            reviewed_bias=float(reviewed.get('horizon_bias_px',0.))
            if reviewed.get('front_file')!=front_name or not math.isfinite(reviewed_bias) or abs(reviewed_bias)>45:
                raise ValueError('Invalid or wrong-clip camera calibration review')
            if reviewed.get('trace'):
                reviewed_trace=np.array([[p['time_s'],p['horizon_px']] for p in reviewed['trace']],float)
                if not np.isfinite(reviewed_trace).all() or np.any(np.diff(reviewed_trace[:,0])<=0) or np.max(abs(reviewed_trace[:,1]-base_front.horizon))>80:
                    raise ValueError('Invalid reviewed background pitch trace')
        horizon=base_front.horizon if base_front else None
        safe_horizon=horizon
        road_x=base_front.width/2 if base_front else None
        self.camera_frames=[]
        self.pitch_safe_frames=[]
        self.scenes=[]; self.audit=[]
        self.rows=rows
        self.camera='all'
        ego=EgoTimeline(rows,self.max_gap)
        aliases={}; previous={}; next_id=1; support_history={}
        for row_index,(row,(data,t,spin)) in enumerate(zip(rows,ego.states)):
            # Moving-car pitch/road grade changes cannot use a fixed image horizon.
            # Lane tangent is an explicit local planar-road proxy, not factory calibration.
            horizon_source='profile'
            if base_front and data and data.get('speed_kmh',0)>1:
                lines=[]
                front_evidence=row.get('cameras',{}).get('front',{})
                for segment in front_evidence.get('pitch_lanes',front_evidence.get('lanes',[])):
                    points=np.asarray(segment['points'],float)
                    if len(points)!=2:continue
                    p,q=points
                    if abs(q[1]-p[1])>=50:
                        lines.append(np.cross(np.r_[p,1],np.r_[q,1]))
                target=None
                if len(lines)>=2:
                    vp=np.cross(lines[0],lines[1])
                    if abs(vp[2])>1e-6:
                        u,v=vp[:2]/vp[2]
                        if .3*base_front.width<u<.7*base_front.width:
                            road_x=float(u);target=float(v);horizon_source='observed_lane_pair'
                elif len(lines)==1 and abs(lines[0][1])>1e-6:
                    a,b,c=lines[0];target=float(-(a*road_x+c)/b)
                    horizon_source='single_lane_with_previous_road_direction'
                if target is not None and abs(target-base_front.horizon)<80:
                    horizon+= (1-math.exp(-1/sample_hz/.15))*(target-horizon)
                    if horizon_source=='observed_lane_pair':
                        safe_horizon+=(1-math.exp(-1/sample_hz/.15))*(target-safe_horizon)
                else:horizon_source='previous_pitch_proxy'
                self.cameras['front']=replace(base_front,horizon=horizon)
            if reviewed_trace is not None:
                corrected=float(np.interp(t,reviewed_trace[:,0],reviewed_trace[:,1]))
                reference=float(reviewed['reference_horizon_px'])
                delta=math.atan((base_front.height/2-corrected)/base_front.focal)-math.atan((base_front.height/2-reference)/base_front.focal)
                for name,camera in list(self.cameras.items()):
                    self.cameras[name]=replace(camera,rig_pitch=delta) if name!='front' else replace(base_front,horizon=corrected)
                horizon_source='reviewed_static_background_rotation'
            elif reviewed_bias is not None and base_front:
                corrected=stopped_horizons.get(row_index,horizon+reviewed_bias)
                self.cameras['front']=replace(base_front,horizon=corrected)
                horizon_source='clip_reviewed_horizon_bias'
            elif row_index in stopped_horizons:
                # Offline shared stop calibration, independent of lane position.
                target=stopped_horizons[row_index]
                horizon+=(1-math.exp(-1/sample_hz/.3))*(target-horizon)
                safe_horizon=horizon
                self.cameras['front']=replace(base_front,horizon=horizon)
                horizon_source='stopped_vehicle_height_consensus'
            self.camera_frames.append(dict(self.cameras))
            safe_cameras=dict(self.cameras)
            if base_front:
                corrected_safe=corrected if reviewed_trace is not None else stopped_horizons.get(row_index,safe_horizon+reviewed_bias) if reviewed_bias is not None else safe_horizon
                safe_cameras['front']=replace(base_front,horizon=corrected_safe)
            self.pitch_safe_frames.append(safe_cameras)
            candidates=[]; rejected=[]; coverage={}; partials=[]
            for camera,camera_data in row.get('cameras',{}).items():
                coverage[camera]='ok' if camera_data.get('available') else 'unavailable'
                geometry=self.cameras.get(camera)
                if geometry is None:
                    coverage[camera]='no geometry'; continue
                if not camera_data.get('available'): continue
                source_time=camera_data.get('source_time_s',t)
                if not math.isfinite(source_time) or abs(source_time-t)>.08:
                    coverage[camera]='out of sync'; continue
                objects=camera_data.get('objects',[])
                for detection in objects:
                    if detection.get('track_status')!='confirmed': continue
                    if detection.get('road_relation')=='beyond_central_boundary':
                        rejected.append({'id':detection['track_id'],'reason':'beyond visible central boundary'})
                        continue
                    bx,by,bw,bh=detection['bbox']
                    if camera=='back' or detection.get('association_only'):
                        partials.append((camera,detection));continue
                    contact=(bx+bw/2,by+bh)
                    occluders=[]
                    for other in objects:
                        if other is detection or other.get('track_status')!='confirmed': continue
                        ox,oy,ow,oh=other['bbox']
                        if ox<contact[0]<ox+ow and oy<contact[1]<oy+oh-10:
                            occluders.append(other['track_id'])
                    if occluders:
                        rejected.append({'id':detection['track_id'],'reason':'ground contact occluded','occluders':occluders})
                        continue
                    fit_kind=detection.get('geometry_kind') or detection.get('render_kind') or detection['kind']
                    if detection['kind'] in ('bicycle','motorcycle'):
                        fit_kind=detection['kind']+'_body'
                    if detection['kind']=='truck':
                        if not detection.get('geometry_kind'):
                            fit_kind=self.truck_shape_priors.get(detection['track_id'],{}).get('shape',fit_kind)
                    direction=stopped_directions.get((row_index,detection['track_id'])) if camera=='front' else None
                    fit_yaw=direction['yaw'] if direction else 0.
                    fit=fit_box(geometry,detection['bbox'],fit_kind,yaw=fit_yaw)
                    fit_geometry=geometry;estimated_pitch=False
                    if fit_kind=='suv' and not detection.get('geometry_kind'):
                        # A class label does not determine its silhouette or size.
                        # Choose between declared compact/tall body priors using image evidence.
                        compact=fit_box(geometry,detection['bbox'],'car',yaw=fit_yaw)
                        if compact is not None and (fit is None or compact[2]<fit[2]):
                            fit,fit_kind=compact,'car'
                    if fit is None and camera=='front' and min(bw,bh)>=40 and horizon_source!='observed_lane_pair':
                        # Do not let a weak single-line pitch delete an observed
                        # vehicle. Retry with the last paired estimate, while
                        # keeping road projection unchanged. This pose is a ghost
                        # and is excluded from observed lane-vote evidence.
                        recovery=safe_cameras['front']
                        fit=fit_box(recovery,detection['bbox'],fit_kind,yaw=fit_yaw)
                        if fit is not None and np.linalg.norm(fit[0])>45:fit=None
                        if fit is not None:fit_geometry=recovery;estimated_pitch=True
                    if fit is None:
                        rejected.append({'id':detection['track_id'],'reason':'grounded box fit unsupported'}); continue
                    point,size,error=fit
                    bx,by,bw,bh=detection['bbox']
                    candidates.append(Candidate(camera,detection['track_id'],detection['kind'],*point,
                        detection['confidence'],size,error,bx<3 or bx+bw>geometry.width-3,detection.get('render_kind'),tuple(detection['bbox']),fit_kind,
                        camera_geometry=fit_geometry,estimated_pitch=estimated_pitch,yaw=fit_yaw))
            clusters=[]
            for group in fuse_candidates(candidates):
                if joint_position(group,self.cameras) is None:
                    rejected.append({'id':','.join(c.local_id for c in group),'reason':'cross-camera reprojection inconsistent'})
                    clusters.extend([[c] for c in group])
                else:clusters.append(group)
            # Partial boxes supply image evidence to an already localized object.
            # They cannot seed metric position or force an ambiguous identity merge.
            for camera,detection in partials:
                choices=[]
                for index,group in enumerate(clusters):
                    if any(c.camera==camera for c in group):continue
                    best=min(group,key=lambda c:c.error)
                    if not compatible(best.kind,detection['kind']):continue
                    position=joint_position(group,self.cameras);shape=best.geometry_kind or best.kind
                    bounds=self.cameras[camera].visible_bounds(corners(*position,dimensions(shape),yaw=best.yaw,kind=shape))
                    if bounds is None:continue
                    score=iou(np.r_[bounds[:2],bounds[2:]-bounds[:2]],detection['bbox'])
                    if score<.35:continue
                    support=Candidate(camera,detection['track_id'],detection['kind'],*position,
                        min(.8,detection['confidence']),dimensions(shape),.15,True,best.render_kind,
                        tuple(detection['bbox']),shape,True,yaw=best.yaw)
                    if joint_position(group+[support],self.cameras) is not None:choices.append((score,index,support))
                choices.sort(key=lambda x:x[0],reverse=True)
                if not choices or (len(choices)>1 and choices[0][0]-choices[1][0]<.15):
                    rejected.append({'id':detection['track_id'],'reason':'partial observation has no unique multi-camera support'});continue
                score,index,support=choices[0]
                key=(support.local_id,tuple(sorted(c.local_id for c in clusters[index])))
                last,hits=support_history.get(key,(-math.inf,0))
                hits=hits+1 if 0<t-last<=self.max_gap else 1
                support_history[key]=(t,hits)
                if hits<3:
                    rejected.append({'id':support.local_id,'reason':'partial association awaiting temporal evidence'});continue
                clusters[index].append(support)
            actors=[]; current={}; used=set(); associations=[]
            for cluster in clusters:
                primary=[c for c in cluster if c.camera!='back']
                weights=np.array([c.confidence/max(.035,c.error) for c in primary]); weights/=weights.sum()
                x,z=joint_position(cluster,self.cameras)
                sources=tuple(sorted(c.local_id for c in cluster))
                identity_sources=tuple(c.local_id for c in primary)
                ids={aliases[source][0] for source in identity_sources if source in aliases and t-aliases[source][1]<=self.max_gap}
                ids=sorted(identity for identity in ids if identity not in used)
                if not ids:
                    handoffs=[]
                    source_cameras={c.camera for c in primary}
                    for key,old in previous.items():
                        if key in used or not 0<t-old[0]<=self.max_gap: continue
                        if not source_cameras.isdisjoint(old[3]): continue
                        if not compatible(old[4],cluster[0].kind): continue
                        score=((x-old[1])/.7)**2+((z-old[2])/1.5)**2
                        if score<1: handoffs.append((score,key))
                    handoffs.sort()
                    if handoffs and (len(handoffs)==1 or handoffs[1][0]-handoffs[0][0]>.3):
                        ids=[handoffs[0][1]]
                identity=ids[0] if ids else f'fused:{next_id}'
                if not ids: next_id+=1
                used.add(identity)
                # Filter once, after ego-motion compensation in WorldTracks.
                # Camera-relative low-pass filtering here creates a second lag.
                current[identity]=(t,x,z,{c.camera for c in primary},primary[0].kind)
                for source in identity_sources: aliases[source]=(identity,t)
                best=max(primary,key=lambda c:c.confidence)
                # Preserve the temporally voted class; do not render every vehicle
                # as a sedan and discard the detector's truck/bus evidence.
                render_kind=best.render_kind or best.kind
                actor=Actor(identity,render_kind,float(x),float(z),confidence=max(c.confidence for c in primary),estimated=True)
                actor.yaw=best.yaw
                actor.sources=sources
                actor.width,actor.length=best.size[:2]
                actor.height=best.size[2]
                if best.geometry_kind in ('bicycle_body','motorcycle_body'):
                    actor.height=dimensions(best.kind)[2]
                actor.geometry_kind=best.geometry_kind or best.kind
                actor.source_geometries={c.local_id:c.camera_geometry or self.cameras[c.camera] for c in primary}
                if any(c.estimated_pitch for c in primary):actor.tracking_state='pitch_held'
                actor.placement_error=float(np.average([c.error+(.12 if c.clipped else 0.) for c in primary],weights=weights))
                actors.append(actor)
                associations.append({'id':identity,'sources':sources,'position':[float(x),float(z)],
                                     'placement_state':actor.tracking_state,
                                     'fit_yaw_rad':actor.yaw,
                                     'partial_support_sources':[c.local_id for c in cluster if c.association_only],
                                     'position_sources':[c.local_id for c in primary],
                                     'rear_role':'association_only_no_position_update',
                                     'identity_evidence':'geometric_temporal_hypothesis' if len(cluster)>1 else 'single_camera',
                                     'box_fit_error':float(np.average([c.error for c in primary],weights=weights))})
            # Reject weaker mutually incompatible placements; never repel or lane-snap vehicles.
            kept=[]
            for actor in sorted(actors,key=lambda a:(-sum(not s.startswith(('back:','review:back:')) for s in a.sources),a.placement_error,-a.confidence,a.key)):
                conflict=next((b for b in kept if footprints_overlap(actor,b)),None)
                if conflict:
                    rejected.append({'id':actor.key,'reason':'ambiguous overlapping footprint','sources':actor.sources})
                else: kept.append(actor)
            previous={a.key:current[a.key] for a in kept}
            lanes=[]
            front_data=row.get('cameras',{}).get('front',{})
            if 'lanes' in front_data and 'front' in self.cameras:
                for segment in front_data['lanes']:
                    points=[self.cameras['front'].ground(*p) for p in segment['points']]
                    if all(p is not None for p in points):
                        lanes.append({'points':[p.tolist() for p in points],'dashed':False,'observed':True,
                                      'source':segment['source'],'extrapolated':False})
            elif self.profile.get('lane_width'):
                width=self.profile['lane_width']
                for x in (-width/2,width/2):
                    lanes.append({'points':[(x,float(z)) for z in np.linspace(-10,35,70)],'dashed':True,'observed':False})
            scene=Scene(t,data,spin*.343,kept,lanes,mode='fused',
                note='FRONT + SIDE POSITION / rear support only / estimated',rejected=len(rejected))
            if front_data.get('road_evidence') is not None and 'front' in self.cameras:
                scene.camera_road_sections=[]
                scene.camera_road_edges=[]
                for section in front_data['road_evidence'].get('cross_sections',[]):
                    points=[self.cameras['front'].ground(*p) for p in section]
                    if all(p is not None for p in points):
                        scene.camera_road_sections.append([p.tolist() for p in points])
                for segment in front_data['road_evidence'].get('edge_segments',[]):
                    points=[self.cameras['front'].ground(*p) for p in segment]
                    if all(p is not None for p in points):
                        scene.camera_road_edges.append([p.tolist() for p in points])
            scene.coverage=coverage
            scene.junction_evidence=front_data.get('junction_evidence',{})
            scene.signals=[dict(s) for s in front_data.get('signals',[])]
            camera_geometry=self.cameras.get('front')
            if camera_geometry:
                for signal in scene.signals:
                    if signal.get('head_type')!='horizontal_candidate' or signal.get('confidence',0)<.45: continue
                    bx,by,bw,bh=signal['bbox']
                    # Representative GLB head width, not a surveyed distance.
                    distance=camera_geometry.focal*1.52/max(bw,1.)
                    cx=camera_geometry.principal_x if camera_geometry.principal_x is not None else camera_geometry.width/2
                    local=np.array([-(bx+bw/2-cx)/camera_geometry.focal*distance,
                                    -(by+bh/2-camera_geometry.height/2)/camera_geometry.focal*distance,distance])
                    world=camera_geometry.origin+camera_geometry.rotation@local
                    if 5<world[2]<60 and 3<world[1]<10:
                        signal['position_estimate']=world.tolist()
                        signal['position_source']='representative_head_width_not_measured'
            scene.review_assisted=bool(manifest.get('review_assisted'))
            scene.fusion_count=sum(len(c)-1 for c in clusters)
            self.scenes.append(scene)
            self.audit.append({'time_s':t,'coverage':coverage,'associations':associations,'rejected':rejected,
                               'front_horizon_px':self.cameras['front'].horizon if base_front else None,
                               'safe_front_horizon_px':safe_cameras['front'].horizon if base_front else None,
                               'front_horizon_source':horizon_source,
                               'stopped_motion_prior':stopped_reports.get(row_index),
                               'visible_ids':[a.key for a in kept]})
        from core.scene_world_tracks import WorldTracks,rotation,bridge_motion_consistent
        world=WorldTracks()
        poses=[];filter_states=[]
        for i,(scene,audit,row) in enumerate(zip(self.scenes,self.audit,rows)):
            image_support={}
            present_sources=set()
            for name,evidence in row.get('cameras',{}).items():
                if name=='back' or not evidence.get('available') or name not in self.camera_frames[i]:continue
                if abs(evidence.get('source_time_s',scene.time)-scene.time)>.08:continue
                for obj in evidence.get('objects',[]):
                    if obj.get('track_status')!='confirmed' or obj.get('association_only') or obj.get('road_relation')=='beyond_central_boundary':continue
                    present_sources.add(obj['track_id'])
                    if name!='front':continue
                    if obj.get('kind') not in ('car','truck','bus','bicycle','motorcycle'):continue
                    image_support[obj['track_id']]=dict(camera=self.pitch_safe_frames[i][name],bbox=obj['bbox'])
            scene.actors,events=world.update(scene.actors,scene.time,scene.telemetry,image_support)
            filter_states.append({a.key:(world.tracks[a.key]['state'].copy(),world.tracks[a.key]['cov'].copy(),
                                         world.tracks[a.key].get('process_accel_variance',2.25))
                                  for a in scene.actors if a.tracking_state=='observed'})
            audit['image_observed_ids']=[a.key for a in scene.actors
                if world.tracks[a.key]['sources']&present_sources]
            poses.append((world.pose.copy(),rotation(world.angle)))
            audit['world_track_events']=events
            audit['world_tracks']=[{'id':a.key,'position':[a.x,a.z],'state':a.tracking_state,
                'prediction_age_s':a.prediction_age,'conditional_sigma_m':a.position_uncertainty,
                'sources':a.sources} for a in scene.actors]
            audit['visible_ids']=[a.key for a in scene.actors]
        self.motion_smoothed_positions=0
        if postprocess and stopped_horizons:
            from core.scene_motion_smoothing import smooth_motion
            self.motion_smoothed_positions=smooth_motion(self.scenes,poses,filter_states,rows,
                self.camera_frames,self.audit,stopped_horizons)
        # Offline footage permits bounded two-sided reconstruction, unlike live driving.
        # Only bridge already-supported predictions, never create unseen actors.
        anchors={};older_anchors={}
        for i,scene in enumerate(self.scenes):
            for actor in scene.actors:
                if actor.tracking_state!='observed':continue
                previous_anchor=anchors.get(actor.key);older=older_anchors.get(actor.key)
                older_anchors[actor.key]=previous_anchor;anchors[actor.key]=(i,actor)
                if previous_anchor is None:continue
                j,before=previous_anchor
                gap=scene.time-self.scenes[j].time
                if i-j<2 or gap>1.5:continue
                between=[next((a for a in self.scenes[k].actors if a.key==actor.key and a.tracking_state=='predicted'),None) for k in range(j+1,i)]
                if any(a is None for a in between):continue
                p0=poses[j][0]+poses[j][1]@np.array([before.x,before.z])
                p1=poses[i][0]+poses[i][1]@np.array([actor.x,actor.z])
                if np.linalg.norm(p1-p0)/gap>55:continue
                v0=v1=None
                if older is not None:
                    k,old_actor=older;dt=self.scenes[j].time-self.scenes[k].time
                    if 0<dt<=.5:
                        pold=poses[k][0]+poses[k][1]@np.array([old_actor.x,old_actor.z]);v0=(p0-pold)/dt
                if i+1<len(self.scenes):
                    following=next((a for a in self.scenes[i+1].actors if a.key==actor.key and a.tracking_state=='observed'),None)
                    dt=self.scenes[i+1].time-scene.time
                    if following is not None and 0<dt<=.5:
                        pnext=poses[i+1][0]+poses[i+1][1]@np.array([following.x,following.z]);v1=(pnext-p1)/dt
                if not bridge_motion_consistent(v0,v1,gap):
                    self.audit[i]['world_track_events'].append({'id':actor.key,'reason':'offline bridge rejected: abrupt motion change'})
                    continue
                for k,a in zip(range(j+1,i),between):
                    q=(self.scenes[k].time-self.scenes[j].time)/gap
                    point=poses[k][1].T@((1-q)*p0+q*p1-poses[k][0])
                    if any(b.key!=a.key and abs(point[0]-b.x)<(a.width+b.width)/2 and abs(point[1]-b.z)<(a.length+b.length)/2 for b in self.scenes[k].actors):continue
                    a.x,a.z=map(float,point);a.tracking_state='interpolated'
                    self.audit[k]['world_track_events'].append({'id':a.key,'reason':'offline two-sided reconstruction',
                        'past_time_s':self.scenes[j].time,'future_time_s':scene.time})
        if postprocess:
            from core.scene_postprocess import recover_positions
            self.recovered_positions=recover_positions(self.scenes,rows,poses,self.camera_frames,self.audit)
        else:self.recovered_positions=0
        self.stitched_handoffs=0
        if postprocess and enable_handoffs:
            from core.scene_handoff import stitch_handoffs
            self.stitched_handoffs=stitch_handoffs(self.scenes,rows,poses,self.audit,
                {name:camera.width for name,camera in self.cameras.items()})
        from core.scene_motion_heading import apply_motion_headings
        self.motion_heading_count=apply_motion_headings(self.scenes,poses,self.audit,stopped_directions)
        self.motion_bundle_report=[]
        if postprocess and manifest.get('motion_bundle_refinement'):
            from core.scene_bundle_refinement import refine_bundles
            self.motion_bundle_report=refine_bundles(self.scenes,rows,poses,self.camera_frames,self.audit,progress=motion_progress)
        for scene,audit,row in zip(self.scenes,self.audit,rows):
            visible_sources=set()
            for name,data in row.get('cameras',{}).items():
                if name=='back' or not data.get('available'):continue
                if abs(data.get('source_time_s',scene.time)-scene.time)>.08:continue
                visible_sources.update(obj['track_id'] for obj in data.get('objects',[])
                    if obj.get('track_status')=='confirmed' and not obj.get('association_only')
                    and obj.get('road_relation')!='beyond_central_boundary')
            for actor in scene.actors:
                bundle_presence=actor.key.startswith('bundle:') and any(
                    obj['track_id'] in actor.sources for data in row.get('cameras',{}).values()
                    for obj in data.get('objects',[]))
                actor.image_observed=bundle_presence or actor.key in audit.get('image_observed_ids',[]) or bool(set(actor.sources)&visible_sources)
            audit['visible_ids']=[a.key for a in scene.actors]
            audit['world_tracks']=[{'id':a.key,'position':[a.x,a.z],'state':a.tracking_state,
                'image_observed':a.image_observed,
                'prediction_age_s':a.prediction_age,'conditional_sigma_m':a.position_uncertainty,
                'sources':a.sources} for a in scene.actors]

        if manifest.get('road_map'):
            from core.scene_map import apply_map_roads
            apply_map_roads(self.scenes,manifest['road_map'],self.audit)
        from core.scene_junction import controlled_turn_intervals,apply_controlled_turn_surfaces
        turn_junctions=controlled_turn_intervals(self.scenes,poses,rows,self.audit)
        from core.scene_road import add_travel_corridors
        add_travel_corridors(self.scenes,poses,self.audit)
        self.ego_poses=poses
        from core.scene_ego_turn import apply_ego_turns
        apply_ego_turns(self.scenes,poses,self.audit)
        apply_controlled_turn_surfaces(self.scenes,poses,self.audit,turn_junctions)
        from core.scene_junction import apply_camera_junctions
        apply_camera_junctions(self.scenes,self.camera_frames,self.audit,poses)
        from core.scene_map import confirm_map_junctions
        confirm_map_junctions(self.scenes,self.audit)
        from core.scene_road_membership import exclude_unrelated_vehicles
        exclude_unrelated_vehicles(self.scenes,self.audit)
        from core.scene_lane_prior import apply_lane_containment_prior
        self.lane_prior_corrections=(apply_lane_containment_prior(self.scenes,self.audit,rows,self.camera_frames)
                                     if postprocess else 0)
        if manifest.get('motion_bundle_refinement'):
            from core.scene_bundle_refinement import reject_unestablished_outliers
            self.motion_abstentions=reject_unestablished_outliers(self.scenes,rows,poses,self.audit)

    def sample(self,t):
        scene=super().sample(t)
        if not self.scenes: return scene
        from bisect import bisect_right
        index=max(0,bisect_right([s.time for s in self.scenes],t)-1)
        base=self.scenes[index]
        if index+1<len(self.scenes) and base.time<t<self.scenes[index+1].time and self.scenes[index+1].time-base.time>self.max_gap:
            # The parent intentionally cleared a long observation gap. Do not
            # restore stale road geometry/signals from the previous frame.
            return scene
        # Keep camera coverage and lane provenance while interpolating object positions.
        scene.lanes=base.lanes
        scene.coverage=base.coverage
        scene.fusion_count=base.fusion_count
        scene.signals=getattr(base,'signals',[])
        scene.review_assisted=getattr(base,'review_assisted',False)
        scene.road_surface=getattr(base,'road_surface',[])
        scene.observed_lane_segments=getattr(base,'observed_lane_segments',[])
        scene.road_surfaces=getattr(base,'road_surfaces',[])
        scene.road_view=getattr(base,'road_view',None)
        scene.ego_turn_path_estimated=getattr(base,'ego_turn_path_estimated',False)
        scene.junction_surface_estimated=getattr(base,'junction_surface_estimated',False)
        scene.ego_position_state=getattr(base,'ego_position_state','estimated')
        if hasattr(base,'ego_display_override'):
            scene.ego_display_override=base.ego_display_override
            if index+1<len(self.scenes):
                following=self.scenes[index+1]
                dt=following.time-base.time
                if 0<dt<=self.max_gap and hasattr(following,'ego_display_override'):
                    q=float(np.clip((t-base.time)/dt,0,1))
                    a=np.array(base.ego_display_override);b=np.array(following.ego_display_override)
                    b[2]=a[2]+(b[2]-a[2]+math.pi)%(2*math.pi)-math.pi
                    scene.ego_display_override=tuple(a+(b-a)*q)
                    if base.road_view and following.road_view:
                        A=np.asarray(base.road_view['rotation']);B=np.asarray(following.road_view['rotation'])
                        first=math.atan2(A[1,0],A[0,0]);second=math.atan2(B[1,0],B[0,0])
                        angle=first+((second-first+math.pi)%(2*math.pi)-math.pi)*q
                        c,s=math.cos(angle),math.sin(angle)
                        offset=np.asarray(base.road_view['offset'])*(1-q)+np.asarray(following.road_view['offset'])*q
                        scene.road_view=dict(rotation=[[c,-s],[s,c]],offset=offset.tolist())
                    if scene.ego_turn_path_estimated or getattr(following,'ego_turn_path_estimated',False) or scene.junction_surface_estimated:
                        if len(base.lanes)==len(following.lanes) and all(
                            np.asarray(a['points']).shape==np.asarray(b['points']).shape for a,b in zip(base.lanes,following.lanes)):
                            scene.lanes=[dict(a,points=(np.asarray(a['points'])*(1-q)+np.asarray(b['points'])*q).tolist())
                                         for a,b in zip(base.lanes,following.lanes)]
                        if np.asarray(base.road_surface).shape==np.asarray(following.road_surface).shape:
                            scene.road_surface=(np.asarray(base.road_surface)*(1-q)+np.asarray(following.road_surface)*q).tolist()
                        if len(base.road_surfaces)==len(following.road_surfaces) and all(
                            np.asarray(a).shape==np.asarray(b).shape for a,b in zip(base.road_surfaces,following.road_surfaces)):
                            scene.road_surfaces=[(np.asarray(a)*(1-q)+np.asarray(b)*q).tolist()
                                                 for a,b in zip(base.road_surfaces,following.road_surfaces)]
        scene.road_status=getattr(base,'road_status',{})
        scene.road_prior=getattr(base,'road_prior',None)
        scene.road_context=getattr(base,'road_context',None)
        return scene
