"""Bounded world-coordinate tracks; prediction is never a new observation.

SEI speed/heading are preview odometry, not surveyed poses. Covariance describes
filter uncertainty conditional on the camera model, not absolute metric accuracy.
"""
from dataclasses import replace
import math
import numpy as np
from core.scene_geometry import footprints_overlap
from core.scene_telemetry import odometry_speed_kmh


def rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


def bridge_motion_consistent(before,after,gap):
    """Reject abrupt endpoint changes; this is not collision detection."""
    if before is None or after is None:return True
    before,after=np.asarray(before),np.asarray(after)
    if not np.isfinite(np.r_[before,after]).all():return False
    if np.linalg.norm(after-before)>3.+6.*gap:return False
    a,b=np.linalg.norm(before),np.linalg.norm(after)
    return not (min(a,b)>3 and np.dot(before,after)/(a*b)<.5)


class WorldTracks:
    def __init__(self):
        self.tracks={}; self.pose=np.zeros(2); self.angle=0.; self.last=None
        self.serial=0

    @staticmethod
    def _supports(track,support):
        return [(source,support[source]) for source in track['sources']
                if source in support and not source.startswith(('back:','review:back:'))]

    def _save_image_anchors(self,track,actor,support,t):
        anchors=track.setdefault('image_anchors',{})
        for source in actor.sources:
            if source not in support or source.startswith(('back:','review:back:')):continue
            if not source.startswith(('front:','review:front:')) or math.hypot(actor.x,actor.z)>45:continue
            evidence=support[source];camera=getattr(actor,'source_geometries',{}).get(source,evidence['camera']);x,y,w,h=evidence['bbox']
            if x<3 or x+w>camera.width-3 or y<3 or y+h>camera.height-3:continue
            point=np.array([actor.x,0.,actor.z]);local=(point-camera.origin)@camera.rotation
            pixels=camera.pixels(point[None,:])
            if pixels is None or local[2]<1:continue
            anchors[source]=dict(time=t,bbox=list(evidence['bbox']),depth=float(local[2]),
                centre_offset=float((pixels[0,0]-(x+w/2))/w))

    def _image_position(self,track,support,t,R,dt):
        """Depth/azimuth continuation from a real metric anchor, never from a lane.

        Vertical box motion is deliberately not used as ground-plane evidence.
        A grade/pitch mismatch can invalidate ground fitting while size and
        horizontal bearing still support a bounded, explicitly ghost pose.
        """
        choices=[]
        for source,evidence in self._supports(track,support):
            anchor=track.get('image_anchors',{}).get(source)
            if anchor is None or not 0<t-anchor['time']<=6.:continue
            camera=evidence['camera'];x,y,w,h=map(float,evidence['bbox'])
            if x<3 or x+w>camera.width-3 or y<3 or y+h>camera.height-3 or min(w,h)<8:continue
            old=anchor['bbox'];ratios=np.array([old[2]/w,old[3]/h])
            # A narrow occlusion sliver must not become a rapidly receding car.
            if min(ratios)<.4 or max(ratios)>2.5 or max(ratios)/min(ratios)>1.35:continue
            depth=anchor['depth']*float(np.sqrt(np.prod(ratios)))
            cx=camera.width/2 if camera.principal_x is None else camera.principal_x
            u=x+w/2+anchor['centre_offset']*w
            local_x=-(u-cx)/camera.focal*depth
            C=camera.rotation
            local_y=-(camera.origin[1]+C[1,0]*local_x+C[1,2]*depth)/C[1,1]
            point=(camera.origin+C@np.array([local_x,local_y,depth]))[[0,2]]
            world=self.pose+R@point
            if not np.isfinite(point).all() or np.linalg.norm(point)>65:continue
            if np.linalg.norm(world-track['previous_position'])>max(1.,55.*dt):continue
            # A changing visible fragment can shrink a box in one frame. Its
            # size-derived depth must agree with the established motion model;
            # the old 55 m/s bound alone allowed five-metre sideways jumps.
            sigma=float(np.sqrt(np.linalg.eigvalsh(track['cov'][:2,:2]).max()))
            innovation_limit=min(2.5,max(1.,3.*sigma))
            if np.linalg.norm(world-track['state'][:2])>innovation_limit:continue
            choices.append((t-anchor['time'],source,point,world))
        return min(choices,key=lambda v:v[0]) if choices else None

    def update(self,actors,t,telemetry,image_support=None):
        image_support=image_support or {}
        data=telemetry or {}; speed=odometry_speed_kmh(data); heading=data.get('heading')
        valid=speed is not None and math.isfinite(speed) and 0<=speed<250
        heading_valid=heading is not None and math.isfinite(heading)
        dt=0. if self.last is None else t-self.last[0]
        pose_valid=valid and (speed==0 or heading_valid)
        if self.last is not None:
            pt,ps,ph,pvalid=self.last
            pose_valid=pose_valid and pvalid and 0<dt<=.5
            if pose_valid:
                delta=0. if not heading_valid or ph is None or speed+ps<.5 else math.radians((heading-ph+180)%360-180)
                self.pose+=rotation(self.angle+delta/2)@np.array([0.,(speed+ps)/7.2*dt])
                self.angle+=delta
        self.last=(t,speed,heading if heading_valid else None,valid and (speed==0 or heading_valid))
        events=[]
        if not pose_valid:
            if self.tracks: events.append({'reason':'odometry unavailable or discontinuous; tracks reset'})
            self.tracks={}
        R=rotation(self.angle)
        for key,tr in list(self.tracks.items()):
            if any(source in tr.get('image_anchors',{}) for source,_ in self._supports(tr,image_support)):
                tr['image_seen']=t
            age=t-max(tr['seen'],tr.get('image_seen',tr['seen']))
            ttl=1.5 if valid and speed==0 else .8
            local=R.T@(tr['state'][:2]-self.pose)
            side_memory=tr['hits']>=5 and any('repeater:' in s for s in tr['sources']) and 1.5<abs(local[0])<9 and abs(local[1])<12
            if side_memory:ttl=2.
            tr['horizon']=ttl
            if age>ttl+1e-8:
                events.append({'id':key,'reason':'prediction expired','age_s':age}); del self.tracks[key]; continue
            F=np.eye(4);F[:2,2:]=np.eye(2)*dt
            G=np.vstack([np.eye(2)*dt*dt/2,np.eye(2)*dt])
            tr['previous_position']=tr['state'][:2].copy()
            # A turning car needs lateral acceleration in its motion model.
            # Otherwise a valid curved path eventually fails the straight-line
            # association gate and drops an otherwise continuous image ID.
            acceleration=max(1.5,min(8.,np.linalg.norm(tr['state'][2:])*abs(tr.get('turn_rate',0.))))
            tr['process_accel_variance']=acceleration**2
            tr['state']=F@tr['state'];tr['cov']=F@tr['cov']@F.T+G@G.T*tr['process_accel_variance']
        used=set(); output=[]; measurements={}
        for actor in sorted(actors,key=lambda a:(getattr(a,'placement_error',0),a.key)):
            identity_sources={s for s in actor.sources if not s.startswith(('back:','review:back:'))}
            measurement=self.pose+R@np.array([actor.x,actor.z])
            sigma=.25+.015*math.hypot(actor.x,actor.z)+2*getattr(actor,'placement_error',.1)
            noise=np.eye(2)*sigma*sigma
            options=[]
            known_identity=any(identity_sources&tr['sources'] for tr in self.tracks.values())
            source_cameras={s.rsplit(':',1)[0] for s in identity_sources}
            for key,tr in self.tracks.items():
                if key in used: continue
                vehicle=lambda k:k in ('car','truck','suv','small_truck','bus')
                if actor.kind!=tr['actor'].kind and not(vehicle(actor.kind) and vehicle(tr['actor'].kind)): continue
                residual=measurement-tr['state'][:2]
                shared=bool(identity_sources&tr['sources'])
                # A noisy observation of a known local track must not steal a
                # nearby stranger when its own association gate fails. That
                # creates enormous false velocity and a flying ghost later.
                if known_identity and not shared:continue
                if not shared and t-tr.get('position_seen',tr['seen'])>tr.get('horizon',.8):continue
                # Distinct live local IDs from one camera are separate objects.
                # A genuine cross-camera handoff still has disjoint cameras.
                if not shared and source_cameras&{s.rsplit(':',1)[0] for s in tr['sources']}:continue
                # Different tracks seen simultaneously by one camera cannot be merged.
                if not shared and t-tr['seen']<.05: continue
                distance=float(np.linalg.norm(residual))
                score=float(residual@np.linalg.solve(tr['cov'][:2,:2]+noise,residual))
                gate=3.+35.*max(0.,t-tr.get('position_seen',tr['seen'])) if shared else 1.4
                if score<9.21 and distance<gate:
                    options.append((0 if shared else 1,score,key))
            options.sort()
            unique=options and (len(options)==1 or options[0][0]<options[1][0] or options[1][1]-options[0][1]>2.)
            if unique:
                key=options[0][2];tr=self.tracks[key]
                residual=measurement-tr['state'][:2]
                gain=np.linalg.solve(tr['cov'][:2,:2]+noise,tr['cov'][:2,:]).T
                tr['state']+=gain@residual
                H=np.zeros((2,4));H[:,:2]=np.eye(2)
                A=np.eye(4)-gain@H
                tr['cov']=A@tr['cov']@A.T+gain@noise@gain.T
                axis=R@np.array([math.sin(actor.yaw),math.cos(actor.yaw)])
                yaw=math.atan2(axis[0],axis[1]);old_yaw=tr.get('world_body_yaw',yaw)
                measured_rate=(yaw-old_yaw+math.pi)%(2*math.pi)-math.pi
                measured_rate/=max(.01,t-tr['seen'])
                tr['turn_rate']=.5*tr.get('turn_rate',0.)+.5*float(np.clip(measured_rate,-1.5,1.5))
                tr['world_body_yaw']=yaw
                tr['hits']+=1;tr['seen']=t;tr['sources'].update(identity_sources);tr['actor']=actor
            else:
                # A local ID that jumps outside its gate must not spawn a duplicate.
                conflict=next((key for key,tr in self.tracks.items() if identity_sources&tr['sources']),None)
                if conflict:
                    events.append({'id':conflict,'reason':'observation outside association gate','sources':actor.sources});continue
                key=actor.key
                if key in self.tracks:
                    self.serial+=1;key=f'{key}:generation{self.serial}'
                tr={'state':np.r_[measurement,0.,0.],'cov':np.diag([sigma*sigma]*2+[225.,225.]),
                    'seen':t,'hits':1,'sources':identity_sources,'actor':actor}
                axis=R@np.array([math.sin(actor.yaw),math.cos(actor.yaw)])
                tr['world_body_yaw']=math.atan2(axis[0],axis[1])
                self.tracks[key]=tr
            used.add(key)
            tr['position_seen']=t
            tr['position_kind']='metric'
            self._save_image_anchors(tr,actor,image_support,t)
            measurements[key]=actor
            local=R.T@(tr['state'][:2]-self.pose)
            output.append(replace(actor,key=key,x=float(local[0]),z=float(local[1]),
                tracking_state='pitch_held' if actor.tracking_state=='pitch_held' else 'observed',
                prediction_age=0.,position_uncertainty=float(np.sqrt(np.linalg.eigvalsh(tr['cov'][:2,:2]).max()))))
        overlaps=footprints_overlap
        # Filtering can move two independently supported, disjoint measurements
        # into each other. In that case retain this frame's measured positions;
        # never push vehicles onto invented lane centers to hide the conflict.
        if any(overlaps(a,b) and not overlaps(measurements[a.key],measurements[b.key])
               for i,a in enumerate(output) for b in output[i+1:]):
            for i,a in enumerate(output):
                measured=measurements[a.key]
                output[i]=replace(a,x=measured.x,z=measured.z)
                self.tracks[a.key]['state'][:2]=self.pose+R@np.array([measured.x,measured.z])
            events.append({'reason':'filter overlap; retained current camera measurements'})
        for key,tr in self.tracks.items():
            if key in used or tr['hits']<3 or not pose_valid: continue
            supported=self._image_position(tr,image_support,t,R,dt)
            if supported is not None:
                age,source,point,world=supported;actor=tr['actor']
                if any(footprints_overlap(actor,a,position_a=point) for a in output):
                    events.append({'id':key,'reason':'image-supported estimate conflicts with current observation'});continue
                tr['state'][:2]=world;tr['position_seen']=t;tr['position_kind']='image_supported'
                tr['image_uncertainty']=.65+.3*age
                output.append(replace(actor,key=key,x=float(point[0]),z=float(point[1]),sources=(source,),
                    tracking_state='image_supported',prediction_age=age,prediction_horizon=6.,
                    position_uncertainty=.65+.3*age))
                events.append({'id':key,'reason':'2D identity retained; metric position estimated',
                    'sources':[source],'metric_anchor_age_s':age})
                continue
            # Image presence leases identity, not an unlimited blind 3D pose.
            position_age=t-tr.get('position_seen',tr['seen'])
            if position_age>tr.get('horizon',.8)+1e-8:continue
            local=R.T@(tr['state'][:2]-self.pose)
            uncertainty=float(np.sqrt(np.linalg.eigvalsh(tr['cov'][:2,:2]).max()))
            if tr.get('position_kind')=='image_supported':
                if np.linalg.norm(tr['state'][2:])>55:continue
                uncertainty=tr['image_uncertainty']+.75*position_age
            if uncertainty>(3.5 if tr.get('horizon',.8)>1.5 else 2.5) or np.linalg.norm(local)>65: continue
            actor=tr['actor']
            actor=tr['actor']
            if any(footprints_overlap(actor,a,position_a=local) for a in output):
                events.append({'id':key,'reason':'prediction conflicts with current observation'});continue
            output.append(replace(actor,key=key,x=float(local[0]),z=float(local[1]),sources=(),
                tracking_state='predicted',prediction_age=position_age,position_uncertainty=uncertainty,
                prediction_horizon=tr.get('horizon',1.5)))
        return output,events
