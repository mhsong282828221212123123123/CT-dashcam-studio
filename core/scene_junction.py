"""Camera-only junction evidence. Crosswalks alone do not prove a junction."""
from collections import deque
import cv2
import numpy as np


def junction_evidence(frame,objects=()):
    h,w=frame.shape[:2];im=cv2.resize(frame,(640,480))
    hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    visible=np.ones((480,640),np.uint8)
    for obj in objects:
        x,y,bw,bh=obj['bbox']
        cv2.rectangle(visible,(int(x*640/w)-3,int(y*480/h)-3),
                      (int((x+bw)*640/w)+3,int((y+bh)*480/h)+3),0,-1)
    white=((hsv[:,:,1]<75)&(gray>140)&(visible!=0)).astype(np.uint8)
    white[:270]=0;white[430:]=0
    count,labels,stats,centres=cv2.connectedComponentsWithStats(white,8)
    bars=[]
    for j in range(1,count):
        x,y,bw,bh,area=stats[j]
        if (area>=65 and bw>=15 and 18<=bh<=90 and y>272 and y+bh<428
                and bw>bh*.65 and area/(bw*bh)>.3):
            bars.append((x,y,bw,bh,area))
    # Repeated paint islands distributed across the road at similar image depth.
    groups=[]
    for bar in bars:
        peers=[v for v in bars if abs((v[1]+v[3]/2)-(bar[1]+bar[3]/2))<20]
        if len(peers)>=3 and max(v[0]+v[2] for v in peers)-min(v[0] for v in peers)>300:
            groups.append(peers)
    if not groups:return dict(candidate=False,reason='no_repeated_transverse_markings',sections=[])
    group=max(groups,key=lambda g:sum(v[4] for v in g))
    # Road appearance in the lateral opening must resemble the near road sample.
    road_sample=gray[385:430,280:360];level=float(np.median(road_sample))
    road=(abs(gray.astype(float)-level)<40)&(hsv[:,:,1]<90)&(gray>35)&(visible!=0)
    sides=[];sections=[]
    # The crossing road is beyond the near crosswalk, not underneath the host.
    # Search its approach-side image band instead of projecting stripe bottoms
    # as the intersection extent.
    near=max(260,min(v[1] for v in group)-80);far=min(390,min(v[1] for v in group)+10)
    for name,left,right in [('left',15,160),('right',480,625)]:
        coverage=float(road[near:far,left:right].mean()) if far>near else 0.
        if coverage>.55:sides.append(name)
    if not sides:return dict(candidate=False,reason='markings_without_lateral_road_support',sections=[])
    for y in range(near,far,8):
        pixels=np.flatnonzero(road[y])
        if len(pixels)>380 and pixels[0]<120 and pixels[-1]>520:
            sections.append([[float(pixels[0]*w/640),float(y*h/480)],
                             [float(pixels[-1]*w/640),float(y*h/480)]])
    return dict(candidate=len(sections)>=3,reason='repeated_markings_and_lateral_road_appearance',
                sides=sides,sections=sections,marking_components=len(group),
                marking_boxes=[list(map(int,v[:4])) for v in group],
                semantics='camera junction hypothesis; topology not fully resolved')


class JunctionTracker:
    def __init__(self):self.history=deque(maxlen=11);self.last=None
    def update(self,evidence,t):
        if self.last is not None and not 0<t-self.last<=.3:self.history.clear()
        self.last=t;self.history.append((t,bool(evidence.get('candidate'))))
        support=sum(v for _,v in self.history)
        return dict(evidence,active=bool(evidence.get('candidate') and t-self.history[0][0]>=.8
                                        and support>=len(self.history)*.8),
                    support_frames=support,source='camera_only_no_map')


def controlled_turn_intervals(scenes,poses,rows,audits):
    """Signal + large turn + lost longitudinal paint: a cheap junction cue.

    A road bend alone is insufficient. This does not infer cross-road branches.
    """
    if not scenes:return []
    times=np.array([s.time for s in scenes]);angles=np.unwrap([np.arctan2(R[1,0],R[0,0]) for _,R in poses])
    rate=np.zeros(len(times))
    for i,t in enumerate(times):
        ids=np.flatnonzero(abs(times-t)<=.3)
        if len(ids)>1:rate[i]=(angles[ids[-1]]-angles[ids[0]])/(times[ids[-1]]-times[ids[0]])
    ids=np.flatnonzero(abs(rate)>np.radians(5));groups=[]
    for i in ids:
        if not groups or times[i]-times[groups[-1][-1]]>.5:groups.append([])
        groups[-1].append(i)
    events=[]
    for group in groups:
        first,last=group[0],group[-1]
        if times[last]-times[first]<.8 or abs(angles[last]-angles[first])<np.radians(35):continue
        before=np.flatnonzero((times>=times[first]-5)&(times<=times[first]+.5))
        signal_frames=0;colour_support=False
        for j in before:
            signals=rows[j].get('cameras',{}).get('front',{}).get('signals',[])
            trusted=[s for s in signals if s.get('head_type')=='horizontal_candidate' and s.get('confidence',0)>=.55]
            signal_frames+=bool(trusted)
            colour_support|=any(s.get('state') in ('red','amber','green') and s.get('support',0)>=3 for s in trusted)
        map_support={}
        for j in before:
            for candidate in getattr(scenes[j],'road_context',{}).get('junction_candidates',[]):
                if candidate['distance_m']<=45 and candidate['point'][1]>=-10:
                    key=candidate['node_id'];map_support[key]=map_support.get(key,0)+1
        map_supported=any(count>=3 for count in map_support.values())
        signal_supported=signal_frames>=3 and colour_support
        if not signal_supported and not map_supported:continue
        gap=[j for j in range(max(0,first-20),last+1) if not scenes[j].lanes]
        if len(gap)<3:continue
        entry=min(gap);exit=last
        for j in range(last+1,len(times)):
            if times[j]-times[last]>3:break
            window=list(range(max(last,j-4),j+1))
            if len(window)>=4 and max(abs(rate[window]))<np.radians(3) and sum(bool(scenes[k].lanes) for k in window)>=3:
                exit=j;break
        if exit<=entry or any(times[j]-times[j-1]>.3 for j in range(entry+1,exit+1)):continue
        if np.linalg.norm(poses[exit][0]-poses[entry][0])<4:continue
        events.append((entry,exit))
        for j in range(max(0,entry-30),exit+1):
            state='inside' if j>=entry and j<exit else 'exit' if j==exit else 'approach'
            scenes[j].road_junction_state=state
            audits[j]['road_junction']=dict(state=state,
                entry_time_s=float(times[entry]),exit_time_s=float(times[exit]),
                source='osm_turn_and_paint_gap' if map_supported else 'signal_turn_and_paint_gap',
                osm_supported=map_supported,topology_verified=False)
    return events


def apply_controlled_turn_surfaces(scenes,poses,audits,events):
    for entry,exit in events:
        road=audits[exit].get('travel_corridors',{});indices=road.get('occupied_lane_indices',[0]);width=road.get('lane_width_assumed_m',3.5)
        low=(min(indices)-.5)*width;high=(max(indices)+.5)*width
        cloud=[]
        for j in range(entry,exit+1):
            o,R=poses[j]
            cloud.extend([o+R@np.array([x,0.]) for x in (low-1.,high+1.)])
        # Fill the corner between approach and exit, rather than drawing the
        # inferred turning lane as a fully marked curved motorway.
        o0,R0=poses[entry];o1,R1=poses[exit];centre=(low+high)/2
        p0=o0+R0@[centre,0.];p1=o1+R1@[centre,0.]
        matrix=np.column_stack([R0[:,1],-R1[:,1]])
        if abs(np.linalg.det(matrix))>.2:
            distances=np.linalg.solve(matrix,p1-p0)
            if max(abs(distances))<80:
                corner=p0+R0[:,1]*distances[0];half=(high-low)/2+1.5
                cloud.extend([corner+R0@np.array([x,z]) for x in (-half,half) for z in (-half,half)])
        hull=cv2.convexHull(np.asarray(cloud,np.float32)).reshape(-1,2)
        for scene,(origin,R),audit in zip(scenes,poses,audits):
            local=(hull-origin)@R
            if local[:,1].max()<-15 or local[:,1].min()>65 or np.min(np.linalg.norm(local,axis=1))>90:continue
            scene.road_surfaces=getattr(scene,'road_surfaces',[])+[local.tolist()]
            scene.junction_surface_estimated=True
            lanes=[]
            for lane in scene.lanes:
                run=[]
                for point in lane['points']:
                    world=origin+R@np.asarray(point)
                    if cv2.pointPolygonTest(hull,(float(world[0]),float(world[1])),False)>=0:
                        if len(run)>1:lanes.append(dict(lane,points=run))
                        run=[]
                    else:run.append(point)
                if len(run)>1:lanes.append(dict(lane,points=run))
            scene.lanes=lanes
            audit.setdefault('road_junction',{}).update(surface_visible=True,parallel_markings_interrupted=True)


def apply_camera_junctions(scenes,cameras,audits,poses=None):
    for i,(scene,geometry,audit) in enumerate(zip(scenes,cameras,audits)):
        evidence=getattr(scene,'junction_evidence',{})
        if not evidence.get('active') or 'front' not in geometry:continue
        quads=[];zs=[]
        sections=[]
        for section in evidence.get('sections',[]):
            points=[geometry['front'].ground(*p) for p in section]
            if all(p is not None for p in points):sections.append(np.array(points))
        for a,b in zip(sections,sections[1:]):
            if np.max(np.linalg.norm(a-b,axis=1))>8:continue
            quads.append([a[0].tolist(),a[1].tolist(),b[1].tolist(),b[0].tolist()]);zs.extend([*a[:,1],*b[:,1]])
        if not quads:continue
        crossing=[]
        # Crossing traffic can support a second road axis. Compensate host motion
        # before measuring vehicle motion; never derive host pose from vehicles.
        if poses is not None:
            origin,R=poses[i]
            for j in range(i-1,-1,-1):
                dt=scene.time-scenes[j].time
                if dt>1.:break
                if dt<.5:continue
                if any('tracks reset' in e.get('reason','') for a in audits[j+1:i+1] for e in a.get('world_track_events',[])):break
                previous={a.key:a for a in scenes[j].actors if a.tracking_state=='observed'}
                o,Q=poses[j]
                for actor in scene.actors:
                    if actor.key not in previous or actor.tracking_state!='observed':continue
                    old=previous[actor.key]
                    velocity=R.T@(origin+R@np.array([actor.x,actor.z])-o-Q@np.array([old.x,old.z]))/dt
                    if abs(velocity[0])>1.5 and abs(velocity[0])>1.5*abs(velocity[1]):
                        crossing.append([actor.x,actor.z])
                break
        if len(crossing)>=2:
            points=np.array(crossing);centre=float(np.median(points[:,1]))
            if np.ptp(points[:,1])<8:
                half=3.5 # Display width assumption, not a measured cross-road width.
                left=min(float(points[:,0].min())-2.,-half)
                right=max(float(points[:,0].max())+2.,half)
                quads.append([[left,centre-half],[right,centre-half],[right,centre+half],[left,centre+half]])
                zs.extend([centre-half,centre+half])
        scene.road_surfaces=getattr(scene,'road_surfaces',[])+quads
        lower,upper=min(zs),max(zs);lanes=[]
        for lane in scene.lanes:
            run=[]
            for point in lane['points']:
                if lower<=point[1]<=upper:
                    if len(run)>1:lanes.append(dict(lane,points=run))
                    run=[]
                else:run.append(point)
            if len(run)>1:lanes.append(dict(lane,points=run))
        scene.lanes=lanes
        audit['camera_junction']=dict(active=True,source='camera_only_no_map',
             sides=evidence.get('sides',[]),longitudinal_band_m=[lower,upper],
             transverse_vehicle_support=len(crossing),
             metric_accuracy='unverified camera ground projection')
