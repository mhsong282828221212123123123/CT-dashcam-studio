"""Camera gap + own turn + optional cached topology; never GPS lane snapping."""
from collections import defaultdict
import cv2,numpy as np
from .road_frame import coordinates,world
from .road_geometry import supported


def infer(times,ego,yaws,road,map_records=(),crossing_times=()):
    if not road.get('reference_line'):return []
    t=np.asarray(times);p=np.asarray(ego);yaw=np.unwrap(yaws)
    rate=abs(np.gradient(yaw,t));moving=np.linalg.norm(np.gradient(p,t,axis=0),axis=1)>.5
    active=np.flatnonzero((rate>np.radians(5))&moving);groups=[]
    for i in active:
        if not groups or t[i]-t[groups[-1][-1]]>1:groups.append([i])
        else:groups[-1].append(i)
    local=coordinates(road,p);result=[]
    for ids in groups:
        i,j=ids[0],ids[-1]
        if not 1<t[j]-t[i]<15 or abs(yaw[j]-yaw[i])<np.radians(30) or np.linalg.norm(p[j]-p[i])<6:continue
        gap=sum(not supported(road,s) for s in local[i:j+1,0])/len(local[i:j+1])
        if gap<.25:continue
        maps=[r for r in map_records if t[i]-4<=r['time_s']<=t[j]+2 and any(c['distance_m']<45 for c in r['junction_candidates'])]
        cross=sorted({x for x in crossing_times if t[i]-8<=x<=t[j]})
        if cross and cross[-1]-cross[0]<.4:cross=[]
        if len(maps)<3 and len(cross)<3:continue
        start=max(0,i-int(round(1.5/np.median(np.diff(t)))));end=min(len(t)-1,j+int(round(1./np.median(np.diff(t)))))
        a,b=local[start,0],local[end,0];bounds=road['boundaries_y'];corners=[]
        for k in (start,end):
            tangent=np.array([np.cos(yaw[k]),np.sin(yaw[k])]);normal=np.array([-tangent[1],tangent[0]])
            for along in (-8,8):
                for d in (bounds[-1]-1.,bounds[0]+1.):corners.append(p[k,:2]+along*tangent+d*normal)
        for station in np.arange(min(a,b)-8,max(a,b)+8,2.):
            for d in (bounds[-1]-1.,bounds[0]+1.):corners.append(world(road,station,d))
        polygon=cv2.convexHull(np.asarray(corners,'float32')).reshape(-1,2)
        result.append(dict(id='junction-'+str(len(result)),start_s=float(t[start]),end_s=float(t[end]),
                           road_station_range_m=[float(min(a,b)-8),float(max(a,b)+8)],polygon_W_xy=polygon.tolist(),
                           turn_deg=float(np.degrees(yaw[j]-yaw[i])),camera_gap_fraction=gap,map_support_records=len(maps),crossing_support_frames=len(cross),
                           support='own_turn_and_camera_marking_gap_plus_'+('osm_topology' if len(maps)>=3 else 'camera_crossing'),
                           surface='estimated_local_opening; not map_registered_or_surveyed',branch_reconstruction='NOT_EVALUATED'))
    return result


def inside(road,point,margin=0.):
    p=tuple(float(v) for v in point[:2])
    return any(cv2.pointPolygonTest(np.array(j['polygon_W_xy'],'float32'),p,True)>=-margin for j in road.get('junctions',[]))


def straight_openings(times,ego,road,segments,map_records):
    """Camera-anchored transverse paint plus repeated nearby map topology.

    The map only accepts a camera candidate within a broad GPS tolerance. It
    does not position the opening, choose a lane, or register the ego pose.
    A stop line alone or an OSM junction alone cannot create a cross road.
    """
    if not road.get('reference_line') or not map_records:return []
    t=np.asarray(times);local=coordinates(road,ego);bins=defaultdict(list)
    for row in segments:
        if row['kind']!='crossbar':continue
        p=coordinates(road,row['points_W_xy']);delta=p[1]-p[0]
        if abs(delta[1])<3 or abs(delta[0])>abs(delta[1])*.4:continue
        s,d=np.mean(p[:,:2],axis=0)
        bounds=road['boundaries_y'];centre=(bounds[0]+bounds[-1])/2
        # A queue may hide our half of a shared crossing. Road-masked paint
        # on the adjacent carriageway can confirm the same mapped junction;
        # this never admits adjacent-carriageway vehicles into our lanes.
        if abs(d-centre)>(bounds[0]-bounds[-1])+2*road.get('lane_width_m_prior',3.5):continue
        role='own_corridor' if bounds[-1]-2<d<bounds[0]+2 else 'adjacent_surface_junction_evidence_only'
        bins[round(s/5)*5].append((float(s),dict(row,junction_paint_role=role)))
    peaks=[]
    for _,rows in sorted(bins.items()):
        frames={r['frame_id'] for _,r in rows};rt=[r['time_s'] for _,r in rows]
        if len(frames)<3 or max(rt)-min(rt)<.4:continue
        s=float(np.median([s for s,_ in rows]));nodes=defaultdict(set)
        for record in map_records:
            now=record['time_s']
            if not min(rt)-2<=now<=max(rt)+2:continue
            es=float(np.interp(now,t,local[:,0]))
            for j in record['junction_candidates']:
                if j['distance_m']<65 and abs(es+j['forward_m']-s)<=25:
                    nodes[j['node_id']].add(now)
        supported_nodes=[n for n,stamps in nodes.items() if len(stamps)>=3]
        if supported_nodes:peaks.append(dict(s=s,frames=len(frames),nodes=supported_nodes,times=[min(rt),max(rt)],
                                             camera_paint_roles=sorted({r['junction_paint_role'] for _,r in rows})))
    groups=[]
    for p in peaks:
        common=set.intersection(*(set(row['nodes']) for row in groups[-1])) if groups else set()
        if not groups or p['s']-groups[-1][-1]['s']>35 or not common.intersection(p['nodes']):groups.append([p])
        else:groups[-1].append(p)
    result=[]
    for group in groups:
        a,b=group[0]['s']-2,group[-1]['s']+12
        if any(not (b<j['road_station_range_m'][0] or a>j['road_station_range_m'][1]) for j in road.get('junctions',[])):continue
        bounds=road['boundaries_y'];left=max(bounds[0]+8,12);right=min(bounds[-1]-8,-12)
        corners=[world(road,s,d) for s in np.arange(a,b+2,2) for d in [left,right]]
        polygon=cv2.convexHull(np.asarray(corners,'float32')).reshape(-1,2)
        result.append(dict(id='camera-map-opening-'+str(len(result)),road_station_range_m=[a,b],polygon_W_xy=polygon.tolist(),
                           support='repeated_camera_transverse_paint_with_nearby_osm_topology',
                           camera_station_peaks=group,map_role='loose_topology_confirmation_only; no GPS lane assignment',
                           surface='estimated_cross_road_opening; width_and_branches_unresolved',branch_reconstruction='NOT_EVALUATED'))
    return result


def crossing_motion(actor,road):
    rows=[r for r in actor['samples'] if inside(road,r['position_W'],3.) and r.get('support')=='detector_observation'
          and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>70]
    if len(rows)<3 or rows[-1]['time_s']-rows[0]['time_s']<.4:return False
    local=coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in rows])
    edge=min(3,max(1,len(rows)//3))
    delta=np.median(local[-edge:,:2],axis=0)-np.median(local[:edge,:2],axis=0)
    return bool(abs(delta[1])>4 and abs(delta[1])>abs(delta[0])*.6)
