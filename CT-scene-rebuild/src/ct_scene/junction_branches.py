"""Map orientation hints on CAMERA-anchored junctions, never map registration.

These short display surfaces do not change ego/actor geometry, lane scopes,
lane counts or the camera-supported junction opening polygon.
"""
from collections import defaultdict
import copy,math
import numpy as np
from .road_frame import world,coordinates
from .road_geometry import boundaries_at
from .junction_display import display_bounds,is_turning_junction,turn_apex


def angle_delta(a,b):return float(np.arctan2(np.sin(a-b),np.cos(a-b)))


def display_station(junction):
    stations=[p['s'] for p in junction.get('camera_station_peaks',[]) if 's' in p]
    return float(np.median(stations)) if stations else sum(junction['road_station_range_m'])/2


def hints(road,map_records,times,ego_yaws,length_m=25.,observed_end_station=None):
    result=copy.deepcopy(road.get('junctions',[]));review=[]
    for junction in result:
        a,b=junction['road_station_range_m'];nodes={n for p in junction.get('camera_station_peaks',[]) for n in p['nodes']}
        start=junction.get('start_s',min((p['times'][0] for p in junction.get('camera_station_peaks',[])),default=-float('inf')))
        end=junction.get('end_s',max((p['times'][1] for p in junction.get('camera_station_peaks',[])),default=float('inf')))
        observations=defaultdict(list);levels=set();node_times=defaultdict(set);node_complete={}
        for record in map_records:
            if not start-3<=record['time_s']<=end+3:continue
            yaw=float(np.interp(record['time_s'],times,np.unwrap(ego_yaws)))
            for candidate in record['junction_candidates']:
                if nodes and candidate['node_id'] not in nodes:continue
                if candidate['distance_m']>65 or (not nodes and candidate['distance_m']>45):continue
                branches=candidate.get('branches',[])
                if not branches:continue
                node_complete[candidate['node_id']]=node_complete.get(candidate['node_id'],True) and len(branches)==candidate.get('degree')
                levels.add(tuple(candidate['level']));node_times[candidate['node_id']].add(record['time_s'])
                for branch in branches:
                    if 'relative_bearing_deg' not in branch:continue
                    observations[candidate['node_id'],branch['neighbor_node_id']].append((record['time_s'],yaw-math.radians(branch['relative_bearing_deg'])))
        usable={n for n,t in node_times.items() if len(t)>=3}
        if len(levels)!=1 or not usable:
            junction['branch_display_hints']=[];review.append(dict(junction_id=junction['id'],status='ambiguous_grade_or_insufficient_repeated_topology'));continue
        # Connected staggered/divided-carriageway 2-node intersections split left/right cross-arms across the two adjacent nodes.
        staggered_pair=len(usable)==2 and any((a,b) in observations or (b,a) in observations for a in usable for b in usable if a!=b)
        min_support=1 if staggered_pair else math.ceil(len(usable)*.6)
        arms=[]
        for (node,neighbor),rows in observations.items():
            if node not in usable or len({t for t,y in rows})<3:continue
            angle=float(np.angle(np.mean(np.exp(1j*np.array([v for t,v in rows])))))
            if max(abs(angle_delta(v,angle)) for t,v in rows)>math.radians(15):continue
            arms.append(dict(node=node,neighbor=neighbor,angle=angle,records=len({t for t,v in rows})))
        clusters=[]
        for arm in sorted(arms,key=lambda p:-p['records']):
            group=next((g for g in clusters if abs(angle_delta(arm['angle'],g[0]['angle']))<math.radians(20)),None)
            if group is None:clusters.append([arm])
            else:group.append(arm)
        main_start=float(coordinates(road,world(road,a,0))[2])+math.pi
        main_end=float(coordinates(road,world(road,b,0))[2])
        if is_turning_junction(junction):
            centre=turn_apex(road,junction)
            junction['turn_apex_W_xy']=centre.tolist()
        else:
            display_s=display_station(junction);display_b=display_bounds(road,display_s)
            centre=np.asarray(world(road,display_s,(display_b[0]+display_b[-1])/2),float)
        bounds=boundaries_at(road,a-.1);width=float(np.clip((bounds[0]-bounds[-1])*.5,3.5,10.));accepted=[];forward_supported=False
        for group in clusters:
            support={p['node'] for p in group}
            if len(support)<min_support:continue
            angle=float(np.angle(np.mean(np.exp(1j*np.array([p['angle'] for p in group])))))
            if abs(angle_delta(angle,main_end))<math.radians(20):forward_supported=True
            if min(abs(angle_delta(angle,main_start)),abs(angle_delta(angle,main_end)))<math.radians(30):continue
            direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
            polygon=[centre+direction*s+normal*d for s,d in [(0,-width/2),(length_m,-width/2),(length_m,width/2),(0,width/2)]]
            accepted.append(dict(polygon_W_xy=np.asarray(polygon).tolist(),heading_W_rad=angle,
                                 origin_W_xy=centre.tolist(),length_m=length_m,width_m=width,
                                 support_node_ids=sorted(support),source='osm_orientation_on_camera_opening; display_hint_only',
                                 width_support='visual_band_from_camera_corridor; no_map_lanes_or_width',
                                 extent_support='local_display_stub; actual_branch_extent_unknown'))
        junction['branch_display_hints']=accepted;junction['branch_reconstruction']='CAMERA_ANCHORED_TOPOLOGY_DISPLAY_HINTS; branch_lanes_unresolved'
        if observed_end_station is not None and a-80<=observed_end_station<a and len(accepted)>=2 and not forward_supported and all(node_complete.get(n,False) for n in usable):
            junction['terminal_display_station_m']=display_station(junction)
            junction['terminal_display_support']='complete_repeated_map_T_topology_ahead_of_observed_motion; visual_only'
        review.append(dict(junction_id=junction['id'],status='camera_anchored_branch_orientation_hints',arms=len(accepted),ego_or_actor_modified=False,gps_registration=False))
    return result,review


def opening_display_polygons(road,junction):
    """A convex scope hull is not asphalt filling the inside of a turn."""
    if 'turn_deg' not in junction:
        peaks=junction.get('camera_station_peaks',[])
        if not peaks:return [junction['polygon_W_xy']]
        # A broad camera/map association interval includes uncertain distant
        # crosswalk projections. It is a search/scope extent, not an asphalt
        # rectangle spanning every candidate station. Keep a short display
        # opening at the robust camera station; separate map arms show shape.
        station=display_station(junction);bounds=display_bounds(road,station)
        width=max(3.5,bounds[0]-bounds[-1]);half=min(10.,max(4.,width*.5+2.))
        return [[world(road,s,d).tolist() for s,d in [(station-half,bounds[-1]),(station+half,bounds[-1]),(station+half,bounds[0]),(station-half,bounds[0])]]]
    a,b=junction['road_station_range_m'];polygons=[]
    cuts=np.r_[np.arange(a,b,3.),b]
    for lo,hi in zip(cuts,cuts[1:]):
        bounds=display_bounds(road,(lo+hi)/2);left,right=bounds[0],bounds[-1]
        polygons.append([world(road,s,d).tolist() for s,d in [(lo,right-1),(hi,right-1),(hi,left+1),(lo,left+1)]])
    return polygons
