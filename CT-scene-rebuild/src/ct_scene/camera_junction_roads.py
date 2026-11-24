"""Camera-only cross-road display when cached map branches are unavailable.

A real crossing trajectory may justify local pavement beyond the ego turn.
This module never changes object positions, lane IDs or OSM registration.
"""
import copy,math
import numpy as np
from .road_frame import coordinates,world
from .junctions import inside
from .junction_display import display_bounds


def supplement(road,actors):
    junctions=copy.deepcopy(road.get('junctions',[]));review=[]
    for junction in junctions:
        if junction.get('branch_display_hints') or not 45<=abs(junction.get('turn_deg',0))<=130:continue
        if junction.get('crossing_support_frames',0)<3:continue
        a,b=junction['road_station_range_m'];exit_yaw=float(coordinates(road,world(road,b,0))[2]);wanted=exit_yaw+math.pi
        start=junction['start_s']-2.;end=junction['end_s']+1.;candidates=[]
        for actor in actors:
            if actor.get('frozen_lead') or actor.get('lane_index')!=-2:continue
            rows=[r for r in actor['samples'] if start<=r['time_s']<=end and r.get('support')=='detector_observation'
                  and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=70 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=35]
            if len({r['frame_id'] for r in rows})<8 or rows[-1]['time_s']-rows[0]['time_s']<1.2:continue
            if np.mean([inside(road,r['position_W'],5.) for r in rows])<.8:continue
            t=np.array([r['time_s'] for r in rows]);xy=np.array([r['position_W'][:2] for r in rows]);X=np.c_[t-t.mean(),np.ones(len(t))];fit=np.linalg.lstsq(X,xy,rcond=None)[0]
            velocity=fit[0];speed=float(np.linalg.norm(velocity));residual=float(np.quantile(np.linalg.norm(X@fit-xy,axis=1),.9))
            if not 2<=speed<=25 or residual>.6 or np.linalg.norm(xy[-1]-xy[0])<5:continue
            angle=float(math.atan2(velocity[1],velocity[0]));difference=abs(math.atan2(math.sin(angle-wanted),math.cos(angle-wanted)))
            if difference>math.radians(25):continue
            cost=residual/.6+difference/math.radians(25)+abs(float(t.mean())-(junction['start_s']+junction['end_s'])/2)/5
            candidates.append((cost,actor,rows,fit,angle,speed,residual))
        if not candidates:continue
        _,actor,rows,fit,angle,speed,residual=min(candidates,key=lambda row:row[0]);centre=fit[1];direction=np.array([math.cos(angle),math.sin(angle)]);normal=np.array([-direction[1],direction[0]])
        bounds=display_bounds(road,b+.1);width=float(np.clip(2*(bounds[0]-bounds[-1]),6.,12.));lo,hi=-15.,25.
        polygon=[(centre+direction*s+normal*d).tolist() for s,d in [(lo,-width/2),(hi,-width/2),(hi,width/2),(lo,width/2)]]
        arm=dict(polygon_W_xy=polygon,heading_W_rad=angle,origin_W_xy=centre.tolist(),length_m=hi,width_m=width,
                 source='camera_observed_crossing_motion_and_own_turn; display_only',
                 width_support='coarse_camera_corridor_width_prior; no_lane_count_assignment',
                 extent_support='short_local_pavement_hint; surveyed_extent_unknown',supporting_actor_id=actor['global_id'])
        junction['branch_display_hints']=[arm];junction['branch_reconstruction']='camera_crossing_motion_display_hint; no_map_registration'
        review.append(dict(junction_id=junction['id'],source_actor_id=actor['global_id'],source_frame_ids=[r['frame_id'] for r in rows],
                           time_interval_s=[rows[0]['time_s'],rows[-1]['time_s']],fit_residual_p90_m=residual,
                           crossing_speed_mps=speed,heading_W_rad=angle,kind='camera_only_cross_road_display',
                           actor_or_ego_modified=False,lane_ids_modified=False,map_used=False,width_m=width))
    return junctions,review
