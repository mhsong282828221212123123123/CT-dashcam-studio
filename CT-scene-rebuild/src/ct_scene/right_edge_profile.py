"""Restrict a global outer band where repeated local solid paint closes it.

Only a supported prefix is narrowed. The later global width stays a hypothesis;
this does not claim an exact exit-lane opening point or change any vehicle pose.
"""
import math
import numpy as np
from .road_frame import frame_for


def refine(road,segments,actors,ego,width=3.5):
    frame=frame_for(road);base=road['boundaries_y'];no=dict(status='unchanged_no_supported_right_edge_prefix',changes=[])
    if frame is None or road.get('junctions') or road.get('lane_sections') or len(base)<4:return [],no
    ego=np.asarray(ego);local_ego=frame.project(ego);start,end=road['reference_line'].get('supported_extent_m',[frame.stations[0],frame.stations[-1]])
    paint=[]
    for r in segments:
        if r['kind']!='white' or r.get('ground_length_m',0)<7 or (r.get('color_continuity') or 0)<.8:continue
        p=np.asarray(r['points_W_xy']);q=frame.project(p.mean(axis=0));v=p[-1]-p[0]
        if abs(math.sin(math.atan2(v[1],v[0])-q[2]))>.3:continue
        paint.append((float(q[0]),float(q[1]),r['frame_id']))
    forward=[]
    for actor in actors:
        if actor.get('camera_id')!='front':continue
        rows=[r for r in actor['samples'] if r.get('support')=='detector_observation' and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=80 and min(r['bbox_xyxy'][0],r.get('image_width',1280)-r['bbox_xyxy'][2])>6]
        if len(rows)<8 or rows[-1]['time_s']-rows[0]['time_s']<1:continue
        q=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows]);t=np.array([r['time_s'] for r in rows]);v=np.linalg.lstsq(np.c_[t-t.mean(),t*0+1],q[:,:2],rcond=None)[0][0]
        if 2<v[0]<55 and abs(v[1])<v[0]*.3:forward.extend(q[:,:2].tolist())
    supported=[]
    first=float(np.min(local_ego[:,0]));last=float(np.max(local_ego[:,0]))
    for s in np.arange(math.ceil(first/20)*20,last,20):
        ep=local_ego[abs(local_ego[:,0]-s)<=35]
        if len(ep)<3 or np.quantile(ep[:,1],.05)<base[-2]+.4:continue
        evidence=[r for r in paint if abs(r[0]-s)<=35 and abs(r[1]-base[-2])<.55]
        frames={r[2] for r in evidence}
        if len(frames)<5:continue
        # Clear forward traffic beyond this stripe vetoes treating it as curb.
        if sum(abs(q[0]-s)<=35 and q[1]<base[-2]-.5 for q in forward)>=8:continue
        supported.append(dict(station_m=float(s),source_frames=sorted(frames)))
    if not supported or supported[0]['station_m']-first>60:return [],no
    prefix=[supported[0]]
    for row in supported[1:]:
        if row['station_m']-prefix[-1]['station_m']>80:break
        prefix.append(row)
    if len(prefix)<5 or prefix[-1]['station_m']-prefix[0]['station_m']<80:return [],no
    close_end=prefix[-1]['station_m'];restore=min(end,close_end+40)
    if restore<=close_end:return [],no
    narrow=[*base[:-1],base[-2]]
    knots=[dict(station_m=float(start),boundaries_y=narrow),dict(station_m=close_end,boundaries_y=narrow),dict(station_m=restore,boundaries_y=list(base))]
    if end>restore:knots.append(dict(station_m=float(end),boundaries_y=list(base)))
    section=dict(id='camera-right-edge-prefix',station_range_m=[float(start),float(end)],boundaries_y=list(base),boundary_profile=knots,minimum_profile_lane_width_m=.8,supported=True,support='repeated_local_solid_right_edge; later_global_width_retained_when_unresolved')
    proof=dict(supported_prefix_stations=prefix,solid_boundary_y=float(base[-2]),narrow_end_station_m=close_end,unresolved_transition_end_m=restore,ego_or_actor_pose_modified=False,scope='display_extent_only; exact_lane_opening_station_unknown')
    return [section],dict(status='camera_supported_right_edge_prefix',changes=[proof])
