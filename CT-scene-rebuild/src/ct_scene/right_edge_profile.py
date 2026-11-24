"""Restrict a global outer band where repeated local solid paint closes it.

A supported edge is retained across paint gaps; a later global width alone
cannot reopen a lane. No vehicle pose is changed.
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
    steady=_steady_clearance_profile(road,paint,ego,width)
    if steady is not None:return steady
    supported=[]
    first=float(np.min(local_ego[:,0]));last=float(np.max(local_ego[:,0]))
    for s in np.arange(math.ceil(first/20)*20,last,20):
        ep=local_ego[abs(local_ego[:,0]-s)<=35]
        if len(ep)<3 or np.quantile(ep[:,1],.05)<base[-2]+.4:continue
        evidence=[r for r in paint if abs(r[0]-s)<=35 and abs(r[1]-base[-2])<.55]
        frames={r[2] for r in evidence}
        if len(frames)<5:continue
        # Vehicle estimates alone cannot turn a persistent solid curb into a divider.
        supported.append(dict(station_m=float(s),source_frames=sorted(frames)))
    if not supported or supported[0]['station_m']-first>60:return [],no
    prefix=[supported[0]]
    for row in supported[1:]:
        if row['station_m']-prefix[-1]['station_m']>80:break
        prefix.append(row)
    if len(prefix)<5 or prefix[-1]['station_m']-prefix[0]['station_m']<80:return [],no
    close_end=prefix[-1]['station_m'];wide=[]
    dashed=[]
    for r in segments:
        if r['kind']=='white' and 1<=r.get('ground_length_m',0)<=5:
            q=frame.project(np.mean(r['points_W_xy'],axis=0));dashed.append((float(q[0]),float(q[1]),r['frame_id']))
    for station in np.arange(close_end+20,end,20):
        inner={fid for x,d,fid in paint if abs(x-station)<=35 and abs(d-base[-2])<.55}
        outer={fid for x,d,fid in paint if abs(x-station)<=35 and abs(d-base[-1])<.55}
        divider={fid for x,d,fid in dashed if abs(x-station)<=35 and abs(d-base[-2])<.55}
        if len(inner)<3 and len(outer)>=5 and len(divider)>=5:wide.append(float(station))
    runs=[]
    for station in wide:
        if not runs or station-runs[-1][-1]>21:runs.append([])
        runs[-1].append(station)
    confirmed=[r for r in runs if len(r)>=3 and r[-1]-r[0]>=40]
    restore=confirmed[0][0] if confirmed else None
    narrow=[*base[:-1],base[-2]]
    knots=[dict(station_m=float(start),boundaries_y=narrow),dict(station_m=close_end,boundaries_y=narrow)]
    if restore is not None:
        knots.append(dict(station_m=restore,boundaries_y=list(base)))
    if end>knots[-1]['station_m']:knots.append(dict(station_m=float(end),boundaries_y=list(base) if restore is not None else narrow))
    section=dict(id='camera-right-edge-prefix',station_range_m=[float(start),float(end)],boundaries_y=list(base),boundary_profile=knots,minimum_profile_lane_width_m=.8,supported=True,support='repeated_local_solid_right_edge; held_until_observed_new_outer_edge_and_dashed_separator')
    proof=dict(supported_prefix_stations=prefix,solid_boundary_y=float(base[-2]),narrow_end_station_m=close_end,confirmed_expansion_station_m=restore,missing_paint_does_not_add_lane=True,ego_or_actor_pose_modified=False,scope='display_extent_only; exact_lane_opening_station_unknown')
    return [section],dict(status='camera_supported_right_edge_prefix',changes=[proof])


def _steady_clearance_profile(road,paint,ego,width):
    """A reference-phase shift is not a new lane when camera curb clearance stays fixed."""
    frame=frame_for(road);q=frame.project(ego)
    if np.any(np.diff(q[:,0])<-.5):return None
    base=road['boundaries_y'];rows=[]
    for s in np.arange(math.ceil(float(q[:,0].min())/20)*20,float(q[:,0].max()),20):
        ep=q[abs(q[:,0]-s)<=35]
        if len(ep)<3:continue
        centre=float(np.median(ep[:,1]));bins={}
        for station,d,fid in paint:
            if abs(station-s)<=35 and .55<centre-d<width*2:bins.setdefault(round(d/.5),[]).append((d,fid))
        good=[v for v in bins.values() if len({fid for d,fid in v})>=5]
        if not good:continue
        v=max(good,key=lambda values:np.median([d for d,fid in values]));edge=float(np.median([d for d,fid in v]));rows.append(dict(station_m=float(s),edge_y=edge,own_y=centre,clearance_m=centre-edge,source_frames=sorted({fid for d,fid in v})))
    if len(rows)<10 or rows[-1]['station_m']-rows[0]['station_m']<150:return None
    clearance=np.array([r['clearance_m'] for r in rows]);edges=np.array([r['edge_y'] for r in rows]);own=np.array([r['own_y'] for r in rows])
    if sum(abs(edges-base[-2])<.6)<3 or sum(abs(edges-base[-1])<.6)<3:return None
    if np.ptp(own)<width*.65 or np.quantile(clearance,.9)-np.quantile(clearance,.1)>width*.25:return None
    if not width*.3<float(np.median(clearance))<width*.7 or np.quantile(clearance,.1)<width*.25:return None
    start,end=road['reference_line'].get('supported_extent_m',[frame.stations[0],frame.stations[-1]]);stations=np.unique(np.r_[start,end,np.arange(start,end,10.),q[:,0]])
    stations=stations[(stations>=start)&(stations<=end)];order=np.argsort(q[:,0]);own_s=q[order,0];own_d=q[order,1];narrow=np.asarray(base[:-1]);knots=[]
    for station in stations:
        gap=float(np.interp(station,[r['station_m'] for r in rows],clearance));edge=float(np.interp(station,own_s,own_d)-gap)
        knots.append(dict(station_m=float(station),boundaries_y=(narrow+edge-narrow[-1]).tolist()))
    section=dict(id='camera-right-edge-gauge',station_range_m=[float(start),float(end)],boundaries_y=narrow.tolist(),boundary_profile=knots,lane_count=len(narrow)-1,minimum_profile_lane_width_m=.8,supported=True,support='camera_constant_right_edge_clearance; road_reference_gauge_only')
    proof=dict(source_edge_windows=rows,lane_count=len(narrow)-1,road_reference_shift_not_extra_lane=True,ego_or_actor_pose_modified=False,vehicle_positions_used=False,unobserved_spans='two_sided_curb_clearance_prior; not_independent_paint_measurement')
    return [section],dict(status='camera_right_edge_clearance_profile',changes=[proof])
