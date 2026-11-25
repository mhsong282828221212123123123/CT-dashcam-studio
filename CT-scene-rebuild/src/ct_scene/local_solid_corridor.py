"""Local narrow-road profiles from camera paint, without map lane counts."""
import math
import numpy as np
from .road_frame import frame_for
from .curved_road import supported_boundaries


def fit(road,segments,actors,ego,width=3.5,max_lanes=4):
    no=dict(status='no_sustained_camera_narrow_corridor',changes=[])
    frame=frame_for(road)
    if frame is None or road.get('junctions') or road.get('lane_sections') or len(road['boundaries_y'])<3:return [],no
    ego=np.asarray(ego);elocal=frame.project(ego);paint=[(float(frame.project(np.mean(r['points_W_xy'],axis=0))[0]),r) for r in segments];rows=[];wide_rows=[]
    for s in np.arange(math.ceil(float(elocal[:,0].min())/20)*20,float(elocal[:,0].max()),20):
        ep=ego[abs(elocal[:,0]-s)<=35]
        if len(ep)<3:continue
        local_actors=[]
        for actor in actors:
            q=frame.project([r.get('unrefined_position_W',r['position_W']) for r in actor['samples']]);selected=[r for r,p in zip(actor['samples'],q) if abs(p[0]-s)<=35]
            if len(selected)>=3:local_actors.append(dict(actor,samples=selected))
        bounds,review=supported_boundaries([r for x,r in paint if abs(x-s)<=35],road['reference_line'],width,local_actors,max_lanes,ego_positions=ep)
        if bounds is None:continue
        yellow=any(p.get('kind')=='yellow_divider' and abs(p['y']-bounds[0])<.2 and p.get('frames',0)>=5 for p in review.get('peaks',[]))
        continuous=review['status']=='repeated_continuous_carriageway_edges'
        row=dict(station_m=float(s),boundaries_y=bounds,lane_count=len(bounds)-1,evidence=review)
        if continuous or yellow:rows.append(row)
        elif len(bounds)>2 and review['status']=='semantic_paint_with_optional_outer_vehicle_extent':wide_rows.append(row)
    narrow=[r for r in rows if r['lane_count']==1];runs=[]
    for r in narrow:
        if not runs or r['station_m']-runs[-1][-1]['station_m']>100:runs.append([])
        runs[-1].append(r)
    runs=[r for r in runs if len(r)>=5 and r[-1]['station_m']-r[0]['station_m']>=100]
    if not runs:return [],no
    run=max(runs,key=len);lo,hi=run[0]['station_m'],run[-1]['station_m']
    contradiction=[]
    stations=[r['station_m'] for r in run];left=[r['boundaries_y'][0] for r in run];right=[r['boundaries_y'][-1] for r in run]
    for actor in actors:
        if actor.get('camera_id')!='front':continue
        clear=[r for r in actor['samples'] if r.get('support')=='detector_observation' and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=60 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=40 and min(r['bbox_xyxy'][0],r.get('image_width',1280)-r['bbox_xyxy'][2])>6]
        if len(clear)<10:continue
        q=frame.project([r.get('unrefined_position_W',r['position_W']) for r in clear]);out=(q[:,1]>np.interp(q[:,0],stations,left)+.4)|(q[:,1]<np.interp(q[:,0],stations,right)-.4);indices=np.flatnonzero(out&(q[:,0]>=lo-20)&(q[:,0]<=hi+20))
        if len(indices)<10:continue
        selected=[clear[i] for i in indices];t=np.array([r['time_s'] for r in selected]);local=q[indices]
        if t[-1]-t[0]<1.5:continue
        velocity=np.linalg.lstsq(np.c_[t-t.mean(),t*0+1],local[:,:2],rcond=None)[0][0]
        if 2<velocity[0]<55 and abs(velocity[1])<min(1.,velocity[0]*.2):contradiction.append(actor['id'])
    if contradiction:return [],dict(status='rejected_clear_parallel_traffic_beyond_narrow_edges',changes=[],traffic_veto_ids=contradiction)
    # Keep only persistent width states, not a brief fragmented stripe near a merge.
    selected=list(run)
    after=[r for r in rows+wide_rows if r['station_m']>hi and r['lane_count']>1]
    for count in sorted({r['lane_count'] for r in after}):
        group=sorted([r for r in after if r['lane_count']==count],key=lambda r:r['station_m'])
        if len(group)>=3 and group[-1]['station_m']-group[0]['station_m']>=40:selected+=group
    selected.sort(key=lambda r:r['station_m']);base=road['boundaries_y'];size=max(len(base),max(len(r['boundaries_y']) for r in selected))
    def pad(values):
        missing=size-len(values);anchor=np.linspace(base[0],base[-1],size)
        choices=[[values[0]]*left+list(values)+[values[-1]]*(missing-left) for left in range(missing+1)]
        return min(choices,key=lambda p:float(np.sum((np.asarray(p)-anchor)**2)))
    start,end=road['reference_line'].get('supported_extent_m',[float(frame.stations[0]),float(frame.stations[-1])]);knots=[dict(station_m=float(start),boundaries_y=pad(base))]
    begin=max(float(start),lo-40)
    if begin>start:knots.append(dict(station_m=begin,boundaries_y=pad(base)))
    wide_start=min([r['station_m']-35 for r in selected if r['lane_count']>1],default=float('inf'))
    for r in selected:
        if r['lane_count']==1 and r['station_m']>wide_start-35:continue
        effective=r['station_m']-35 if r['lane_count']>1 else r['station_m']
        knots.append(dict(station_m=effective,boundaries_y=pad(r['boundaries_y'])))
    last=knots[-1]
    if end-last['station_m']>80:
        knots.extend([dict(station_m=last['station_m']+40,boundaries_y=last['boundaries_y']),dict(station_m=last['station_m']+80,boundaries_y=pad(base))])
    if end>knots[-1]['station_m']:knots.append(dict(station_m=float(end),boundaries_y=knots[-1]['boundaries_y']))
    section=dict(id='camera-solid-width-profile',station_range_m=[float(start),float(end)],boundaries_y=list(base),boundary_profile=knots,camera_width_profile=True,minimum_profile_lane_width_m=.8,lane_count=size-1,supported=True,support='local_camera_edges_with_explicit_wider_traffic_extent_priors; no_map_lane_count')
    proof=dict(narrow_stations_m=[r['station_m'] for r in run],observed_width_states=[dict(station_m=r['station_m'],boundaries_y=r['boundaries_y'],lane_count=r['lane_count'],evidence=r['evidence']) for r in selected],vehicle_role='narrowing_veto; wider_following_regions_may_use_labelled_forward_vehicle_extent_priors',ego_or_actor_pose_modified=False,transition_role='wider_state_begins_at_lower_edge_of_35m_paint_support_window; overlap_transition_estimated',wide_support_window_lower_edge_m=-35.)
    return [section],dict(status='camera_local_solid_width_profile',changes=[proof])


def unresolved_side_height_position(road,actor,sample,position):
    """Keep a weak off-road side estimate unlocalized, not on a phantom lane."""
    if actor.get('frozen_lead') or actor.get('lane_index')==-2:return False
    if sample.get('camera_id') not in ('left_repeater','right_repeater') or sample.get('projection_support')!='apparent_vehicle_height_prior':return False
    from .road_frame import coordinates
    from .road_geometry import section_at,boundaries_at
    station,lateral,_=coordinates(road,position);section=section_at(road,float(station))
    if not section:return False
    bounds=boundaries_at(road,float(station))
    if section.get('camera_width_profile') and len(bounds)==2:margin=.25
    elif not section.get('camera_width_profile') and section.get('support','').startswith(('local_camera_paint','local_repeated_outer_paint')):margin=.75
    else:return False
    diag=sample.get('side_projection_diagnostics',{})
    # A change of fusion weight across 0.5 is not evidence that a clearly
    # observed vehicle disappeared. Allow only calibrated lateral rig error;
    # radial depth uncertainty must not admit arbitrarily distant off-road cars.
    if (diag.get('pitch_calibration_accepted') and diag.get('clear_ground_candidate')
            and 0<float(diag.get('yaw_sigma_rad',0.))<=math.radians(5.)):
        margin+=min(2.5,2.*float(diag.get('yaw_sigma_m',0.)))
    return bool(lateral>bounds[0]+margin or lateral<bounds[-1]-margin)
