"""Camera-proven left approach band; preserve main IDs and existing pockets."""
import copy
import numpy as np
from .road_frame import frame_for


def extend(road,actors,geometry_fit,width=3.5):
    result=copy.deepcopy(geometry_fit);frame=frame_for(road);review=[]
    if frame is None or not road.get('junctions'):return result,review
    extent=road['reference_line'].get('supported_extent_m',[float(frame.stations[0]),float(frame.stations[-1])]);regions=road.get('lane_sections') or [dict(station_range_m=extent,boundaries_y=road['boundaries_y'])]
    for actor in actors:
        ids=set(actor.get('inside_median_frame_ids',[]));rows=[r for r in actor['samples'] if r.get('frame_id') in ids]
        if len(rows)<8:continue
        q=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows])
        for section in regions:
            lo,hi=section['station_range_m'];left=section['boundaries_y'][0];mask=(q[:,0]>=lo)&(q[:,0]<hi)&(q[:,1]>left+.5)&(q[:,1]<left+width*1.3)
            if sum(mask)<8:continue
            selected=[r for r,yes in zip(rows,mask) if yes];s=q[mask,0];d=q[mask,1]
            if selected[-1]['time_s']-selected[0]['time_s']<1.2 or np.quantile(abs(d-np.median(d)),.8)>1.2:continue
            ends=[j['road_station_range_m'][0] for j in road['junctions'] if np.max(s)<=j['road_station_range_m'][0]<=np.max(s)+80]
            if not ends:continue
            end=float(min(ends));start=float(max(lo,np.min(s)-30))
            if end-start<15:continue
            if any(start<p['end_x'] and end>p['start_x'] for p in result['pockets']):
                review.append(dict(local_id=actor['id'],status='existing_supported_pocket_preserved'));continue
            result['pockets'].append(dict(id='left-pocket-'+str(len(result['pockets'])),lane_id=-1,start_x=start,end_x=end,width_knots=[[start,width],[end,width]],inner_reference_y=float(left),coordinate_system='road_station_lateral',
                support='repeated_left_raw_double_yellow_inside_vehicle; inferred_adjacent_approach_band',turn_role='left_auxiliary_corridor; turn_direction_not_classified',width_support='nominal_width_prior; outer_metric_edge_unmeasured',
                entry_support='unobserved_start; display_continuation_up_to_30m_before_source_vehicle',exit_support='supported_junction_opening',vehicle_local_id=actor['id'],vehicle_frame_ids=[r['frame_id'] for r in selected],geometry_status='inferred_not_measured'))
            result['occupancy_used']=True;review.append(dict(local_id=actor['id'],status='raw_inside_median_approach_band',station_range_m=[start,end],source_frames=len(selected),main_lane_ids_modified=False))
    return result,review
