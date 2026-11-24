"""Independent camera road extents between supported junction openings.

Each section has local left-to-right lane indices. Changing an index at a
section boundary is a topology transition, not an ego/actor lane change.
No ego or observed object positions are modified here.
"""
import numpy as np
from .road_frame import frame_for
from .curved_road import supported_boundaries,vehicle_supported_paint_gaps


def fit(road,segments,actors,ego,width,max_lanes=4):
    frame=frame_for(road)
    if frame is None or not road.get('junctions'):return [],dict(status='no_supported_junction_partition')
    extent=road['reference_line'].get('supported_extent_m',[float(frame.stations[0]),float(frame.stations[-1])])
    gaps=sorted([j['road_station_range_m'] for j in road['junctions']]);merged=[]
    for a,b in gaps:
        if merged and a<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],b)
        else:merged.append([a,b])
    regions=[];start=extent[0]
    for a,b in merged:
        if a-start>=15:regions.append([start,a])
        start=max(start,b)
    if extent[1]-start>=15:regions.append([start,extent[1]])
    paint=[(float(frame.project(np.mean(r['points_W_xy'],axis=0))[0]),r) for r in segments]
    ego=np.asarray(ego);elocal=frame.project(ego);sections=[];reviews=[]
    for a,b in regions:
        ep=ego[(elocal[:,0]>=a)&(elocal[:,0]<b)]
        if len(ep)<3:continue
        local_paint=[r for s,r in paint if a<=s<b];local_actors=[]
        for actor in actors:
            rows=actor['samples'];local=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows])
            selected=[r for s,r in zip(local[:,0],rows) if a<=s<b]
            if len(selected)>=3:local_actors.append(dict(actor,samples=selected))
        divider=road['left_boundary_y'] if road.get('left_boundary_role')=='camera_observed_yellow_divider_estimated_geometry' else None
        bounds,review=supported_boundaries(local_paint,road['reference_line'],width,local_actors,max_lanes,ego_positions=ep,inherited_divider=divider)
        if bounds is None:
            bounds,traffic=vehicle_supported_paint_gaps(review,road['reference_line'],width,local_actors,ep,max_lanes)
            review=dict(paint=review,traffic=traffic)
        reviews.append(dict(station_range_m=[a,b],**review))
        if bounds is None:continue
        sections.append(dict(id='road-section-'+str(len(sections)),station_range_m=[float(a),float(b)],
                             boundaries_y=[float(x) for x in bounds],lane_count=len(bounds)-1,supported=True,
                             support='local_camera_paint_with_explicit_optional_vehicle_width_priors',evidence=review))
    if len(sections)<2:return [],dict(status='insufficient_two_sided_local_support',regions=reviews)
    # A pointless split of identical geometry adds no information and should
    # not reset lane-event state on otherwise unchanged roads.
    changed=any(len(s['boundaries_y'])!=len(sections[0]['boundaries_y']) or
                np.max(abs(np.asarray(s['boundaries_y'])-sections[0]['boundaries_y']))>width*.6
                for s in sections[1:])
    if not changed:return [],dict(status='local_geometry_consistent_with_global',regions=reviews)
    return sections,dict(status='camera_supported_piecewise_lane_sections',regions=reviews,ego_modified=False,
                         actor_positions_modified=False,lane_indices='local_to_section; topology transition is not a lane change')
