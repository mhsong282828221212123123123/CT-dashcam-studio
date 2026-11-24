"""Conditional recovery from independent side ground contacts, never lane snapping."""
from collections import Counter
import numpy as np
from .road_frame import coordinates
from .road_geometry import lane_at


def recover(actor, road):
    if actor['camera_id'] not in ('left_repeater', 'right_repeater'):
        return None
    if road.get('left_boundary_role') != 'inferred_extent_not_confirmed_centreline':
        return None
    if road.get('lane_geometry_status') == 'unknown_route_display_only' or road.get('junctions'):
        return None
    rows=[r for r in actor['samples'] if r.get('side_ground_contact_W') is not None]
    if len(rows)<12 or rows[-1]['time_s']-rows[0]['time_s']<2.4:
        return None
    times=np.array([r['time_s'] for r in rows]); q=coordinates(road,[r['side_ground_contact_W'] for r in rows])
    lanes=[lane_at(road,float(s),float(d)) for s,d,_ in q]
    known=[v for v in lanes if v is not None and v>=0]
    if not known:return None
    lane,count=Counter(known).most_common(1)[0]
    if count/len(rows)<.9:return None
    design=np.c_[times-times[0],np.ones(len(times))]
    fit=np.linalg.lstsq(design,q[:,:2],rcond=None)[0]
    velocity=fit[0]; residual=np.abs(q[:,:2]-design@fit)
    if not 5<=velocity[0]<=55 or abs(velocity[1])>.5:
        return None
    if np.quantile(residual[:,1],.9)>.6 or np.quantile(residual[:,0],.9)>2.:
        return None
    # Use only directly observed, unclipped ground contacts; never translate
    # the rejected height estimate onto a chosen lane or extrapolate its depth.
    recovered=[dict(r,position_W=list(r['side_ground_contact_W']),
                    unrefined_position_W=list(r['side_ground_contact_W']),
                    projection_support='repeated_clear_ground_contact_after_conflicting_height_prior')
               for r,v in zip(rows,lanes) if v==lane]
    return dict(samples=recovered,lane=lane,evidence=dict(
        source='independent_unclipped_bbox_ground_contacts',observations=len(recovered),
        span_s=[recovered[0]['time_s'],recovered[-1]['time_s']],
        longitudinal_speed_mps=float(velocity[0]),lateral_speed_mps=float(velocity[1]),
        lateral_residual_p90_m=float(np.quantile(residual[:,1],.9)),
        road_ego_or_camera_modified=False,lane_translation_applied=False))
