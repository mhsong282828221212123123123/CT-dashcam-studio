"""A turning ego route is a display/search reference, never a measured lane.

When camera lane fitting fails, using world Y=0 as a road can delete every car
after a turn. Keep the known ego route, mark all lane geometry unknown, and
retain observed actors without a parallel-lane position prior.
"""
import numpy as np
from .curved_road import route_index


def route_display(ego,yaws):
    p=np.asarray(ego,float);yaw=np.unwrap(yaws)
    if np.ptp(yaw)<np.radians(20) or np.sum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))<25:return None
    try:frame=route_index(p,yaws=yaw)
    except ValueError:return None
    a,b=map(float,frame.stations[[0,-1]])
    return dict(points_W_xy=frame.points.tolist(),station_origin_m=a,supported_extent_m=[a,b],
                unsupported_intervals_m=[[a,b]],support='ego_route_display_only; camera_lane_geometry_unknown',
                camera_coverage='none_accepted',station_support=[],estimated_rig=True,
                ego_used=True,actors_used=False,osm_used=False,lane_priors_enabled=False)
