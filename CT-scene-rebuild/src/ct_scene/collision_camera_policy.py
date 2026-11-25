"""Permit depth-only correction where an unresolved road previously locked it."""
import numpy as np
from .road_frame import coordinates
from .road_geometry import lane_at
from .junctions import inside as inside_junction


def unresolved_front_depth_axis(row,position,road,ego_at_observation,ego_now):
    if row.get('camera_id')!='front' or row.get('support') not in ('detector_observation','flow_observation'):return None
    box=row.get('bbox_xyxy')
    if box is None or min(box[2]-box[0],box[3]-box[1])<25:return None
    if 'truncated' in row.get('quality_flags',[]) or min(box[0],box[1],row.get('image_width',1280)-box[2],row.get('image_height',960)-box[3])<6:return None
    q=coordinates(road,position)
    if lane_at(road,float(q[0]),float(q[1])) is not None or inside_junction(road,position,5.):return None
    extent=(road.get('reference_line') or {}).get('supported_extent_m')
    if extent and not extent[0]<=q[0]<=extent[1]:return None
    if np.linalg.norm(np.asarray(position)[:2]-np.asarray(ego_now)[:2])<=8.:return None
    raw=np.asarray(row.get('unrefined_position_W',row['position_W']))[:2]
    ray=raw-np.asarray(ego_at_observation)[:2];length=np.linalg.norm(ray)
    return (ray/length).tolist() if length>1. else None
