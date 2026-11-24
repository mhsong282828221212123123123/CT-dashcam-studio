"""Reference gaps may be supported by actual two-sided road-edge observations.

This does not lengthen an unobserved-road TTL. Every spatial bin must contain
repeated camera evidence on BOTH edges of an established one-lane carriageway.
Only the reference centre interpolation remains inferred; poses stay fixed.
"""
from collections import defaultdict
import math
import numpy as np
from .road_frame import frame_for


def support_gaps(road,segments,bin_m=20.,minimum_frames=2):
    line=road.get('reference_line');frame=frame_for(road)
    if frame is None or len(road['boundaries_y'])!=2 or road.get('lane_sections'):
        return line,dict(status='not_single_lane_reference_scope',intervals=[])
    gaps=line.get('unsupported_intervals_m',[])
    if not gaps:return line,dict(status='no_reference_gaps',intervals=[])
    left,right=road['boundaries_y'];projected=[]
    for row in segments:
        if row['kind']!='white' or row.get('ground_length_m',0)<4 or (row.get('color_continuity') or 0)<.7:continue
        s,d,_=frame.project(np.mean(row['points_W_xy'],axis=0))
        projected.append((float(s),float(d),row['frame_id'],row['time_s']))
    retained=[];accepted=[];reviews=[]
    for a,b in gaps:
        if any(not (b<j['road_station_range_m'][0] or a>j['road_station_range_m'][1]) for j in road.get('junctions',[])):
            retained.append([a,b]);reviews.append(dict(station_range_m=[a,b],status='junction_scope_unresolved'));continue
        n=max(1,int(math.ceil((b-a)/bin_m)));bins=[defaultdict(set) for _ in range(n)];times=defaultdict(list)
        for s,d,fid,t in projected:
            if not a<=s<=b:continue
            k=min(n-1,int((s-a)/bin_m))
            for side,edge in [('left',left),('right',right)]:
                if abs(d-edge)<1.:
                    bins[k][side].add(fid);times[fid].append(t)
        counts=[dict(left=len(q['left']),right=len(q['right'])) for q in bins]
        good=all(min(q['left'],q['right'])>=minimum_frames for q in counts)
        evidence=dict(station_range_m=[float(a),float(b)],spatial_bin_m=bin_m,bin_frame_counts=counts,
                      source_frame_ids=sorted(times),status='two_sided_camera_edges_inferred_reference' if good else 'insufficient_two_sided_edge_coverage')
        reviews.append(evidence)
        if good:accepted.append(evidence)
        else:retained.append([a,b])
    new=dict(line,unsupported_intervals_m=retained,original_phase_gaps_m=gaps,
             inferred_reference_intervals_m=accepted)
    return new,dict(status='camera_edges_support_reference_interpolation' if accepted else 'no_gap_accepted',intervals=reviews,
                    ego_modified=False,actor_positions_modified=False,road_width_modified=False,actors_used=False)
