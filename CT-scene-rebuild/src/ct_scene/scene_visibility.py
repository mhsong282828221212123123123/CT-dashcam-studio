"""Visibility uses the actual nearest source frame shown in each camera tile."""
from collections import defaultdict
import numpy as np


def build(frames,observations,membership,rear_review=None):
    ids=defaultdict(set)
    references=(rear_review or {}).get('intervals',[])
    for row in observations:
        if row.get('support') not in ('detector_observation','flow_observation'):continue
        gid=membership.get(row['local_track_id'])
        if gid is not None:ids[row['frame_id']].add(gid)
        if row.get('camera_id')=='back':
            choices={r['global_id'] for r in references if r['rear_id']==row['local_track_id'] and r['start_s']<=row['camera_time_s']<=r['end_s']}
            if len(choices)==1:ids[row['frame_id']].update(choices)
    groups=defaultdict(list)
    for frame in frames:groups[frame['camera_id']].append(frame)
    result={}
    for cam,rows in groups.items():
        rows=sorted(rows,key=lambda r:r['camera_time_s'])
        result[cam]=(np.array([r['camera_time_s'] for r in rows]),[ids[r['frame_id']] for r in rows])
    return result


def visible_at(index,t,tolerance=.12):
    visible=set()
    for times,ids in index.values():
        k=int(np.searchsorted(times,t));options=[i for i in (k-1,k) if 0<=i<len(times)]
        if not options:continue
        i=min(options,key=lambda i:abs(times[i]-t))
        if abs(times[i]-t)<=tolerance:visible.update(ids[i])
    return visible
