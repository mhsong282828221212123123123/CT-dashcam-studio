"""Audit every prominent cached observation, not a hand-picked vehicle subset.

Detection coverage is conditional on detector output, not ground-truth recall.
"""
from collections import defaultdict,Counter
import numpy as np


def audit_motion(timeline):
    source_ids=defaultdict(set);observations=defaultdict(list);excluded_boundary=0
    for scene in timeline.scenes:
        for actor in scene.actors:
            for source in actor.sources:source_ids[source].add(actor.key)
    for i,row in enumerate(timeline.rows):
        for name,data in row['cameras'].items():
            if name=='back' or not data.get('available'):continue
            for obj in data.get('objects',[]):
                if obj.get('track_status')!='confirmed' or obj['kind'] not in ('car','truck','bus','motorcycle','bicycle'):continue
                if obj.get('road_relation')=='beyond_central_boundary':
                    excluded_boundary+=1;continue
                x,y,w,h=obj['bbox']
                if h<(24 if name=='front' else 50) or w<30:continue
                observations[obj['track_id']].append((i,obj))
    tracks=[]
    for source,items in observations.items():
        if len(items)<8:continue
        ids=source_ids[source];missing=[];placed=[];states=Counter()
        for i,obj in items:
            actor=next((a for a in timeline.scenes[i].actors if a.key in ids),None)
            if actor is None:missing.append(round(timeline.scenes[i].time,1));continue
            placed.append((i,actor));states[actor.tracking_state]+=1
        speeds=[];angles=[]
        for (i,a),(j,b) in zip(placed,placed[1:]):
            dt=timeline.scenes[j].time-timeline.scenes[i].time
            if dt>.2:continue
            p=timeline.ego_poses[i][0]+timeline.ego_poses[i][1]@np.array([a.x,a.z])
            q=timeline.ego_poses[j][0]+timeline.ego_poses[j][1]@np.array([b.x,b.z])
            speeds.append((round(float(np.linalg.norm(q-p)/dt),2),round(timeline.scenes[j].time,1)))
            angles.append(float(abs((b.yaw-a.yaw+np.pi)%(2*np.pi)-np.pi)/dt*180/np.pi))
        tracks.append(dict(source=source,kind=items[0][1]['kind'],frames=len(items),rendered=len(placed),
            ids=sorted(ids),missing=missing,states=dict(states),max_speed=max(speeds,default=[0,None]),
            max_body_turn_deg_s=round(max(angles,default=0),2),start=timeline.scenes[items[0][0]].time,
            end=timeline.scenes[items[-1][0]].time))
    tracks.sort(key=lambda r:(-(r['frames']-r['rendered']),-r['max_speed'][0]))
    return dict(tracks=tracks,track_count=len(tracks),observation_frames=sum(r['frames'] for r in tracks),
        missing_frames=sum(len(r['missing']) for r in tracks),
        excluded_boundary_observations=excluded_boundary,
        semantics='all size-eligible detector tracks; not independent ground truth',
        criteria=dict(front_min_height_px=24,side_min_height_px=50,min_width_px=30,min_frames=8))
