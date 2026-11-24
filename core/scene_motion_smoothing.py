"""Two-sided Kalman smoothing with image validation, no new track identities."""
from collections import defaultdict
import numpy as np


def smooth_motion(scenes,poses,states,rows,cameras,audits,eligible):
    from core.scene_fusion import corners,dimensions
    from core.scene_analysis import iou
    tracks=defaultdict(list);count=0
    for i,scene in enumerate(scenes):
        if i not in eligible:continue
        for actor in scene.actors:
            if actor.tracking_state=='observed' and actor.key in states[i]:tracks[actor.key].append((i,actor))
    for history in tracks.values():
        groups=[]
        for item in history:
            if not groups or item[0]!=groups[-1][-1][0]+1:groups.append([])
            groups[-1].append(item)
        pending=[(group,False) for group in groups]
        while pending:
            group,fade_start=pending.pop()
            if len(group)<6:continue
            x,P=states[group[-1][0]][group[-1][1].key][:2]
            smoothed=[x.copy()]
            for (i,a),(j,_) in zip(group[-2::-1],group[:0:-1]):
                dt=scenes[j].time-scenes[i].time
                filtered,cov=states[i][a.key][:2]
                F=np.eye(4);F[:2,2:]=np.eye(2)*dt
                G=np.vstack([np.eye(2)*dt*dt/2,np.eye(2)*dt])
                next_state=states[j][a.key]
                variance=next_state[2] if len(next_state)>2 else 2.25
                predicted=F@cov@F.T+G@G.T*variance
                gain=np.linalg.solve(predicted,F@cov).T
                x=filtered+gain@(x-F@filtered);smoothed.append(x.copy())
            smoothed.reverse()
            def supported(i,a,point):
                if not np.isfinite(point).all() or np.linalg.norm(point-[a.x,a.z])>3:return False
                observations={o['track_id']:(name,o) for name,data in rows[i].get('cameras',{}).items()
                              if name!='back' and data.get('available',True)
                              for o in data.get('objects',[])}
                valid=False
                for source in a.sources:
                    evidence=observations.get(source)
                    if evidence is None:continue
                    name,obj=evidence;geometry=cameras[i].get(name)
                    if geometry is None:continue
                    shape=a.geometry_kind or a.kind
                    bounds=geometry.visible_bounds(corners(*point,dimensions(shape),yaw=a.yaw,kind=shape))
                    if bounds is not None and iou(np.r_[bounds[:2],bounds[2:]-bounds[:2]],obj['bbox'])>=.5:valid=True
                return valid
            points=[poses[i][1].T@(state[:2]-poses[i][0]) for (i,_),state in zip(group,smoothed)]
            valid=[supported(i,a,p) for (i,a),p in zip(group,points)]
            if not all(valid):
                # An occluded endpoint must not disable useful future evidence
                # for every earlier frame. Re-solve independent validated runs,
                # retaining rejected positions and the same track identity.
                start=0
                for end in range(len(group)+1):
                    if end<len(group) and valid[end]:continue
                    if end-start>=6:pending.append((group[start:end],fade_start or start>0))
                    start=end+1
                continue
            for (i,a),point in zip(group,points):
                if fade_start:
                    q=min(1.,(scenes[i].time-scenes[group[0][0]].time)/.5)
                    point=(1-q)*np.array([a.x,a.z])+q*point
                    if not supported(i,a,point):continue
                old=[a.x,a.z];a.x,a.z=map(float,point);count+=1
                audits[i].setdefault('motion_smoothing',[]).append(dict(id=a.key,previous=old,
                    position=point.tolist(),source='two_sided_kalman_image_validated',surveyed_accuracy=False))
    return count
