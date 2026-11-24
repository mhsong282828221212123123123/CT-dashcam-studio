"""Rebuild and audit the explicitly reviewed single-clip reference."""
import json,pickle,sys,math
from pathlib import Path
from collections import defaultdict
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline
from tools.audit_scene_motion import audit_motion

def motion_outliers(timeline):
    from core.scene_telemetry import odometry_speed_kmh
    history=defaultdict(list);out=[]
    for i,scene in enumerate(timeline.scenes):
        for a in scene.actors:history[a.key].append((i,a))
    for key,items in history.items():
        for (i,a),(j,b) in zip(items,items[1:]):
            dt=timeline.scenes[j].time-timeline.scenes[i].time
            if not 0<dt<=.2:continue
            p=timeline.ego_poses[i][0]+timeline.ego_poses[i][1]@np.array([a.x,a.z])
            q=timeline.ego_poses[j][0]+timeline.ego_poses[j][1]@np.array([b.x,b.z])
            speed=float(np.linalg.norm(q-p)/dt)
            axes=[timeline.ego_poses[k][1]@np.array([math.sin(actor.yaw),math.cos(actor.yaw)])
                  for k,actor in ((i,a),(j,b))]
            headings=[math.atan2(axis[0],axis[1]) for axis in axes]
            turn=abs((headings[1]-headings[0]+math.pi)%(2*math.pi)-math.pi)/dt*180/math.pi
            host_speed=max((odometry_speed_kmh(timeline.scenes[k].telemetry) or 0.)/3.6 for k in (i,j))
            limit=max(25.,host_speed+12.)
            if speed>limit or (speed<1 and turn>60):
                out.append(dict(id=key,time=timeline.scenes[j].time,speed=speed,turn=turn,
                                world_speed_limit_mps=limit,heading_frame='world'))
    return out

if __name__=='__main__':
    root=Path(sys.argv[1]);tag=sys.argv[2]
    source=root/'reviewed-input'
    manifest=json.loads((source/'manifest.json').read_text(encoding='utf8'))
    rows=[json.loads(line) for line in (source/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    for row in rows:
        for data in row['cameras'].values():
            for obj in data.get('objects',[]):
                if obj['track_id']=='front:3':obj.update(geometry_kind='suv',render_kind='suv')
                if obj['track_id'] in ('left_repeater:27','left_repeater:30'):obj['reviewed_world_yaw']=0.
    manifest['reference_review_notes']='Clip-specific manual shape and front/back review; no hand-keyframed positions. Background-derived pitch. Not a general accuracy benchmark.'
    (source/'observations.jsonl').write_text('\n'.join(json.dumps(r,ensure_ascii=False) for r in rows)+'\n',encoding='utf8')
    (source/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
    def progress(s):
        print(s['source'],s.get('accepted'),s.get('occlusion_hypothesis'),flush=True)
    timeline=FusedSceneTimeline(rows,manifest,motion_progress=progress)
    with (root/(tag+'.pkl')).open('wb') as f:pickle.dump(timeline,f)
    for suffix,data in [('audit',audit_motion(timeline)),('models',timeline.motion_bundle_report),
                        ('motion-outliers',motion_outliers(timeline)),('abstentions',timeline.motion_abstentions)]:
        (root/(tag+'-'+suffix+'.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in audit_motion(timeline).items() if k!='tracks'}),flush=True)
