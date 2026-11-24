"""Selected development-source event checks; not an exhaustive holdout score."""
import argparse,json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_at

ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
case=Path('runs/new-date-recovery-suite-001/2026-08-28_08-55-51');old=Path('runs/aug28-joint-road-lane-gate-002');manifest=verify(a.scene)
states=[json.loads(l) for l in (a.scene/'render-states.jsonl').read_text().splitlines()];before=[json.loads(l) for l in (old/'render-states.jsonl').read_text().splitlines()]
actors=load(a.scene/'actors.json');mapping={k:v['global_id'] for v in actors for k in v['local_ids']};road=load(a.scene/'report.json')['road']
obs=[json.loads(l) for l in (case/'tracks/observations.jsonl').read_text().splitlines()];front=[r for r in obs if r['camera_id']=='front' and abs(r['camera_time_s']-13.4115)<.03]
def iou(a,b):
    a,b=np.array(a),np.array(b);area=lambda q:np.prod(np.maximum(0,q[2:]-q[:2]));inter=np.prod(np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])));return float(inter/max(1,area(a)+area(b)-inter))
# Manually read from the full raw 1280x960 frame 00067.jpg, after source
# inspection. These are development annotations, not hidden evaluation labels.
targets=[dict(name='white_suv_left_queue',bbox=[3,435,190,580],expected_local='front-0020'),dict(name='dark_sedan_left_queue',bbox=[167,449,289,537],expected_local='front-0033')]
results=[];state=min(states,key=lambda s:abs(s['time_s']-13.4))
for target in targets:
    matches=sorted([(iou(target['bbox'],r['bbox_xyxy']),r['local_track_id']) for r in front],reverse=True);score,key=matches[0];gid=mapping.get(key);rendered=next((r for r in state['actors'] if r['id']==gid),None)
    local=coordinates(road,rendered['position_W']) if rendered else [0,0,0];lane=lane_at(road,float(local[0]),float(local[1])) if rendered else None
    result=dict(target=target,source_time_s=13.4115,match_iou=score,matched_local_id=key,fixed_global_id=gid,solid=bool(rendered and not rendered['ghost']),lane=lane)
    result['passed']=score>.65 and key==target['expected_local'] and result['solid'] and lane==-1;results.append(result)
same=lambda key:[(s['time_s'],r['position_W'],r['yaw_rad']) for s in states for r in s['actors'] if r['id']==key]==[(s['time_s'],r['position_W'],r['yaw_rad']) for s in before for r in s['actors'] if r['id']==key]
review=dict(scope='selected development source anchors only; no complete recall/metric/holdout claim',artifact_id=manifest['artifact_id'],
            source_original_image=str(case/'source/images/00067.jpg'),source_annotations_after_prediction_review=True,
            targets=results,ego_camera_samples_exactly_unchanged=[s['camera_view'] for s in states]==[s['camera_view'] for s in before],
            frozen_lead_pose_samples_exactly_unchanged=same('front-0002'),
            selected_oncoming_front0043_excluded='front-0043' not in mapping,
            inferred_pockets=[dict(station_range_m=[p['start_x'],p['end_x']],vehicle_local_id=p.get('vehicle_local_id'),width_support=p['width_support']) for p in road['left_pockets']],
            ego_events=load(a.scene/'ego-lane-state.json')['events'])
(a.output/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in review.items() if k not in ['ego_events','inferred_pockets']}))
assert all(r['passed'] for r in results) and review['ego_camera_samples_exactly_unchanged'] and review['frozen_lead_pose_samples_exactly_unchanged'] and review['selected_oncoming_front0043_excluded']
