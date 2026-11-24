"""Score a raw-reviewed manoeuvre separately from lane/road accuracy."""
import argparse,json,math
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,digest
from ct_scene.ingest import file_hash
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_at
from review_guards import require_window


def evaluate(states,actor_id,events,road,control):
    start,end=control['required_world_interval_s'];require_window(states,start,end)
    window=[s for s in states if start<=s['time_s']<=end]
    actors=[next((a for a in s['actors'] if a['id']==actor_id),None) for s in window]
    checks=dict(full_world_presence=bool(actors and all(a is not None for a in actors)))
    candidates=[e for e in events if e['direction']=='right' and e['from_lane']==0 and e['to_lane']==1
                and e.get('completion_s') is not None and start<=e['crossing_s']<=end]
    checks['right_change_event']=len(candidates)==1
    if candidates:
        e=candidates[0];a,b=control['crossing_interval_s'];checks['crossing_matches_raw_interval']=a<=e['crossing_s']<=b
        a,b=control['completion_interval_s'];checks['completion_matches_raw_interval']=a<=e['completion_s']<=b
    metrics={}
    if checks['full_world_presence']:
        checks['visible_car_not_ghosted']=all(not a['ghost'] for a in actors)
        p=np.array([a['position_W'] for a in actors]);t=np.array([s['time_s'] for s in window])
        yaw=np.unwrap([a['yaw_rad'] for a in actors]);delta=np.diff(p,axis=0);speed=np.linalg.norm(delta,axis=1)/np.diff(t)
        angles=np.abs(np.diff(yaw));local=coordinates(road,p)
        checks['moves_right_in_fixed_road_frame']=bool(local[-1,1]-local[0,1]<-2.)
        checks['no_teleport_or_heading_flip']=bool(np.max(speed)<65 and np.max(angles)<math.radians(25))
        moving=np.linalg.norm(delta[:,:2],axis=1)>.01
        alignment=np.cos(yaw[1:][moving]-np.arctan2(delta[moving,1],delta[moving,0]))
        checks['body_points_along_motion']=bool(len(alignment) and np.quantile(alignment,.1)>.7)
        metrics=dict(max_speed_mps=float(np.max(speed)),max_yaw_step_deg=float(np.degrees(np.max(angles))),
                     road_lateral_delta_m=float(local[-1,1]-local[0,1]),body_motion_alignment_p10=float(np.quantile(alignment,.1)) if len(alignment) else None)
    lanes=[]
    for anchor in control['manual_source_anchors']:
        if 'vehicle_lane' not in anchor:continue
        s=min(states,key=lambda s:abs(s['time_s']-anchor['time_s']))
        a=next((a for a in s['actors'] if a['id']==actor_id),None)
        got=lane_at(road,*map(float,coordinates(road,a['position_W'])[:2])) if a else None
        ego=lane_at(road,*map(float,coordinates(road,s['ego_position_W'])[:2]))
        lanes.append(dict(time_s=anchor['time_s'],expected_vehicle_lane=anchor['vehicle_lane'],render_vehicle_lane=got,
                          expected_ego_lane=anchor['ego_lane'],render_ego_lane=ego))
    spatial=bool(lanes and all(r['expected_vehicle_lane']==r['render_vehicle_lane'] and r['expected_ego_lane']==r['render_ego_lane'] for r in lanes))
    motion=bool(checks and all(checks.values()))
    return dict(overall_source_gate='PASS' if motion and spatial else 'FAIL',motion_gate='PASS' if motion else 'FAIL',checks=checks,metrics=metrics,
                lane_anchors=lanes,source_lane_gate='PASS' if spatial else 'FAIL',
                scope='One raw-reviewed right lane change; motion pass does not waive road/ego lane failures')


def main():
    p=argparse.ArgumentParser()
    for key in ('scene','control','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();m=verify(a.scene);control=json.loads(a.control.read_text())
    if file_hash(Path(control['source_path']))!=control['source_sha256']:raise ValueError('Raw control source changed')
    actors=json.loads((a.scene/'actors.json').read_text());target=next(x for x in actors if control['source_local_id'] in x['local_ids'])
    front=[r for r in target['samples'] if r.get('camera_id')=='front']
    if not front or any(not r['frame_id'].startswith(control['source_sha256']+':') for r in front):raise ValueError('Source control/scene mismatch')
    states=[json.loads(l) for l in (a.scene/'render-states.jsonl').read_text().splitlines()]
    road=json.loads((a.scene/'report.json').read_text())['road'];events=json.loads((a.scene/'actor-lane-state.json').read_text())[target['global_id']]['events']
    result=evaluate(states,target['global_id'],events,road,control)
    result.update(scene_artifact_id=m['artifact_id'],control_sha256=digest(a.control.read_bytes()),reviewer_code_sha256=digest(Path(__file__).read_bytes()))
    a.output.mkdir(exist_ok=False);(a.output/'review.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    return 0 if result['overall_source_gate']=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
