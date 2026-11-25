"""Filter an immutable final world timeline, then replay with the original strip.
No surviving actor, ego or road pose can change; no inference is repeated.
"""
import argparse,copy,json
from pathlib import Path
from ct_scene.artifacts import verify,publish,canonical,digest
from ct_scene.driving_road_scope import mark_raw_scope,filter_plan
from ct_scene.scene_replay import run as replay
from ct_scene.scene_audit import observation_coverage

p=argparse.ArgumentParser()
for name in ['scene','assets-root','prototype-config','output']:p.add_argument('--'+name,type=Path,required=True)
p.add_argument('--tracks',type=Path)
a=p.parse_args()
parent=verify(a.scene)
if a.output.exists():raise FileExistsError(a.output)
report=json.loads((a.scene/'report.json').read_text());road=report['road'];actor_bytes=(a.scene/'actors.json').read_bytes();actors=json.loads(actor_bytes)
states=[json.loads(x) for x in (a.scene/'render-states.jsonl').read_text().splitlines()];before=copy.deepcopy(states)
for actor in actors:mark_raw_scope(actor,road)
removed=filter_plan(states,actors,road)
for old,new in zip(before,states):
    assert {k:v for k,v in old.items() if k!='actors'}=={k:v for k,v in new.items() if k!='actors'}
    old_rows={r['id']:r for r in old['actors']}
    assert all(r==old_rows[r['id']] for r in new['actors'])
proof=dict(parent_scene_artifact_id=parent['artifact_id'],removed=removed,remaining_actor_states_exact=True,ego_and_road_unchanged=True,raw_actors_bytes_exact=True,model_inference_calls=0,policy='post_collision_display_filter_only; uncertainty_abstains; junction_crossing_retained')
report['render_actor_count_range']=[min(len(x['actors']) for x in states),max(len(x['actors']) for x in states)]
report['driving_road_scope']=dict(removed_actor_frames=len(removed),remaining_poses_unchanged=True,raw_observations_preserved=True)
if a.tracks:
    tracks=verify(a.tracks)
    if not any(r['artifact_id']==tracks['artifact_id'] and r['role']=='tracks' for r in parent['inputs']):raise ValueError('Source tracks do not match baseline')
    observations=[json.loads(x) for x in (a.tracks/'observations.jsonl').read_text().splitlines()]
    coverage=observation_coverage(observations,actors,states,[],{})
    report['observation_world_coverage_before_scope']=report.get('observation_world_coverage',{})
    report['observation_world_coverage']=coverage['camera_summary']
else:
    coverage=None
    report['observation_world_coverage_before_scope']=report.pop('observation_world_coverage',{})
# This intermediate plan keeps parent video ONLY as a source-strip witness.
# Its distinct stage is not a completed 3D result; deliver the replay output.
report['scope_plan_video_role']='unfiltered parent video; raw-strip witness only, not the filtered result'
payloads={'surrounding-world.mp4':(a.scene/'surrounding-world.mp4').read_bytes(),'report.json':canonical(report),'render-states.jsonl':b''.join(canonical(x)+b'\n' for x in states),'actors.json':actor_bytes,'driving-road-scope-review.json':canonical(proof)}
if coverage:payloads['observation-world-audit.json']=canonical(coverage)
for name in ['ego-lane-state.json','actor-lane-state.json','camera-strip-coverage.json','branch-hints-review.json','camera-crossing-road-review.json','blindspot-inference-review.json']:
    if (a.scene/name).exists():payloads[name]=(a.scene/name).read_bytes()
plan=a.output.with_name(a.output.name+'-scope-plan')
publish(plan,stage='driving_road_scope_plan',payloads=payloads,inputs=[dict(role='parent_world_scene',artifact_id=parent['artifact_id'])],config=dict(policy=proof['policy']),code_revision=digest((Path(__file__).resolve().parents[1]/'src/ct_scene/driving_road_scope.py').read_bytes()),runtime=dict(cuda=False,model_inference_calls=0))
replay(plan,a.assets_root,a.prototype_config,a.output)
print('Excluded',len(removed),'actor-frames; all retained states exactly preserved')
