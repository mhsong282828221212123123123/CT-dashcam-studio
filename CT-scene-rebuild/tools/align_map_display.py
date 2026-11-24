"""Revise only the map display on a preserved automatic scene, then replay it."""
import argparse,copy,json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,publish,canonical,digest
from ct_scene.context_surface import extract
from ct_scene.scene_replay import run as replay

ap=argparse.ArgumentParser()
for name in ('scene','base','telemetry','context','assets-root','output'):ap.add_argument('--'+name,type=Path,required=True)
ap.add_argument('--prototype-config',type=Path,default=Path('configs/surrounding-prototype.json'))
a=ap.parse_args();sm,bm,tm,mm=[verify(p) for p in (a.scene,a.base,a.telemetry,a.context)]
expected={i['role']:i['artifact_id'] for i in sm['inputs']}
for role,m in [('frozen_ego_lead',bm),('ego_lamp_telemetry',tm),('osm_topology_reference',mm)]:
    if expected.get(role)!=m['artifact_id']:raise ValueError('Mismatched frozen scene dependency: '+role)
report=json.loads((a.scene/'report.json').read_text());road=copy.deepcopy(report['road'])
poses=json.loads((a.base/'trajectory.json').read_text());records=json.loads((a.context/'context.json').read_text())
context=extract(mm,a.telemetry,np.array([p['camera_time_s'] for p in poses]),np.array([p['ego_position_W'] for p in poses]),records,[Path('map-cache')],road=road)
old_context=road.get('context_surface',{});road['context_surface']=context
camera_before=copy.deepcopy(report['road']);camera_before.pop('context_surface',None)
camera_after=copy.deepcopy(road);camera_after.pop('context_surface',None)
assert camera_before==camera_after
proof=dict(role='map_display_only',parent_scene_artifact_id=sm['artifact_id'],
           previous_status=old_context.get('status'),previous_polygons=len(old_context.get('polygons_W_xy',[])),
           revised_status=context['status'],revised_polygons=len(context['polygons_W_xy']),
           main_road_alignment=context.get('main_road_alignment'),
           camera_road_ego_actor_and_state_unchanged=True)
report.update(road=road,map_display_revision=proof)
payloads={'report.json':canonical(report),'road-display-review.json':canonical(proof)}
for name in ('surrounding-world.mp4','render-states.jsonl','actors.json','ego-lane-state.json','actor-lane-state.json','prediction-display-review.json'):
    if (a.scene/name).is_file():payloads[name]=(a.scene/name).read_bytes()
revision=a.output.with_name(a.output.name+'-state')
publish(revision,stage='road_display_revision',payloads=payloads,
        inputs=[dict(role='automatic_scene',artifact_id=sm['artifact_id']),dict(role='map_context',artifact_id=mm['artifact_id'])],
        config=dict(map_display=context,video_role='parent_video_for_camera_strip_only'),
        code_revision=digest(Path('src/ct_scene/context_surface.py').read_bytes()+Path(__file__).read_bytes()),runtime=dict(cuda=False,network_requests=0))
result=replay(revision,a.assets_root,a.prototype_config,a.output)
assert (a.output/'render-states.jsonl').read_bytes()==(a.scene/'render-states.jsonl').read_bytes()
print(json.dumps(dict(artifact_id=result['artifact_id'],review=proof)),flush=True)
