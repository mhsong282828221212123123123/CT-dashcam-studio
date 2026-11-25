"""Create an explicit corrected-position plan, then replay with original 4-camera strip."""
import argparse,copy,json
from pathlib import Path
from ct_scene.artifacts import verify,publish,canonical,digest
from ct_scene.far_lead_stability import refine_far_leads
from ct_scene.collision_constraints import audit as collision_audit
from ct_scene.scene_replay import run as replay


def run(scene,assets_root,prototype_config,lens_config,output):
    scene,output=Path(scene),Path(output);parent=verify(scene)
    if output.exists():raise FileExistsError(output)
    report=json.loads((scene/'report.json').read_text());raw=(scene/'actors.json').read_bytes();actors=json.loads(raw)
    states=[json.loads(x) for x in (scene/'render-states.jsonl').read_text().splitlines()];before=copy.deepcopy(states)
    cfg=json.loads(Path(prototype_config).read_text());profiles=json.loads(Path(lens_config).read_text())['cameras']
    proof=refine_far_leads(states,actors,profiles['front'],report['vehicle_dimensions_m'],cfg['cameras']['front'])
    changed=set(proof['applied_actor_ids']);changed_frames=0
    for old,new in zip(before,states):
        assert {k:v for k,v in old.items() if k!='actors'}=={k:v for k,v in new.items() if k!='actors'}
        assert [x['id'] for x in old['actors']]==[x['id'] for x in new['actors']]
        for a,b in zip(old['actors'],new['actors']):
            if a['id'] not in changed:assert a==b
            else:
                assert {k:v for k,v in a.items() if k not in ('position_W','position_support')}=={k:v for k,v in b.items() if k not in ('position_W','position_support')}
                changed_frames+=a!=b
    proof.update(parent_scene_artifact_id=parent['artifact_id'],non_target_actor_states_exact=True,raw_actors_bytes_exact=True,changed_actor_frames=changed_frames,
                 collision_after=collision_audit(states,report['vehicle_dimensions_m']))
    # Old audits describe upstream geometry, never the newly corrected lead.
    report['far_front_refinement']=dict(applied_actor_ids=sorted(changed),metric_accuracy='NOT_EVALUATED',actor_lane_audit_role='upstream_before_final_refinement')
    report['ego_and_lead_source']='parent frozen ego; lead positions explicitly refined from body-size evidence' if changed else report.get('ego_and_lead_source')
    report['previous_geometry_audits_role']='Upstream input only; final actor positions must use far-front-refinement-review and render-states'
    report['plan_video_role']='Parent video for raw 4-camera strip only; deliver replay output'
    payloads={'surrounding-world.mp4':(scene/'surrounding-world.mp4').read_bytes(),'report.json':canonical(report),'actors.json':raw,
              'render-states.jsonl':b''.join(canonical(s)+b'\n' for s in states),'far-front-refinement-review.json':canonical(proof)}
    for name in ('ego-lane-state.json','actor-lane-state.json','driving-road-scope-review.json','observation-world-audit.json','camera-strip-coverage.json','blindspot-inference-review.json','branch-hints-review.json','camera-crossing-road-review.json'):
        if (scene/name).exists():payloads[name]=(scene/name).read_bytes()
    plan=output.with_name(output.name+'-refinement-plan')
    publish(plan,stage='far_front_refinement_plan',payloads=payloads,inputs=[dict(role='parent_scene',artifact_id=parent['artifact_id'])],
            config=dict(policy=proof['policy'],optics=profiles['front'],mount=cfg['cameras']['front']),
            code_revision=digest(Path(__file__).resolve().parents[1].joinpath('src/ct_scene/far_lead_stability.py').read_bytes()),runtime=dict(cuda=False,model_inference_calls=0))
    replay(plan,Path(assets_root),prototype_config,output)
    print('Refined actor IDs:',sorted(changed),'frames:',changed_frames,flush=True)
    return proof


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('scene','assets-root','prototype-config','lens-config','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.scene,a.assets_root,a.prototype_config,a.lens_config,a.output)
