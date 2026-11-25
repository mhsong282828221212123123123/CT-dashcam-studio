"""Offline collision-policy diagnostic, never modifies the source artifact."""
import argparse,json,numpy as np
from pathlib import Path
from ct_scene.artifacts import verify
from review_guards import require_input_artifact
from ct_scene.road_frame import coordinates
from ct_scene.junctions import inside as inside_junction
from ct_scene.road_geometry import lane_at
from ct_scene.collision_constraints import constrain,omit_conflicting_predictions
from ct_scene.scene_visibility import build,visible_at


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--tracks',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    manifest=verify(a.scene);require_input_artifact(manifest,verify(a.source),'source');require_input_artifact(manifest,verify(a.tracks),'tracks')
    actors=json.loads((a.scene/'actors.json').read_text());report=json.loads((a.scene/'report.json').read_text());byid={a.get('global_id',a['id']):a for a in actors};states=[json.loads(x) for x in (a.scene/'render-states.jsonl').read_text().splitlines()];ts=np.array([s['time_s'] for s in states]);ego=np.array([s['ego_position_W'] for s in states]);obs=[json.loads(x) for x in (a.tracks/'observations.jsonl').read_text().splitlines()];idx=build(json.loads((a.source/'frames.json').read_text()),obs,{k:a.get('global_id',a['id']) for a in actors for k in a['local_ids']})
    for state in states:
        visible=visible_at(idx,state['time_s'])
        for body in state['actors']:
            track=byid[body['id']];r=min(track['samples'],key=lambda r:abs(r['time_s']-state['time_s']));body['position_W']=body.get('position_before_collision_W',body['position_W']);body['collision_cap_m']=float(np.clip(2.58*r.get('kalman_sigma_m',0),.55,4.5)) if not track.get('frozen_lead') and 'kalman_sigma_m' in r else 0.
            body['source_visible']=body['id'] in visible;body['ghost']=not body['source_visible']
            q=coordinates(report['road'],body['position_W'])
            if body['collision_cap_m'] and r.get('camera_id')=='front' and lane_at(report['road'],float(q[0]),float(q[1])) is None and not inside_junction(report['road'],body['position_W'],5.) and np.linalg.norm(np.asarray(body['position_W'][:2])-np.asarray(state['ego_position_W'][:2]))>8.:
                raw=np.array(r.get('unrefined_position_W',r['position_W'])[:2]);ep=np.array([np.interp(r['time_s'],ts,ego[:,k]) for k in range(2)]);ray=raw-ep
                if np.linalg.norm(ray)>1:body['collision_axis_W']=(ray/np.linalg.norm(ray)).tolist()
    states,omitted=omit_conflicting_predictions(states,report['vehicle_dimensions_m']);states,review=constrain(states,report['road'],report['vehicle_dimensions_m'])
    out=dict(review=review,omissions=omitted,scope='diagnostic conditional-sigma ray displacement and source-frame visibility; no independent 3D ground truth')
    a.output.write_text(json.dumps(out,indent=2));print('AFTER',review['after'],'maxmove',review['max_correction_m'],'omitted',omitted['omitted_count'])
