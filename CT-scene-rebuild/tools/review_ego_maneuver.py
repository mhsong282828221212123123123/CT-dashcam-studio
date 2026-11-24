"""Verify correction is bounded to ego lane changes and normal output is exact."""
import json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify


out=Path('runs/ego-maneuver-review-001');out.mkdir(exist_ok=True);results={}
for name in ['oct21','sep17','oct02']:
    old=Path(f'runs/{name}-pocket-003');new=Path(f'runs/{name}-ego-maneuver-001');verify(new)
    load=lambda p:json.loads(p.read_text(encoding='utf-8'))
    compensation=load(new/'ego-maneuver-compensation.json');windows=compensation['windows']
    old_actors={a['global_id']:a for a in load(old/'actors.json')};new_actors=load(new/'actors.json')
    active=lambda t:any(w['start_s']<=t<=w['end_s'] for w in windows)
    checks=dict(ego_lane_state_exact=(old/'ego-lane-state.json').read_bytes()==(new/'ego-lane-state.json').read_bytes(),
                road_exact=load(old/'report.json')['road']==load(new/'report.json')['road'],
                identity_edges_exact=(old/'association-review.json').read_bytes()==(new/'association-review.json').read_bytes(),
                actor_ids_exact=set(old_actors)=={a['global_id'] for a in new_actors},
                no_observed_ghost_conflicts=not load(new/'observation-world-audit.json')['observed_ghost_conflicts'],
                full_export_decoded=load(new/'report.json')['all_frames_decoded'])
    untouched=True;longitudinal=True
    for a in new_actors:
        old_rows=old_actors[a['global_id']]['samples'];assert len(old_rows)==len(a['samples'])
        for r,s in zip(a['samples'],old_rows):
            longitudinal &= r['position_W'][0]==s['position_W'][0] and r['position_W'][2]==s['position_W'][2]
            if not active(r['time_s']):untouched &= r==s
    checks.update(outside_window_samples_exact=bool(untouched),all_longitudinal_and_height_coordinates_exact=bool(longitudinal))
    old_states=[json.loads(l) for l in (old/'render-states.jsonl').read_text().splitlines()]
    new_states=[json.loads(l) for l in (new/'render-states.jsonl').read_text().splitlines()]
    checks['outside_window_render_actor_states_exact']=all(a['actors']==b['actors'] and a['visible_unlocalized_ids']==b['visible_unlocalized_ids'] for a,b in zip(old_states,new_states) if not active(b['time_s']))
    checks['camera_ego_follow_outside_window']=all(s['camera_view']['mode']=='ego_follow' for s in new_states if not active(s['time_s']))
    if name=='oct02':checks['long_curve_false_event_not_compensated']=not windows
    else:checks['one_isolated_lane_change_window']=len(windows)==1
    if name=='oct21':
        lead=next(r for r in compensation['actors'] if r['global_id']=='front-0060')
        checks['lead_lateral_swing_reduced']=lead['lateral_range_before_m']>2 and lead['lateral_range_after_m']<.25
        results[name]=dict(checks=checks,windows=windows,lead_range_before_m=lead['lateral_range_before_m'],lead_range_after_m=lead['lateral_range_after_m'])
    else:results[name]=dict(checks=checks,windows=windows)
    assert all(checks.values()),(name,checks)
(out/'review.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
