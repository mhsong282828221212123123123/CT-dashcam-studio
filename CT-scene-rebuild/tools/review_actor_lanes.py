"""Lane-keeping source fixtures and explicit failed generalization cases."""
import json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify


out=Path('runs/actor-lane-source-review-001');results={}
for name,old_name,new_name in [('oct21','oct21-tesla-dark-001','oct21-actor-lanes-001'),
                               ('sep17','sep17-handoff-002','sep17-actor-lanes-001')]:
    old,new=[Path('runs')/p for p in (old_name,new_name)];verify(new)
    load=lambda p:json.loads(p.read_text(encoding='utf-8'))
    before,after=[load(p/'actors.json') for p in (old,new)];by_old={a['global_id']:a for a in before}
    original_anchor=next(a for a in before if a['frozen_lead'])
    new_anchor=next(a for a in after if a['frozen_lead'])
    # Preserve the original front anchor, before later attached camera samples.
    original_rows=[r for r in original_anchor['samples'] if r['camera_id']=='front' and r['time_s']<=32.9906] if name=='oct21' else original_anchor['samples']
    new_rows={r['time_s']:r for r in new_anchor['samples']}
    checks=dict(ego_exact=(old/'ego-lane-state.json').read_bytes()==(new/'ego-lane-state.json').read_bytes(),
                road_exact=load(old/'report.json')['road']==load(new/'report.json')['road'],
                handoffs_exact=(old/'association-review.json').read_bytes()==(new/'association-review.json').read_bytes(),
                original_anchor_positions_exact=all(r['position_W']==new_rows[r['time_s']]['position_W'] for r in original_rows),
                original_ids_retained=set(by_old)=={a['global_id'] for a in after},
                no_observed_ghost_conflicts=not load(new/'observation-world-audit.json')['observed_ghost_conflicts'],
                full_export_decoded=load(new/'report.json')['all_frames_decoded'])
    checks['longitudinal_positions_unchanged']=all(np.allclose([r['position_W'][0] for r in a['samples']], [r['position_W'][0] for r in by_old[a['global_id']]['samples']],atol=1e-8,rtol=0) for a in after)
    if name=='oct21':
        states=load(new/'actor-lane-state.json')
        checks['source_reviewed_lane_keepers_have_no_events']=all(not states[k]['events'] for k in ['front-0002','front-0012'])
        boundaries=load(new/'report.json')['road']['boundaries_y'];slope=load(new/'report.json')['road']['heading_evidence']['slope']
        checks['lane_keepers_stay_in_original_lane']=all(boundaries[1]<=r['position_W'][1]-slope*r['position_W'][0]<=boundaries[0]
            for a in after if a['global_id'] in ['front-0002','front-0012'] for r in a['samples'] if 29<r['time_s']<32.8)
    assert all(checks.values()),(name,checks)
    results[name]=dict(status='PASS_FOR_REVIEWED_LANE_KEEPING_DEFECT',checks=checks,frames=load(new/'report.json')['frames'])

for name in ['aug30','oct02']:
    p=Path(f'runs/{name}-actor-lanes-001');ego=json.loads((p/'ego-lane-state.json').read_text())
    results[name]=dict(status='REJECTED_FOR_DELIVERY',reason='Curved road / pose-to-road discrepancy in straight road model; raw lane keeping must not be called a lane change',
                       ego_outside_samples=sum(r['lane_id'] is None for r in ego['records']),ego_samples=len(ego['records']),ego_events=ego['events'])
results['real_lane_change_positive_validation']='PENDING; synthetic continuous-change, camera-switch and observer-motion cases pass, no real positive claim'
(out/'review.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
