"""Raw-reviewed Aug26 bus handoff, including unobserved vs observed presence."""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import canonical,verify
from review_guards import require_window

p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--opposing-scene',type=Path)
a=p.parse_args();manifest=verify(a.scene);a.output.mkdir(exist_ok=False)
if not any(r['role']=='source' and r['artifact_id']=='aca9855ded75af90a8cdcf45f998313c68095f559879131535695e40f64435f9' for r in manifest['inputs']):raise ValueError('Bus source witness belongs to a different source package')
actors=json.loads((a.scene/'actors.json').read_text())
bus=next((r for r in actors if {'front-0001','left_repeater-0016'}<=set(r['local_ids'])),None)
states=[json.loads(r) for r in (a.scene/'render-states.jsonl').read_text().splitlines()]
require_window(states,12.85,24.1)
window=[s for s in states if 12.85<=s['time_s']<=24.1]
rows=[next((r for r in s['actors'] if bus and r['id']==bus['global_id']),None) for s in window]
checks=dict(same_raw_bus_identity_joined=bool(bus),
            other_coobserved_bus_not_joined=bool(bus and 'right_repeater-0001' not in bus['local_ids']),
            complete_side_interval_in_world=bool(rows and all(r is not None for r in rows)),
            source_visible_side_observations_are_solid=bool(rows and all(r and not r['ghost'] for r in rows)),
            bus_model_preserved=bool(bus and bus['model']=='bus'))
if a.opposing_scene:
    manifest=verify(a.opposing_scene)
    if not any(r['role']=='source' and r['artifact_id']=='5247c64fef9f9ae0c14f396e5633119722a8a83ae026405325912ce5057c9395' for r in manifest['inputs']):raise ValueError('Opposing witness source mismatch')
    other=json.loads((a.opposing_scene/'actors.json').read_text());ids={k for r in other for k in r['local_ids']}
    median=json.loads((a.opposing_scene/'image-median-review.json').read_text())
    checks['raw_opposing_mercedes_and_side_handoff_excluded']=not ids.intersection({'front-0016','left_repeater-0003'})
    checks['exclusion_supported_by_raw_yellow_median']=median.get('front-0016',{}).get('excluded') is True
result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,
            source_evidence='runs/correction-bus-handoff-source-001.jpg',
            opposing_source_evidence='runs/scope-handoff-oct22-additional-source-003.jpg' if a.opposing_scene else None,
            scope='Selected raw-reviewed bus retention and optional opposite-car exclusion; not raster visibility or general identity accuracy')
(a.output/'review.json').write_bytes(canonical(result));print(json.dumps(result));raise SystemExit(0 if result['status']=='PASS' else 2)
