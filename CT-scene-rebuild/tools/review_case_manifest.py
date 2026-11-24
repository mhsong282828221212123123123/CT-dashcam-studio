"""Full-clock numerical review of an exact manifest; source acceptance stays separate."""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import verify,canonical,digest
from ct_scene.quality_review import review
from review_guards import require_cases,require_full_clock

p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
data=json.loads(a.manifest.read_text());require_cases([r['case_id'] for r in data['cases']],data['expected_case_ids'])
if data['status']!='finished' or not data.get('engine_unchanged') or any(r['state']!='complete' for r in data['cases']):raise ValueError('Manifest is incomplete')
engine=Path(data['engine_root'])/'src/ct_scene'
if digest(b''.join(p.read_bytes() for p in sorted(engine.glob('*.py'))))!=data['engine_sha256']:raise ValueError('Frozen engine mismatch')
a.output.mkdir(exist_ok=False);rows=[]
for case in data['cases']:
    row=dict(case_id=case['case_id'])
    try:
        scene=Path(case['scene']);m=verify(scene);bm=verify(Path(case['base']))
        if m['artifact_id']!=case['artifact_id']:raise ValueError('Scene identity changed')
        if not any(r['artifact_id']==bm['artifact_id'] and r['role']=='frozen_ego_lead' for r in m['inputs']):raise ValueError('Scene/base mismatch')
        report=json.loads((scene/'report.json').read_text())
        if report.get('render_is_preview'):raise ValueError('Preview cannot pass full-source review')
        states=[json.loads(s) for s in (scene/'render-states.jsonl').read_text().splitlines()]
        numeric=review(scene,a.output/'numerical'/case['case_id'])
        clock=require_full_clock(states,json.loads((Path(case['base'])/'report.json').read_text()),numeric['frames_decoded'])
        row.update(numerical_gate=numeric['numerical_gate'],frames=numeric['frames_decoded'],clock=clock)
    except (ValueError,OSError,KeyError) as error:row.update(numerical_gate='FAIL',error=str(error))
    rows.append(row)
passed=bool(rows) and all(r['numerical_gate']=='PASS' for r in rows)
result=dict(numerical_gate='PASS' if passed else 'FAIL',source_gate='NOT_EVALUATED',overall_gate='REVIEW_REQUIRED' if passed else 'FAIL',cases=rows,engine_sha256=data['engine_sha256'],reviewer_sha256=digest(Path(__file__).read_bytes()+Path(__file__).with_name('review_guards.py').read_bytes()))
(a.output/'review.json').write_bytes(canonical(result));print(json.dumps(dict(numerical_gate=result['numerical_gate'],overall_gate=result['overall_gate'],cases=len(rows))))
raise SystemExit(0 if passed else 2)
