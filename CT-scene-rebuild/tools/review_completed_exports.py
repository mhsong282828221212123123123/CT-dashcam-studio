"""Audit this bounded validation run after all three frozen suites finish."""
import argparse,json,time,subprocess,sys
from pathlib import Path
from ct_scene.artifacts import canonical,verify
from ct_scene.quality_review import review

p=argparse.ArgumentParser();p.add_argument('--branch',type=Path,required=True);p.add_argument('--four',type=Path,required=True);p.add_argument('--six',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(exist_ok=False)
roots=[a.branch,a.four,a.six];start=time.monotonic();previous=None
while True:
    records=[json.loads((r/'suite.json').read_text()) if (r/'suite.json').exists() else {} for r in roots]
    counts=[len(r.get('results',[])) for r in records]
    if counts!=previous:print('Completed exports',counts,flush=True);previous=counts
    if all(r.get('status')=='finished' for r in records):break
    if time.monotonic()-start>1800:raise TimeoutError('Current validation did not finish within 30 minutes')
    time.sleep(10)
rows=[]
for record,expected_count in zip(records,[4,4,6]):
    ids=[r.get('name',r.get('source_id')) for r in record['results']]
    if len(ids)!=expected_count or len(set(ids))!=expected_count:raise ValueError('Incomplete or duplicate legacy suite; use explicit case manifest')
for root,record in zip(roots,records):
    for item in record['results']:
        key=item.get('name',item.get('source_id'));scene=root/key if root==a.branch else root/key/'scene'
        row=dict(suite=root.name,source_id=key,returncode=item['returncode'])
        if item['returncode']==0:
            manifest=verify(scene);target=a.output/root.name/key
            numeric=review(scene,target)
            row.update(artifact_id=manifest['artifact_id'],numerical_gate=numeric['numerical_gate'],failure_count=numeric['failure_count'],frames=numeric['frames_decoded'],last_five_seconds_audited=numeric['last_five_seconds_audited'],video=str((scene/'surrounding-world.mp4').resolve()))
        rows.append(row)
result=subprocess.run([sys.executable,'tools/review_current_source_witnesses.py','--branch',str(a.branch),'--four',str(a.four),'--output',str(a.output/'source-witnesses')],capture_output=True,text=True)
(a.output/'source-witnesses.log').write_text(result.stdout+result.stderr,encoding='utf-8')
report=dict(exports=rows,engine_sha256=[r['engine_sha256'] for r in records],engine_unchanged=all(r.get('engine_unchanged') for r in records),same_engine=len({r['engine_sha256'] for r in records})==1,selected_source_witnesses_passed=result.returncode==0,
            numerical_exports_passed=all(r.get('numerical_gate')=='PASS' for r in rows),source_quality_scope='Selected development witnesses only; new clips require separate raw-source review; not exhaustive recall or independent new-date holdout')
(a.output/'review.json').write_bytes(canonical(report));print(json.dumps(report),flush=True)
raise SystemExit(0 if rows and report['numerical_exports_passed'] and report['selected_source_witnesses_passed'] and report['same_engine'] and report['engine_unchanged'] else 2)
