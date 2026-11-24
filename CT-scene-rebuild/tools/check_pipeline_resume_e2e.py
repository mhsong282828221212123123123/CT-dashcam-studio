"""Interrupt a REAL analysis child after published stages, then resume same code."""
import json,os,shutil,subprocess,sys,time
from pathlib import Path
from ct_scene.artifacts import verify,canonical,digest

out=Path('runs/resume-e2e-001').resolve();out.mkdir(exist_ok=False);f=out/'frozen'
shutil.copytree('src/ct_scene',f/'src/ct_scene',ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree('configs',f/'configs');shutil.copytree('E:/tesla/project/fsd-sedan-3d/models',f/'assets')
selection=json.loads(Path('runs/new-date-selection-002/selection.json').read_text())['selected']
source=next(s for s in selection if s['source_id']=='2026-09-20_21-59-02')['source_files']['front'];root=Path.cwd().resolve()
cmd=[sys.executable,'-m','ct_scene','analyze','--front',source,'--output',str(out/'analysis'),
     '--detector',str(root.parent/'scene-models/yolo11x.onnx'),'--road-model',str(root.parent/'scene-models/model-review/yolopv2-roads.onnx'),
     '--lens-config',str(f/'configs/multicam-optical-candidates.json'),'--prototype-config',str(f/'configs/surrounding-prototype.json'),'--assets-root',str(f/'assets')]
env=os.environ.copy();env['PYTHONPATH']=str(f/'src');env['PYTHONUNBUFFERED']='1'
child=subprocess.Popen(cmd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8')
lines=[];interrupted=False
try:
    for line in child.stdout:
        lines.append(line)
        if '"stage": "detections", "state": "starting"' in line:
            # Allow the actual worker to enter model initialization; terminate
            # only the Popen child this script owns, never a computed PID list.
            time.sleep(.5);child.terminate();child.wait(timeout=20);interrupted=True;break
finally:
    if child.poll() is None:child.terminate();child.wait(timeout=20)
(out/'interrupted.log').write_text(''.join(lines),encoding='utf-8')
if not interrupted:raise RuntimeError('Real analysis did not reach the intended interruption stage')
before={name:verify(out/'analysis'/name)['artifact_id'] for name in ['source','telemetry']}
print('REAL_CHILD_INTERRUPTED_AFTER_PUBLISHED_STAGES',before,flush=True)
with (out/'resumed.log').open('w',encoding='utf-8') as stream:r=subprocess.run(cmd,env=env,stdout=stream,stderr=subprocess.STDOUT)
state=json.loads((out/'analysis/pipeline.json').read_text());after={name:verify(out/'analysis'/name)['artifact_id'] for name in before}
log=(out/'resumed.log').read_text();checks=dict(real_interruption=interrupted,completed=r.returncode==0 and state['status']=='complete',
                                            published_artifacts_identical=before==after,verified_cache_used=log.count('verified_cache')>=2,
                                            full_export=verify(out/'analysis/scene')['stage']=='surrounding_world_prototype')
result=dict(checks=checks,status='PASS' if all(checks.values()) else 'FAIL',source=source,
            source_relation_quality='NOT_EVALUATED; lifecycle check only',interrupted_and_resumed_same_frozen_code=True)
(out/'review.json').write_bytes(canonical(result));print(json.dumps(result),flush=True)
