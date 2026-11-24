"""Recover two rejected CLI invocations, retaining their original failure logs."""
import json,os,subprocess,sys,time
from pathlib import Path
from ct_scene.artifacts import canonical

suite=Path('runs/new-date-suite-003').resolve();m=json.loads((suite/'suite.json').read_text())
if m['status']!='finished':raise ValueError('Wait for the original suite to finish')
env=os.environ.copy();env['PYTHONPATH']=str(suite/'frozen/src');env['PYTHONUNBUFFERED']='1'
records=[]
for original in m['results']:
    sid=original['source_id']
    if original['returncode']!=2 or sid not in m['osm_caches']:continue
    log=(suite/(sid+'.log')).read_text()
    if 'unrecognized arguments: --osm-cache' not in log:raise ValueError('Unexpected failure')
    source=next(s for s in m['sources'] if s['source_id']==sid);f=suite/'frozen';root=Path.cwd().resolve()
    cmd=[sys.executable,'-m','ct_scene','analyze','--front',source['source_files']['front'],'--output',str(suite/sid),
         '--detector',str(root.parent/'scene-models/yolo11x.onnx'),'--road-model',str(root.parent/'scene-models/model-review/yolopv2-roads.onnx'),
         '--lens-config',str(f/'configs/multicam-optical-candidates.json'),'--prototype-config',str(f/'configs/surrounding-prototype.json'),
         '--assets-root',str(f/'assets'),'--osm',m['osm_caches'][sid]['path']]
    print('RETRY',sid,flush=True);start=time.perf_counter()
    with (suite/(sid+'.cli-retry.log')).open('w') as out:r=subprocess.run(cmd,env=env,stdout=out,stderr=subprocess.STDOUT)
    records.append(dict(source_id=sid,original_returncode=2,retry_returncode=r.returncode,elapsed_s=time.perf_counter()-start))
    (suite/'cli-retry.json').write_bytes(canonical(records));print(records[-1],flush=True)
