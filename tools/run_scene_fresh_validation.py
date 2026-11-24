"""Frozen source-selected independent 35-second cases, including a negative case."""
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import SceneDetector
from core.scene_pipeline import analyze
from core.scene_fusion import FusedSceneTimeline

root=Path(sys.argv[1])
catalog=json.loads((root/'catalog/catalog.json').read_text(encoding='utf-8'))
selected={'2026-08-18_07-38-46':['day','dense_traffic'],
          '2026-08-19_17-16-37':['day','intersection','cross_traffic'],
          '2026-08-31_12-57-14':['rain','curve','narrow_road'],
          '2026-10-16_20-54-27':['night','parking','negative_road_case']}
cases=[dict(c,validation_categories=selected[c['id']]) for c in catalog['inventory'] if c['id'] in selected]
assert len(cases)==len(selected)
out=root/'fresh-validation';out.mkdir(exist_ok=True)
configuration={'start_s':1.,'duration_s':35.,'sample_hz':10,'model':'yolo11x.onnx',
               'source_selection':'manual source review before running predictions','acceptance':False,
               'cases':cases,'code_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in sorted(Path('core').glob('scene*.py'))}}
freeze=out/'run-configuration.json'
if freeze.exists():
    saved=json.loads(freeze.read_text(encoding='utf-8'))
    if saved!=configuration:raise RuntimeError('Validation configuration changed; use a new output root/run')
else:freeze.write_text(json.dumps(configuration,ensure_ascii=False,indent=2),encoding='utf-8')
detector=SceneDetector('../scene-models/yolo11x.onnx',.25,640,include_traffic_lights=True,backend='directml')
report=[]
for case in cases:
    target=out/'analysis'/case['id']
    print('START',case['id'],flush=True)
    if not (target/'manifest.json').exists():
        analyze(case['source'],target,sample_hz=10,duration=35,start=1,detector=detector)
    manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
    rows=[json.loads(x) for x in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    timeline=FusedSceneTimeline(rows,manifest)
    (target/'fusion-audit.json').write_text(json.dumps(timeline.audit),encoding='utf-8')
    report.append(dict(id=case['id'],frames=len(rows),stitched_handoffs=timeline.stitched_handoffs,
        truck_shape_priors=timeline.truck_shape_priors,manual_review_complete=False,acceptance=False))
    (out/'run-report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('COMPLETE',case['id'],len(rows),flush=True)
