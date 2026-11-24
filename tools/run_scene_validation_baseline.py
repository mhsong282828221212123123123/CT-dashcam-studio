"""Run frozen multi-clip baseline; coverage is diagnostic, not ground truth."""
import collections
import json
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import SceneDetector
from core.scene_pipeline import analyze
from core.scene_fusion import FusedSceneTimeline,default_profile


def run(root):
    root=Path(root)
    catalog=json.loads((root/'catalog'/'catalog.json').read_text(encoding='utf-8'))
    # Selected by visual scene category, before running model predictions.
    categories={1:['day','multilane'],3:['day','urban'],5:['day','multilane'],
        6:['day','intersection','occlusion'],7:['night','intersection'],8:['day','truck'],
        9:['day','truck'],10:['night','intersection'],11:['day','dense_traffic','bus'],
        13:['day','close_vehicle'],19:['night','multilane'],21:['day','intersection'],
        26:['indoor','parking'],31:['day','intersection','occlusion'],
        33:['pedestrian'],39:['day','open_road'],40:['day','motorcycle','dense_traffic'],
        42:['day','curve','adjacent_vehicle'],46:['day','close_vehicle'],
        47:['day','truck','adjacent_vehicle'],53:['night','intersection']}
    cases=[dict(catalog['review_queue'][i],case_index=i,visual_categories=tags)
           for i,tags in categories.items()]
    (root/'baseline-cases.json').write_text(json.dumps(cases,ensure_ascii=False,indent=2),encoding='utf-8')
    detector=SceneDetector('../scene-models/yolo11x.onnx',.25,640,
                           include_traffic_lights=True,backend='directml')
    reports=[]
    legacy_profile=default_profile()
    legacy_profile['cameras']['left_repeater']['horizon']=585.
    legacy_profile['cameras']['right_repeater']['horizon']=545.
    for number,case in enumerate(cases):
        target=root/'baseline'/case['id']
        print(f"CASE {number+1}/{len(cases)} {case['id']} {case['split']}",flush=True)
        if not (target/'manifest.json').exists():
            start=min(10.,max(0.,case['duration_s']-10.))
            for attempt in range(3):
                try:
                    analyze(case['source'],target,sample_hz=10,duration=10,start=start,detector=detector)
                    break
                except PermissionError:
                    if attempt==2:raise
                    time.sleep(1)
        manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
        rows=[json.loads(s) for s in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
        timeline=FusedSceneTimeline(rows,manifest,profile=legacy_profile)
        rejected=collections.Counter(e['reason'] for a in timeline.audit for e in a['rejected'])
        by_camera={}
        for camera in manifest['cameras']:
            total=placed=0
            for row,scene in zip(rows,timeline.scenes):
                ids={o['track_id'] for o in row['cameras'][camera].get('objects',[])
                     if o.get('track_status')=='confirmed'}
                supported={s for a in scene.actors for s in a.sources}
                total+=len(ids);placed+=len(ids&supported)
            by_camera[camera]=dict(confirmed_observations=total,placed_observations=placed)
        report=dict(id=case['id'],split=case['split'],categories=case['visual_categories'],
                    cameras=by_camera,rejection_reasons=dict(rejected),
                    road_frames=sum(bool(s.road_surface) for s in timeline.scenes),frames=len(rows),
                    ground_truth_evaluated=False,acceptance='NOT EVALUATED')
        (target/'fusion-audit.json').write_text(json.dumps(timeline.audit,ensure_ascii=False,indent=2),encoding='utf-8')
        reports.append(report)
        (root/'baseline-report.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False),flush=True)


if __name__=='__main__':
    run(sys.argv[1])
