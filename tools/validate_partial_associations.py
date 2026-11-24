"""Adversarial replay of one partial camera observation; not population accuracy."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline


def validate(result,output,camera,source):
    root=Path(result);output=Path(output)
    if output.exists():raise FileExistsError(output)
    raw=(root/'observations.jsonl').read_bytes()
    rows=[json.loads(line) for line in raw.decode('utf-8').splitlines()]
    manifest=json.loads((root/'manifest.json').read_text())
    baseline=FusedSceneTimeline(rows,manifest)
    supported=sum(any(source in a.sources and len(a.sources)>1 for a in s.actors) for s in baseline.scenes)
    cases={}
    for mode in ('wrong_region','out_of_sync','incompatible_class','isolated_flashes'):
        changed=copy.deepcopy(rows)
        for i,row in enumerate(changed):
            observation=row['cameras'][camera]
            if mode=='out_of_sync':observation['source_time_s']+=.2
            if mode=='isolated_flashes' and i%3:
                observation['objects']=[o for o in observation['objects'] if o['track_id']!=source]
            for obj in observation['objects']:
                if obj['track_id']!=source:continue
                if mode=='wrong_region':
                    x,y,w,h=obj['bbox'];obj['bbox']=[manifest['cameras'][camera]['width']-x-w,y,w,h]
                if mode=='incompatible_class':obj['kind']='person'
        timeline=FusedSceneTimeline(changed,manifest)
        merges=sum(any(source in a.sources and len(a.sources)>1 for a in s.actors) for s in timeline.scenes)
        duplicates=sum(len(s.actors)!=len({a.key for a in s.actors}) for s in timeline.scenes)
        cases[mode]={'unexpected_merge_frames':merges,'duplicate_id_frames':duplicates,
                     'passed':merges==0 and duplicates==0}
    report={'input_sha256':hashlib.sha256(raw).hexdigest(),'source':source,
        'baseline_supported_frames':supported,'frame_count':len(rows),'cases':cases,
        'scope':'Single reviewed development clip with deliberate perturbations; not independent identity precision/recall.',
        'passed':supported>0 and all(c['passed'] for c in cases.values())}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',required=True);p.add_argument('--output',required=True)
    p.add_argument('--camera',default='back');p.add_argument('--source',default='review:back:B-partial')
    a=p.parse_args();report=validate(a.result,a.output,a.camera,a.source)
    print(json.dumps(report,indent=2));sys.exit(0 if report['passed'] else 1)
