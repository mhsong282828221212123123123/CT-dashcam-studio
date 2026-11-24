"""Check that rear evidence cannot move or relabel primary-camera world tracks."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline


def validate(root,output):
    root=Path(root);output=Path(output)
    if output.exists():raise FileExistsError(output)
    raw=(root/'observations.jsonl').read_bytes()
    rows=[json.loads(line) for line in raw.decode('utf-8').splitlines()]
    manifest=json.loads((root/'manifest.json').read_text());base=FusedSceneTimeline(rows,manifest)
    cases={}
    for mode in ('remove_rear','perturb_rear','rear_only'):
        changed=copy.deepcopy(rows)
        for i,row in enumerate(changed):
            for camera,data in row['cameras'].items():
                if mode=='rear_only' and camera!='back':data['objects']=[]
                if camera!='back':continue
                if mode=='remove_rear':data['objects']=[]
                elif mode=='perturb_rear':
                    for obj in data['objects']:
                        obj['bbox'][0]+=80*np.sin(i);obj['bbox'][1]+=100*np.cos(i)
                        obj['confidence']=.99 if i%2 else .26
                        obj['track_id']='back:changed-'+str(i%4)
        trial=FusedSceneTimeline(changed,manifest)
        if mode=='rear_only':
            count=sum(len(s.actors) for s in trial.scenes)
            cases[mode]={'spawned_actors':count,'passed':count==0};continue
        max_delta=0.;mismatches=0
        for before,after in zip(base.scenes,trial.scenes):
            a={x.key:x for x in before.actors};b={x.key:x for x in after.actors}
            if set(a)!=set(b):mismatches+=1
            for key in set(a)&set(b):
                max_delta=max(max_delta,float(np.hypot(a[key].x-b[key].x,a[key].z-b[key].z)))
                if a[key].kind!=b[key].kind or a[key].tracking_state!=b[key].tracking_state:mismatches+=1
        cases[mode]={'max_position_delta':max_delta,'identity_or_state_mismatches':mismatches,
                     'passed':max_delta<1e-9 and mismatches==0}
    # Positive control: primary measurements must still affect the reconstruction.
    changed=copy.deepcopy(rows)
    for row in changed:
        for name,camera in row['cameras'].items():
            if name!='back':
                for o in camera.get('objects',[]):o['bbox'][0]+=40
    trial=FusedSceneTimeline(changed,manifest);deltas=[]
    for a,b in zip(base.scenes,trial.scenes):
        aa={source:x for x in a.actors for source in x.sources if not source.startswith(('back:','review:back:'))}
        bb={source:x for x in b.actors for source in x.sources if not source.startswith(('back:','review:back:'))}
        deltas.extend(float(np.hypot(aa[s].x-bb[s].x,aa[s].z-bb[s].z)) for s in set(aa)&set(bb))
    cases['primary_positive_control']={'max_position_delta':max(deltas,default=0.),'passed':max(deltas,default=0.)>.01}
    result={'input_sha256':hashlib.sha256(raw).hexdigest(),'cases':cases,
            'policy':'Rear associates only; pose, size, confidence and authoritative identity use front/side cameras.',
            'passed':all(x['passed'] for x in cases.values())}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--result',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();r=validate(a.result,a.output);print(json.dumps(r,indent=2));sys.exit(0 if r['passed'] else 1)
