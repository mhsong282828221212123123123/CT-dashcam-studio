"""Estimate experimental horizon priors from development tracks only.

Representative car height is an explicit prior, not a metric calibration.
Each track and then each clip gets one vote, avoiding stationary-frame bias.
The profile is saved for A/B review; never silently installed as a default.
"""
import collections,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import default_profile,FusedSceneTimeline


def estimate(root):
    root=Path(root);cases=json.loads((root/'baseline-cases.json').read_text(encoding='utf-8'))
    groups=collections.defaultdict(lambda:collections.defaultdict(lambda:collections.defaultdict(list)))
    for case in cases:
        if case['split']!='development':continue
        path=root/'baseline'/case['id']/'observations.jsonl'
        if not path.exists():continue
        for line in path.read_text(encoding='utf-8').splitlines():
            row=json.loads(line)
            for camera,data in row['cameras'].items():
                if camera not in ('left_repeater','right_repeater'):continue
                objects=data.get('objects',[])
                for obj in objects:
                    x,y,w,h=obj['bbox']
                    if (obj['kind']!='car' or obj.get('track_status')!='confirmed'
                        or not 30<h<150 or not 1<w/h<2.5 or x<=20 or x+w>=1260):continue
                    if any(other is not obj and other['bbox'][0]<x+w/2<sum(other['bbox'][::2])
                           and other['bbox'][1]<y+h<sum(other['bbox'][1::2])-10 for other in objects):continue
                    groups[camera][case['id']][obj['track_id']].append(y+h-h*.9/1.65)
    profile=default_profile();support={}
    for camera,clips in groups.items():
        votes={clip:float(np.median([np.median(v) for v in tracks.values() if len(v)>=5]))
               for clip,tracks in clips.items() if any(len(v)>=5 for v in tracks.values())}
        if len(votes)<3:continue
        values=np.array(list(votes.values()));median=float(np.median(values))
        mad=float(np.median(abs(values-median)))
        support[camera]=dict(clip_votes=votes,median_px=median,mad_px=mad,
                             assumption='camera height .9m / representative car height 1.65m')
        if mad<25 and 400<median<580:
            profile['cameras'][camera]['horizon']=median
    profile.update(name='Development-track horizon prior',status='experimental_not_calibrated',
                   support=support,source_split='development')
    (root/'experimental-camera-prior.json').write_text(json.dumps(profile,indent=2),encoding='utf-8')
    print(json.dumps(support,indent=2),flush=True)
    output=[]
    for case in cases:
        target=root/'baseline'/case['id']
        if not (target/'manifest.json').exists():continue
        rows=[json.loads(s) for s in (target/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
        manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
        timeline=FusedSceneTimeline(rows,manifest,profile=profile)
        counts={}
        for camera in ('front','left_repeater','right_repeater'):
            total=placed=0
            for row,scene in zip(rows,timeline.scenes):
                ids={o['track_id'] for o in row['cameras'].get(camera,{}).get('objects',[]) if o.get('track_status')=='confirmed'}
                supported={s for actor in scene.actors for s in actor.sources}
                total+=len(ids);placed+=len(ids&supported)
            counts[camera]=dict(total=total,placed=placed)
        output.append(dict(id=case['id'],split=case['split'],counts=counts,
                           metric_accuracy='NOT EVALUATED'))
    (root/'experimental-camera-prior-coverage.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
    print('Compared',len(output),'clips',flush=True)


if __name__=='__main__':estimate(sys.argv[1])
