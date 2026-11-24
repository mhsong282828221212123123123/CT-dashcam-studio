"""Experimental plumb-line lens proposal from masked rear video edges.

Straight-edge hypotheses require visual review; this is not metric calibration.
"""
import json,sys
from pathlib import Path
import cv2,numpy as np
from scipy.optimize import minimize_scalar
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader

root=Path('../scene-results');out=root/'rear-rectification';out.mkdir(exist_ok=True)
cfg=json.loads((root/'quality-rebuild/fresh-validation/run-configuration.json').read_text())
cases=[]
for c in cfg['cases']:
    if c['id'] not in ('2026-08-18_07-38-46','2026-08-19_17-16-37'):continue
    cases.append((c['id'],Path(c['source']).parent,root/'quality-rebuild/fresh-validation/analysis'/c['id'],'train'))
cases.append(('2026-09-05',Path('E:/tesla/a2/TeslaCam/SavedClips/2026-09-05_14-56-11'),root/'directml-upgrade/high-v11x-analysis','validation'))
for c in json.loads((root/'quality-rebuild/baseline-cases.json').read_text()):
    cases.append((c['id'],Path(c['source']).parent,root/'quality-rebuild/baseline'/c['id'],
                  'train' if c['split']=='development' else 'validation'))
lines=[];reviews=[]
for key,source,result,split in cases:
    manifest=json.loads((result/'manifest.json').read_text());rows=[json.loads(s) for s in (result/'observations.jsonl').read_text().splitlines()]
    reader=CameraReader(source/manifest['cameras']['back']['file'])
    try:
        for t in np.arange(rows[0]['time_s']+1,rows[-1]['time_s'],2):
            row=min(rows,key=lambda r:abs(r['time_s']-t));_,frame=reader.read_at(row['time_s'])
            if frame is None:continue
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);h,w=gray.shape
            if (w,h)!=(1280,960):raise ValueError('Review resolution mismatch')
            edges=cv2.Canny(gray,50,120);edges[:int(h*.17)]=0;edges[int(h*.72):]=0
            for obj in row['cameras']['back'].get('objects',[]):
                x,y,bw,bh=map(round,obj['bbox']);cv2.rectangle(edges,(x-8,y-8),(x+bw+8,y+bh+8),0,-1)
            contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
            review=frame.copy();count=0
            for contour in contours:
                p=contour[:,0,:].astype(float);span=np.ptp(p,axis=0)
                if len(p)<80 or span[1]<60 or span[0]>span[1]*.45:continue
                if np.linalg.norm(p[0]-p[-1])<span[1]*.5:
                    # Canny traces both sides of a thin edge in a loop. Reduce
                    # each image row to one location to avoid double weighting.
                    p=np.array([[np.median(p[p[:,1]==y,0]),y] for y in np.unique(p[:,1])])
                if len(p)<40:continue
                p=p[np.linspace(0,len(p)-1,min(100,len(p))).astype(int)]
                lines.append(dict(clip=key,time_s=row['time_s'],split=split,points=p.tolist()))
                cv2.polylines(review,[p.astype(np.int32)],False,(0,255,0),2);count+=1
            if count:
                cv2.putText(review,f'{key} {t}s {count} edge hypotheses',(10,30),0,.7,(0,255,255),2)
                cv2.imencode('.jpg',cv2.resize(review,(640,480)))[1].tofile(str(out/f'edges-{key}-{t}.jpg'))
    finally:reader.close()

def error(line,k):
    p=(np.array(line['points'])-[640,480])/650
    denominator=1+k*np.sum(p*p,axis=1)
    if np.min(denominator)<.2:return 1e6
    original_span=float(np.linalg.norm(np.ptp(p,axis=0)))
    p=p/denominator[:,None];span=float(np.linalg.norm(np.ptp(p,axis=0)));p-=p.mean(0)
    _,_,v=np.linalg.svd(p,full_matrices=False)
    return float(np.sqrt(np.mean((p@v[-1])**2))*650*original_span/max(span,1e-9))

train=[v for v in lines if v['split']=='train'];validation=[v for v in lines if v['split']=='validation']
if len(train)<20 or len(validation)<10:raise RuntimeError(f'Insufficient edge hypotheses: {len(train)} train / {len(validation)} validation')
def balanced(items,k):
    clips=sorted({v['clip'] for v in items})
    return float(np.median([np.median([error(v,k) for v in items if v['clip']==clip]) for clip in clips]))
cost=lambda k:balanced(train,k)
fit=minimize_scalar(cost,bounds=(-.45,.25),method='bounded');k=float(fit.x)
report=dict(model='division1',coefficient=k,train_lines=len(train),validation_lines=len(validation),
    train_before=cost(0),train_after=cost(k),
    validation_before=balanced(validation,0),
    validation_after=balanced(validation,k),
    status='proposal_only_requires_visual_review',limitations='Unverified straight-edge hypotheses; fixed centre/focal gauge; no metric accuracy')
report['accepted']=bool(-.43<k<.23 and report['validation_after']<report['validation_before']*.85)
report['status']='proposal_requires_visual_review' if report['accepted'] else 'rejected_not_applied'
report['rejection_reason']='Boundary optimum and/or insufficient independent straightness improvement' if not report['accepted'] else None
(out/'edge-hypotheses.json').write_text(json.dumps(lines,indent=2))
(out/'lens-estimate-report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
