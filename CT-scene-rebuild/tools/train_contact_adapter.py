"""Fit new local supervised heads; select on validation, evaluate test once."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from ct_scene.contact_learning import (
    guard_splits,contact_target,metadata,rich_metadata,flip_record,
    flip_contact_uv,flip_direction_index,ridge,metrics,CAMERAS,DIRECTIONS
)

p=argparse.ArgumentParser();p.add_argument('--annotations',type=Path,required=True);p.add_argument('--features',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
records=json.loads(a.annotations.read_text())['records'];guard_splits(records)
provenance=json.loads((a.features/'provenance.json').read_text())
if provenance['annotations_sha256']!=hashlib.sha256(a.annotations.read_bytes()).hexdigest():raise ValueError('Feature provenance/annotation mismatch; re-extract with these labels')
f=np.load(a.features/'features.npz');assert list(f['ids'])==[r['id'] for r in records]
train=np.array([r['split']=='train' for r in records]);validation=np.array([r['split']=='validation' for r in records]);test=np.array([r['split']=='test' for r in records]);features=f['features'].astype(float)
contact=np.array([contact_target(r) if contact_target(r) is not None else [np.nan,np.nan] for r in records]);visible=np.isfinite(contact).all(1);direction=np.array([DIRECTIONS.index(r['direction']) if r['direction'] in DIRECTIONS else -1 for r in records]);results={};weights={};selected={}
use_v3 = provenance.get('encoder_version') == 'v3' and 'features_flip' in f
adapter_config = None

if use_v3:
    features_flip=f['features_flip'].astype(float)
    train_feats=np.vstack([features[train],features_flip[train]])
    mean=train_feats.mean(0);scale=np.maximum(train_feats.std(0),.1)
    zo=np.clip((features-mean)/scale,-6,6);zf=np.clip((features_flip-mean)/scale,-6,6)
    c0,c1=2304,features.shape[1];d0,d1=0,2304
    adapter_config=dict(contact_slice=[c0,c1],direction_slice=[d0,d1],flip_tta=True,height_space_contact=True,rich_meta_contact=True)
    zco,zcf=zo[:,c0:c1]/np.sqrt(c1-c0),zf[:,c0:c1]/np.sqrt(c1-c0)
    zdo,zdf=zo[:,d0:d1]/np.sqrt(d1-d0),zf[:,d0:d1]/np.sqrt(d1-d0)
    mco=np.array([rich_metadata(r) for r in records]);mcf=np.array([rich_metadata(flip_record(r)) for r in records])
    mdo=np.array([metadata(r) for r in records]);mdf=np.array([metadata(flip_record(r)) for r in records])
    ratios=np.array([(r['bbox_xyxy'][2]-r['bbox_xyxy'][0])/(r['bbox_xyxy'][3]-r['bbox_xyxy'][1]) for r in records])
    # Contact head: height-space regression with flip aug + TTA, selected strictly on validation
    candidates=[]
    tr_c=train&visible;va_c=validation&visible;va_rows=[r for r,k in zip(records,va_c) if k]
    yo=np.c_[(contact[tr_c,0]-.5)*ratios[tr_c],contact[tr_c,1]-1.0]
    yf=np.c_[(1.0-contact[tr_c,0]-.5)*ratios[tr_c],contact[tr_c,1]-1.0]
    yc=np.vstack([yo,yf])
    for strength in [0.,.25,.5,1.,2.]:
        xo=np.c_[zco*strength,mco];xf=np.c_[zcf*strength,mcf];xc=np.vstack([xo[tr_c],xf[tr_c]])
        for alpha in [.1,.25,.5,1.,2.5,5.,10.,25.]:
            w,b=ridge(xc,yc,alpha)
            po=xo[va_c]@w+b;pf=xf[va_c]@w+b
            po=np.c_[po[:,0]/ratios[va_c]+.5,po[:,1]+1.0];pf=flip_contact_uv(np.c_[pf[:,0]/ratios[va_c]+.5,pf[:,1]+1.0])
            pred=0.5*(po+pf)
            score=metrics(pred,contact[va_c],va_rows)['mean_error_vehicle_height']
            candidates.append((score,strength,alpha,w,b))
    score,strength,alpha,w,b=min(candidates,key=lambda r:r[0])
    selected['contact']=dict(feature_strength=strength,alpha=alpha,validation_selection_score=score);weights['contact_w']=w;weights['contact_b']=b
    po=np.c_[zco*strength,mco]@w+b;pf=np.c_[zcf*strength,mcf]@w+b
    po=np.c_[po[:,0]/ratios+.5,po[:,1]+1.0];pf=flip_contact_uv(np.c_[pf[:,0]/ratios+.5,pf[:,1]+1.0])
    contact_pred=0.5*(po+pf)
    # Direction head: whole-car 2x2 features with flip aug + TTA, selected strictly on validation
    candidates=[]
    tr_d=train&(direction>=0);va_d=validation&(direction>=0)
    yd=np.vstack([np.eye(4)[direction[tr_d]],np.eye(4)[flip_direction_index(direction[tr_d])]])
    for strength in [0.,.25,.5,1.,2.]:
        xo=np.c_[zdo*strength,mdo];xf=np.c_[zdf*strength,mdf];xd=np.vstack([xo[tr_d],xf[tr_d]])
        for alpha in [.1,.25,.5,1.,2.5,5.,10.,25.]:
            w,b=ridge(xd,yd,alpha)
            po=xo[va_d]@w+b;pf=(xf[va_d]@w+b)[:,[0,1,3,2]];pred=0.5*(po+pf)
            score=float(np.mean((pred-np.eye(4)[direction[va_d]])**2))
            candidates.append((score,strength,alpha,w,b))
    score,strength,alpha,w,b=min(candidates,key=lambda r:r[0])
    selected['direction']=dict(feature_strength=strength,alpha=alpha,validation_selection_score=score);weights['direction_w']=w;weights['direction_b']=b
    po=np.c_[zdo*strength,mdo]@w+b;pf=(np.c_[zdf*strength,mdf]@w+b)[:,[0,1,3,2]]
    direction_pred=np.argmax(0.5*(po+pf),axis=1)
else:
    mean=features[train].mean(0);scale=np.maximum(features[train].std(0),.1);z=np.clip((features-mean)/scale,-6,6)/np.sqrt(features.shape[1]);meta=np.array([metadata(r) for r in records])
    # Separate selection is completed before the untouched final test is read.
    for name,target,eligible in [('contact',contact-np.array([.5,1]),visible),('direction',np.eye(4)[np.maximum(direction,0)],direction>=0)]:
        candidates=[]
        for strength in [0.,.25,1.]:
            x=np.c_[z*strength,meta]
            for alpha in [.01,.1,1.,10.]:
                w,b=ridge(x[train&eligible],target[train&eligible],alpha);pred=x[validation&eligible]@w+b
                score=metrics(pred,target[validation&eligible],[r for r,k in zip(records,validation&eligible) if k])['mean_error_vehicle_height'] if name=='contact' else float(np.mean((pred-target[validation&eligible])**2))
                candidates.append((score,strength,alpha,w,b))
        score,strength,alpha,w,b=min(candidates,key=lambda r:r[0]);selected[name]=dict(feature_strength=strength,alpha=alpha,validation_selection_score=score);weights[name+'_w']=w;weights[name+'_b']=b
        pred=np.c_[z*strength,meta]@w+b
        if name=='contact':contact_pred=pred+np.array([.5,1])
        else:direction_pred=np.argmax(pred,axis=1)

# Preserve every held-out prediction, including failures.
predictions=[]
for i,r in enumerate(records):predictions.append(dict(id=r['id'],split=r['split'],camera=r['camera'],contact_uv=contact_pred[i].tolist(),target_contact_uv=contact_target(r).tolist() if visible[i] else None,direction=DIRECTIONS[direction_pred[i]],target_direction=r['direction']))
for split,mask in [('train',train),('validation',validation),('test',test)]:
    chosen=mask&visible;rows=[r for r,k in zip(records,chosen) if k];baseline=np.tile([.5,1],(sum(chosen),1));counts={c:sum(r['camera']==c for r in rows) for c in CAMERAS}
    known=mask&(direction>=0);majority=np.bincount(direction[train&(direction>=0)],minlength=4).argmax()
    results[split]=dict(contact_model=metrics(contact_pred[chosen],contact[chosen],rows),bbox_bottom_baseline=metrics(baseline,contact[chosen],rows),contact_count_by_camera=counts,direction_samples=int(sum(known)),direction_accuracy=float(np.mean(direction_pred[known]==direction[known])),training_majority_baseline=float(np.mean(majority==direction[known])))
tr=results['test'];candidate=tr['contact_model']['mean_error_vehicle_height']<.9*tr['bbox_bottom_baseline']['mean_error_vehicle_height'] and tr['contact_model']['p90_error_vehicle_height']<=tr['bbox_bottom_baseline']['p90_error_vehicle_height']
report=dict(status='trained_experimental_image_heads',adapter_version='v3' if use_v3 else 'v1',adapter_config=adapter_config,runtime_adopted=False,metric_xyz_accuracy='NOT_EVALUATED',new_supervised_weights=True,full_backbone_finetuned=False,source_split_unit='recorded drive date/session',samples={s:sum(r['split']==s for r in records) for s in ['train','validation','test']},selected=selected,results=results,contact_2d_candidate_pass=bool(candidate),all_cameras_have_three_test_contacts=all(n>=3 for n in tr['contact_count_by_camera'].values()),direction_candidate_pass=tr['direction_accuracy']>=.7,annotation_provenance=dict(reviewers=sorted({r.get('reviewer','unspecified') for r in records}),independent_human_review=json.loads(a.annotations.read_text()).get('independent_human_review',False),source='visual image labels; no surveyed XYZ'),adoption_rule='Keep disabled: pilot labels only; 2D contact and coarse image direction do not validate metric 3D centre or world yaw',cuda=False,annotations_sha256=hashlib.sha256(a.annotations.read_bytes()).hexdigest(),encoder_sha256=hashlib.sha256((a.features/'contact-encoder.onnx').read_bytes()).hexdigest())
np.savez_compressed(a.output/'adapter.npz',feature_mean=mean,feature_scale=scale,**weights)
(a.output/'report.json').write_text(json.dumps(report,indent=2));(a.output/'predictions.json').write_text(json.dumps(predictions,indent=2));print(json.dumps(report))
