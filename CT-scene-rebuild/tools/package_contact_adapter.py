"""Package an experimental head with matching labels/features; never approve it."""
import argparse,hashlib,json,shutil
from pathlib import Path
import numpy as np
from ct_scene.contact_learning import guard_splits


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def package(trained,features,annotations,output):
    if output.exists():raise FileExistsError(output)
    data=json.loads(annotations.read_text());guard_splits(data['records'])
    report=json.loads((trained/'report.json').read_text())
    if report['annotations_sha256']!=sha(annotations):raise ValueError('Training report/annotation mismatch; use the exact training labels')
    if report['encoder_sha256']!=sha(features/'contact-encoder.onnx'):raise ValueError('Training report/encoder mismatch')
    provenance=json.loads((features/'provenance.json').read_text())
    if provenance['annotations_sha256']!=sha(annotations):raise ValueError('Feature provenance/annotation mismatch; extract features from these exact labels')
    encoded=np.load(features/'features.npz',allow_pickle=False)
    ids=[r['id'] for r in data['records']]
    if list(encoded['ids'])!=ids:raise ValueError('Feature sample order mismatch')
    predictions=json.loads((trained/'predictions.json').read_text())
    if [r['id'] for r in predictions]!=ids:raise ValueError('Prediction sample order mismatch')
    ver=report.get('adapter_version','v1')
    req_weights=['feature_mean','feature_scale','direction_w','direction_b'] if ver=='v4' else ['feature_mean','feature_scale','contact_w','contact_b','direction_w','direction_b']
    with np.load(trained/'adapter.npz',allow_pickle=False) as weights:
        for name in req_weights:
            if name not in weights or not np.isfinite(weights[name]).all():raise ValueError('Invalid learned weight '+name)
    schema=f'ct_scene.experimental_contact_adapter_{ver}' if ver in ('v3','v4') else 'ct_scene.experimental_contact_adapter_v1'
    manifest=dict(schema=schema,selected=report['selected'],adapter_config=report.get('adapter_config'),runtime_adopted=False,
        adoption_gate='NOT_APPROVED: experimental packaging only; review held-out metrics and metric 3D evidence before any integration',
        annotation_source=str(annotations.resolve()),annotations_sha256=sha(annotations),cuda=False,
        input_spec=dict(crop='bbox plus 12 percent left/right/top and16 percent bottom; clip to1280x960 source',color='RGB',resize='PIL bicubic224x224',normalize_mean=[.485,.456,.406],normalize_std=[.229,.224,.225],image_resolution=[1280,960]),
        files_sha256={name:sha(path) for name,path in [('adapter.npz',trained/'adapter.npz'),('contact-encoder.onnx',features/'contact-encoder.onnx')]})
    output.mkdir(parents=True,exist_ok=False)
    for source,name in [(trained/'adapter.npz','adapter.npz'),(features/'contact-encoder.onnx','contact-encoder.onnx'),(trained/'report.json','training-report.json'),(features/'provenance.json','encoder-provenance.json')]:shutil.copy2(source,output/name)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['trained','features','annotations','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();package(a.trained,a.features,a.annotations,a.output);print('Packaged experimental model; runtime adoption remains disabled')
