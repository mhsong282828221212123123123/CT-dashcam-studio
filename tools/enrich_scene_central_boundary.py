"""Add visible separator gating to a NEW cache, without object inference."""
import argparse,json,sys
from pathlib import Path
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader
from core.scene_central_boundary import CentralBoundaryTracker
from core.scene_fusion import cameras_for,load_profile


def enrich(result,source,output):
    result,source,output=Path(result),Path(source),Path(output)
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((result/'manifest.json').read_text(encoding='utf-8'))
    if any(meta.get('rectification',{}).get('status')=='applied' for meta in manifest.get('cameras',{}).values()):
        raise ValueError('Rectified observations require rectified source frames; rerun analysis with calibration')
    rows=[json.loads(line) for line in (result/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    geometry=cameras_for(manifest,load_profile(manifest));readers={};trackers={};counts={}
    output.parent.mkdir(parents=True,exist_ok=True)
    try:
        for name,meta in manifest['cameras'].items():
            readers[name]=CameraReader(source/meta['file']);trackers[name]=CentralBoundaryTracker()
            counts[name]=dict(active=0,excluded=0)
        for row in rows:
            for name,data in row['cameras'].items():
                _,frame=readers[name].read_at(row['time_s'])
                calibrated=manifest['cameras'][name].get('rectification',{}).get('status')=='applied'
                data['central_boundary']=trackers[name].update(frame,data.get('objects',[]),
                    geometry.get(name) if name!='back' or calibrated else None,row['time_s'])
                counts[name]['active']+=data['central_boundary']['active']
                counts[name]['excluded']+=sum(o.get('road_relation')=='beyond_central_boundary' for o in data.get('objects',[]))
    finally:
        for reader in readers.values():reader.close()
    manifest['central_boundary_analysis']='visible yellow separator; image-space far-side rejection before metric fitting'
    manifest['source_front_path']=str((source/manifest['cameras']['front']['file']).resolve())
    output.mkdir()
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (output/'observations.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf-8')
    (output/'boundary-review.json').write_text(json.dumps(counts,indent=2),encoding='utf-8')
    print(json.dumps(counts),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for field in ('result','source','output'):p.add_argument('--'+field,required=True)
    args=p.parse_args();enrich(args.result,args.source,args.output)
