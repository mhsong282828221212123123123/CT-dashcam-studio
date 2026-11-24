"""Replay cached raw detections through current pixel tracker; no inference."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import Detection,MotionTracker,iou,CameraReader

def retrack(result,output,source_directory=None):
    source,output=Path(result),Path(output)
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((source/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('review_assisted'):raise ValueError('Reviewed overrides require explicit remapping')
    rows=[json.loads(s) for s in (source/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    trackers={};readers={}
    try:
        if source_directory:
            if any(meta.get('rectification',{}).get('status')=='applied' for meta in manifest['cameras'].values()):
                raise ValueError('Appearance replay needs matching rectified frames')
            for name,meta in manifest['cameras'].items():readers[name]=CameraReader(Path(source_directory)/meta['file'])
        for row in rows:
            for name,data in row['cameras'].items():
                if 'raw_detections' not in data:raise ValueError('Raw detections missing')
                detections=[Detection(v['kind'],v['confidence'],v['bbox']) for v in data['raw_detections'] if v['kind']!='traffic_light']
                tracker=trackers.setdefault(name,MotionTracker(name))
                frame=readers[name].read_at(row['time_s'])[1] if name in readers and data.get('available') else None
                data['objects']=tracker.update(detections,row['time_s'],frame=frame)
                for obj in data['objects']:
                    best=max(data['raw_detections'],key=lambda v:iou(obj['bbox'],v['bbox']),default=None)
                    if best and best.get('mask_polygon') and iou(obj['bbox'],best['bbox'])>.5:obj['mask_polygon']=best['mask_polygon']
    finally:
        for reader in readers.values():reader.close()
    manifest['tracking']='motion replay from original raw detections; per-axis size gate'
    manifest['retracked_from']=str(source.resolve())
    manifest['appearance_association']=bool(source_directory)
    output.mkdir(parents=True)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    (output/'observations.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n',encoding='utf-8')
    print('Retracked',len(rows),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result',required=True);p.add_argument('--output',required=True)
    p.add_argument('--source',help='Original camera folder for colour-supported association')
    a=p.parse_args();retrack(a.result,a.output,a.source)
