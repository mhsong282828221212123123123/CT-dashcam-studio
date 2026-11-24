"""Denser CPU SfM input, synchronized by validated common SEI sequence IDs."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scene-runtime-calibration'))
import pycolmap as pc
from core.decoder import RealTeslaSEIDecoder
from core.scene_sync import SequenceAlignment


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('analysis', type=Path)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    root = args.output
    root.mkdir(exist_ok=True, parents=True)
    manifest = json.loads((args.analysis/'manifest.json').read_text(encoding='utf8'))
    rows = [json.loads(s) for s in (args.analysis/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    directory = Path(manifest['source_front_path']).parent
    names = ['front', 'left_repeater', 'right_repeater']
    paths = {n: directory / manifest['cameras'][n]['file'] for n in names}
    telemetry = {n: RealTeslaSEIDecoder(str(paths[n])).telemetry_cache for n in names}
    front_count = manifest['cameras']['front']['frame_count']
    sync = {n: SequenceAlignment(telemetry['front'], telemetry[n], front_count,
                                manifest['cameras'][n]['frame_count']) for n in names}
    if not all(v.report['status']=='sequence_matched' for v in sync.values()):
        raise ValueError({n:v.report for n,v in sync.items()})
    refs = []
    cap = cv2.VideoCapture(str(paths['front']))
    for time in np.arange(9., 27.01, .5):
        cap.set(cv2.CAP_PROP_POS_MSEC, time*1000)
        ok, _ = cap.read()
        if not ok:
            raise ValueError('Missing reference frame')
        refs.append((float(time), int(cap.get(cv2.CAP_PROP_POS_FRAMES))-1))
    cap.release()
    index = []
    for name in names:
        cap = cv2.VideoCapture(str(paths[name]))
        (root/'images'/name).mkdir(parents=True,exist_ok=True)
        (root/'masks'/name).mkdir(parents=True,exist_ok=True)
        for time, ref in refs:
            idx = sync[name].target_index(ref)
            if idx is None:
                raise ValueError('No common sequence for requested image')
            filename = f'{name}/{time:05.1f}.jpg'
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, image = cap.read()
            if not ok:
                raise ValueError('Missing sequence-aligned frame')
            h,w = image.shape[:2]
            mask = np.full((h,w),255,np.uint8)
            mask[:int(h*.06)]=0;mask[int(h*.86):]=0
            if name=='left_repeater':mask[:,:int(w*.07)]=0
            if name=='right_repeater':mask[:,int(w*.93):]=0
            row=min(rows,key=lambda r:abs(r['time_s']-time))
            for obj in row['cameras'][name].get('objects',[]):
                x,y,bw,bh=obj['bbox']
                cv2.rectangle(mask,(max(0,int(x)-20),max(0,int(y)-20)),(int(x+bw)+20,int(y+bh)+20),0,-1)
            cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,97])[1].tofile(root/'images'/filename)
            cv2.imencode('.png',mask)[1].tofile(root/'masks'/(filename+'.png'))
            index.append(dict(name=filename,time_s=time,frame_index=idx,front_frame_index=ref,
                              sequence=telemetry[name][idx]['frame_seq_no'],
                              pts_s=cap.get(cv2.CAP_PROP_POS_MSEC)/1000))
        cap.release()
    (root/'input-index.json').write_text(json.dumps(index,indent=2),encoding='utf8')
    (root/'sync.json').write_text(json.dumps({n:v.report for n,v in sync.items()},indent=2),encoding='utf8')
    db=root/'database.db'
    extraction=pc.FeatureExtractionOptions();extraction.num_threads=4
    extraction.max_image_size=1280;extraction.sift.max_num_features=4000
    for name in names:
        reader=pc.ImageReaderOptions();reader.mask_path=root/'masks'
        reader.camera_model='RADIAL';reader.camera_params=f'{1640 if name=="front" else 1100},640,480,-.3,0'
        pc.extract_features(db,root/'images',image_names=[r['name'] for r in index if r['name'].startswith(name+'/')],
                            camera_mode=pc.CameraMode.PER_FOLDER,reader_options=reader,
                            extraction_options=extraction,device=pc.Device.cpu)
    matching=pc.FeatureMatchingOptions();matching.num_threads=4
    pc.match_exhaustive(db,matching_options=matching,device=pc.Device.cpu)
    sparse=root/'sparse';sparse.mkdir(exist_ok=True)
    opts=pc.IncrementalPipelineOptions();opts.num_threads=4;opts.min_model_size=8
    opts.mapper.init_min_tri_angle=8.;opts.mapper.init_max_forward_motion=.99
    opts.max_runtime_seconds=180;opts.ba_global_max_num_iterations=100
    models=pc.incremental_mapping(db,root/'images',sparse,options=opts)
    report=[dict(id=k,images=[i.name for i in m.images.values() if i.has_pose],
                 points=m.num_points3D(),error_px=m.compute_mean_reprojection_error(),
                 cameras={int(k):dict(model=c.model_name,params=c.params.tolist()) for k,c in m.cameras.items()})
            for k,m in models.items()]
    (root/'sfm-report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print('COMPLETE', [(r['id'],len(r['images']),r['error_px']) for r in report],flush=True)


if __name__=='__main__':main()
