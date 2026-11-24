"""CPU-only static-scene SfM calibration, isolated from the app environment.

No detected vehicle boxes are used as calibration measurements. Their pixels
are masked. COLMAP estimates real static 3D tie points and camera intrinsics.
"""
import json,sys,cv2
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scene-runtime-calibration'))
import pycolmap as pc

if __name__=='__main__':
    source=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=True,parents=True)
    manifest=json.loads((source/'manifest.json').read_text(encoding='utf8'))
    rows=[json.loads(x) for x in (source/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    directory=Path(manifest['source_front_path']).parent
    images=out/'images';masks=out/'masks';db=out/'database.db';sparse=out/'sparse';sparse.mkdir(exist_ok=True)
    index=[]
    for name in ('front','left_repeater','right_repeater','back'):
        cap=cv2.VideoCapture(str(directory/manifest['cameras'][name]['file']))
        (images/name).mkdir(exist_ok=True,parents=True);(masks/name).mkdir(exist_ok=True,parents=True)
        for time in np.arange(1.,36.,1.):
            row=min(rows,key=lambda r:abs(r['time_s']-time));filename=f'{name}/{time:05.1f}.jpg'
            if not (images/filename).exists():
                cap.set(cv2.CAP_PROP_POS_MSEC,time*1000);ok,frame=cap.read()
                if not ok:raise ValueError('Missing original frame')
                h,w=frame.shape[:2];mask=np.full((h,w),255,np.uint8)
                mask[:int(h*.08)]=0;mask[int(h*(.78 if name=='back' else .88)):]=0
                if name=='left_repeater':mask[:,:int(w*.06)]=0
                if name=='right_repeater':mask[:,int(w*.94):]=0
                for obj in row['cameras'][name].get('objects',[]):
                    x,y,bw,bh=obj['bbox'];cv2.rectangle(mask,(max(0,int(x)-10),max(0,int(y)-10)),(int(x+bw)+10,int(y+bh)+10),0,-1)
                cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,97])[1].tofile(str(images/filename))
                cv2.imencode('.png',mask)[1].tofile(str(masks/(filename+'.png')))
            index.append(dict(name=filename,time_s=float(time),camera=name))
        cap.release()
    (out/'input-index.json').write_text(json.dumps(index,indent=2),encoding='utf8')
    extraction=pc.FeatureExtractionOptions();extraction.num_threads=4;extraction.max_image_size=960;extraction.sift.max_num_features=2000
    for name in (() if '--map-only' in sys.argv else ('front','left_repeater','right_repeater','back')):
        reader=pc.ImageReaderOptions();reader.mask_path=masks
        if name=='back':reader.camera_model='OPENCV_FISHEYE';reader.camera_params='500,500,640,480,0,0,0,0'
        else:reader.camera_model='SIMPLE_RADIAL';reader.camera_params=f'{1500 if name=="front" else 800},640,480,0'
        pc.extract_features(db,images,image_names=[r['name'] for r in index if r['camera']==name],
            camera_mode=pc.CameraMode.PER_FOLDER,reader_options=reader,extraction_options=extraction,device=pc.Device.cpu)
    matching=pc.FeatureMatchingOptions();matching.num_threads=4
    if '--map-only' not in sys.argv:pc.match_exhaustive(db,matching_options=matching,device=pc.Device.cpu)
    options=pc.IncrementalPipelineOptions();options.num_threads=4;options.min_model_size=8
    options.mapper.init_min_tri_angle=8.;options.mapper.init_max_forward_motion=.99
    options.max_runtime_seconds=240;options.ba_global_max_num_iterations=70
    models=pc.incremental_mapping(db,images,sparse,options=options)
    report=[]
    for key,model in models.items():
        report.append(dict(id=key,registered_images=model.num_reg_images(),points3D=model.num_points3D(),
            mean_reprojection_error_px=model.compute_mean_reprojection_error(),
            cameras={int(k):dict(model=c.model_name,params=c.params.tolist(),width=c.width,height=c.height)
                     for k,c in model.cameras.items()},
            images=[i.name for i in model.images.values()],status='not_metric_or_rig_validated'))
    (out/'sfm-report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print('SFM REPORT',[(r['registered_images'],r['points3D'],r['cameras']) for r in report],flush=True)
