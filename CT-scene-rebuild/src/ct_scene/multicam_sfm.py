"""Try static landmark connectivity, without fabricating a rigid camera rig."""
from collections import Counter
from itertools import combinations
from pathlib import Path
import io
import json
import shutil
import time
import numpy as np
from PIL import Image,ImageDraw
from .artifacts import canonical,digest,publish,verify
from .sfm import select_frames
from .static_features import feature_roi

CAMERAS=('front','left_repeater','right_repeater')


def pairing_schedule(frames,within_s=4,cross_s=8):
    train=[f for f in frames if not f['heldout']]
    return [(a['sfm_name'],b['sfm_name']) for a,b in combinations(train,2)
            if abs(a['selection_time_s']-b['selection_time_s'])<=(within_s if a['camera_id']==b['camera_id'] else cross_s)]


def connected_camera_graph(edges,cameras=CAMERAS,min_shared=30):
    reached={cameras[0]}
    while True:
        before=len(reached)
        for a,b,count in edges:
            if count>=min_shared and (a in reached or b in reached):reached.update([a,b])
        if len(reached)==before:return set(cameras)<=reached


def audit_component(model):
    image_cameras={iid:im.name.split('/')[0] for iid,im in model.images.items()}
    counts=Counter(image_cameras.values());shared=Counter()
    for point in model.points3D.values():
        if point.track.length()<3:continue
        cameras={image_cameras[e.image_id] for e in point.track.elements}
        for a,b in combinations(sorted(cameras),2):shared[a,b]+=1
    edges=[(a,b,shared[tuple(sorted((a,b)))]) for a,b in combinations(CAMERAS,2)]
    connected=connected_camera_graph(edges) and all(counts[c]>=6 for c in CAMERAS)
    return {'registered_by_camera':dict(counts),'shared_landmarks':[{'a':a,'b':b,'count':n} for a,b,n in edges],
            'connected_three_camera_candidate':connected,'total_points3D':model.num_points3D()}


def run(package,masks,lens_config,destination,sample_hz=1):
    import pycolmap as pc
    package,masks,destination=Path(package),Path(masks),Path(destination)
    if destination.exists():raise FileExistsError(destination)
    pm,mm=verify(package),verify(masks)
    if not any(i['artifact_id']==pm['artifact_id'] for i in mm['inputs']):raise ValueError('Source/mask mismatch')
    config=json.loads(Path(lens_config).read_text());frames=[]
    all_frames=json.loads((package/'frames.json').read_text())
    for camera in CAMERAS:
        selected=select_frames(all_frames,camera,() if camera=='front' else (12,20,28),sample_hz)
        for f in selected:
            if [f['width'],f['height']]!=config['cameras'][camera]['resolution']:raise ValueError('Optical seed resolution mismatch')
            f['sfm_name']=camera+'/'+f['sfm_name'];frames.append(f)
    code=digest(Path(__file__).read_bytes())
    work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False)
    for name in ['images','query','masks','sparse']:(work/name).mkdir()
    observations={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    for f in frames:
        target=work/('query' if f['heldout'] else 'images')/f['sfm_name'];target.parent.mkdir(exist_ok=True)
        shutil.copyfile(package/f['image'],target)
        target=work/'masks'/(f['sfm_name']+'.png');target.parent.mkdir(exist_ok=True)
        mask=np.asarray(Image.open(masks/observations[f['frame_id']]['static_mask']))
        Image.fromarray(feature_roi(mask)).save(target)
    pc.set_random_seed(20261114);pc.logging.minloglevel=2
    database=work/'features.db';camera_ids={}
    with pc.Database.open(database) as db:
        for camera in CAMERAS:
            profile=config['cameras'][camera]
            cam=pc.Camera(model=profile['model'],width=profile['resolution'][0],height=profile['resolution'][1],params=profile['params'])
            camera_ids[camera]=db.write_camera(cam)
    extraction=pc.FeatureExtractionOptions();extraction.use_gpu=False;extraction.num_threads=4
    extraction.max_image_size=1280;extraction.sift.max_num_features=4096
    start=time.perf_counter()
    for camera in CAMERAS:
        reader=pc.ImageReaderOptions();reader.existing_camera_id=camera_ids[camera];reader.mask_path=work/'masks'
        names=[f['sfm_name'] for f in frames if f['camera_id']==camera and not f['heldout']]
        pc.extract_features(database,work/'images',image_names=names,reader_options=reader,
                            extraction_options=extraction,device=pc.Device.cpu)
        print('Extracted masked CPU features:',camera,len(names),flush=True)
    pairs=pairing_schedule(frames)
    pairs_path=work/'pairs.txt';pairs_path.write_text(''.join(a+' '+b+'\n' for a,b in pairs),encoding='utf-8')
    matcher=pc.FeatureMatchingOptions();matcher.use_gpu=False;matcher.num_threads=4
    pairing=pc.ImportedPairingOptions();pairing.match_list_path=pairs_path
    pc.match_image_pairs(database,matching_options=matcher,pairing_options=pairing,device=pc.Device.cpu)
    payloads={};edge_counts=Counter();strongest={}
    with pc.Database.open(database) as db:
        images={im.image_id:im for im in db.read_all_images()};ids,geometries=db.read_two_view_geometries()
        for pid,geometry in zip(ids,geometries):
            a,b=pc.pair_id_to_image_pair(pid);ca,cb=images[a].name.split('/')[0],images[b].name.split('/')[0]
            if ca==cb:continue
            key=tuple(sorted((ca,cb)));count=len(geometry.inlier_matches)
            if count>=30:edge_counts[key]+=1
            if key not in strongest or count>strongest[key][0]:strongest[key]=(count,a,b,geometry.inlier_matches.copy())
        for (ca,cb),(count,a,b,matches) in strongest.items():
            if count==0:continue
            ia=Image.open(work/'images'/images[a].name).convert('RGB');ib=Image.open(work/'images'/images[b].name).convert('RGB')
            proof=Image.new('RGB',(ia.width+ib.width,max(ia.height,ib.height)));proof.paste(ia,(0,0));proof.paste(ib,(ia.width,0));draw=ImageDraw.Draw(proof)
            ka=db.read_keypoints(a);kb=db.read_keypoints(b)
            for ai,bi in matches[::max(1,len(matches)//60)]:
                x,y=ka[ai,:2];u,v=kb[bi,:2];draw.line([x,y,ia.width+u,v],fill='yellow',width=1)
            out=io.BytesIO();proof.save(out,format='JPEG',quality=90);payloads[f'pair-proof/{ca}-{cb}.jpg']=out.getvalue()
    print('Cross-camera verified pair counts:',dict(edge_counts),flush=True)
    options=pc.IncrementalPipelineOptions();options.num_threads=4;options.random_seed=20261114
    options.ba_use_gpu=False;options.ba_refine_focal_length=False;options.ba_refine_extra_params=False
    options.mapper.abs_pose_refine_focal_length=False;options.mapper.abs_pose_refine_extra_params=False
    options.ba_refine_sensor_from_rig=False;options.min_model_size=6;options.max_num_models=5
    options.max_runtime_seconds=180;options.init_num_trials=50;options.mapper.init_min_tri_angle=4
    models=pc.incremental_mapping(database,work/'images',work/'sparse',options=options)
    audits=[dict(model_id=int(i),**audit_component(m)) for i,m in models.items()]
    report={'components':audits,'cross_camera_pair_counts_ge30':[{'a':a,'b':b,'pairs':n} for (a,b),n in edge_counts.items()],
            'pair_schedule_count':len(pairs),'device':'cpu','cuda':False,'lens_seed_status':config['status'],
            'connectivity_gate':'PASS_CANDIDATE_ONLY' if any(a['connected_three_camera_candidate'] for a in audits) else 'FAIL',
            'rig_transform':'NOT_ESTIMATED','clock_offset':'NOT_ESTIMATED','metric_scale':'unestablished',
            'sampling_hz':sample_hz,'wall_time_s':time.perf_counter()-start,
            'warning':'Image poses are unconstrained by camera rigidity. Shared landmarks are a candidate connection, not a verified rigid rig. Search times are relative container times, not exposure sync.'}
    if models:
        chosen=max(models,key=lambda i:(audit_component(models[i])['connected_three_camera_candidate'],len(audit_component(models[i])['registered_by_camera']),models[i].num_reg_images()))
        report['selected_model']=int(chosen)
        for p in (work/'sparse'/str(chosen)).iterdir():
            if p.is_file():payloads['reconstruction/'+p.name]=p.read_bytes()
    payloads['report.json']=canonical(report);payloads['input-index.json']=canonical(frames)
    return publish(destination,stage='multicam_landmark_connectivity',payloads=payloads,
                   inputs=[{'role':'raw_package','artifact_id':pm['artifact_id']},{'role':'dynamic_masks','artifact_id':mm['artifact_id']}],
                   config={'lens_config':config,'sample_hz':sample_hz,'same_camera_pair_window_s':4,'cross_camera_pair_window_s':8,
                           'side_holdout_times':[12,20,28],'min_shared_landmarks':30},
                   code_revision=code,runtime={'pycolmap':pc.__version__,'cuda':False})
