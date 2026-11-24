"""Masked multi-frame optical experiment with excluded-image localization.

CPU-only COLMAP. Reconstruction coordinates have no metric/body-frame meaning.
"""
from pathlib import Path
import io
import json
import shutil
import time
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .artifacts import canonical,digest,publish,verify
from .static_features import feature_roi,optical_feature_roi


def select_frames(frames,camera,holdout_times=(4,12,20),sample_hz=1):
    if sample_hz not in (1,2,5):raise ValueError('Use 1, 2 or 5 Hz SfM sampling')
    sequence=sorted((f for f in frames if f['camera_id']==camera),key=lambda f:f['relative_request_s'])
    if not sequence:raise ValueError('Missing camera')
    selected=[]
    for step in range((int(max(f['relative_request_s'] for f in sequence))+1)*sample_hz):
        t=step/sample_hz
        f=min(sequence,key=lambda f:abs(f['relative_request_s']-t))
        if abs(f['relative_request_s']-t)>.11:raise ValueError('Missing requested SfM input time')
        selected.append(dict(f,sfm_name=f'{round(t*1000):06d}.jpg',heldout=t in holdout_times,selection_time_s=t))
    if len({f['frame_id'] for f in selected})!=len(selected):raise ValueError('Duplicate SfM frames')
    if not set(holdout_times)<={f['selection_time_s'] for f in selected}:raise ValueError('Holdout time absent')
    return selected


def query_matches(query_desc,map_desc,map_points):
    if len(query_desc)<2 or len(map_desc)<2:return np.empty(0,int),np.empty((0,3))
    matches=cv2.BFMatcher(cv2.NORM_L2).knnMatch(np.asarray(query_desc,np.float32),np.asarray(map_desc,np.float32),k=2)
    good=[a for pair in matches if len(pair)==2 for a,b in [pair] if a.distance<.75*b.distance]
    best={}
    for m in sorted(good,key=lambda m:m.distance):
        if m.trainIdx not in best:best[m.trainIdx]=m
    chosen=list(best.values())
    return np.array([m.queryIdx for m in chosen],int),np.array([map_points[m.trainIdx] for m in chosen],float).reshape(-1,3)


def consensus_matches(query_desc,reference_views,points_by_id,query_time=None):
    """Match before pose fitting; require mutual descriptor votes from 2 views."""
    query_desc=np.asarray(query_desc,np.float32);votes={}
    matcher=cv2.BFMatcher(cv2.NORM_L2)
    temporal={v[0]:v[3] for v in reference_views if len(v)==4}
    if query_time is not None:
        reference_views=[v for v in reference_views if v[0] in temporal and abs(temporal[v[0]]-query_time)<=3]
    can_bracket=query_time is not None and any(temporal[v[0]]<query_time for v in reference_views) and any(temporal[v[0]]>query_time for v in reference_views)
    for view in reference_views:
        view_id,desc,point_ids=view[:3]
        desc=np.asarray(desc,np.float32)
        if len(query_desc)<2 or len(desc)<2:continue
        reverse={m.queryIdx:m.trainIdx for m in matcher.match(desc,query_desc)}
        for pair in matcher.knnMatch(query_desc,desc,k=2):
            if len(pair)!=2:continue
            a,b=pair;pid=point_ids[a.trainIdx]
            if pid not in points_by_id or a.distance>=.75*b.distance or reverse.get(a.trainIdx)!=a.queryIdx:continue
            votes.setdefault(a.queryIdx,{}).setdefault(pid,[]).append((view_id,a.distance))
    candidates=[]
    for query_id,options in votes.items():
        ranked=sorted(options.items(),key=lambda item:(-len({v for v,d in item[1]}),np.mean([d for v,d in item[1]])))
        pid,evidence=ranked[0];count=len({v for v,d in evidence})
        other=len({v for v,d in ranked[1][1]}) if len(ranked)>1 else 0
        bracket=not can_bracket or (any(temporal[v]<query_time for v,d in evidence) and any(temporal[v]>query_time for v,d in evidence))
        if count>=2 and count>=2*other and bracket:candidates.append((query_id,pid,count,float(np.mean([d for v,d in evidence]))))
    selected={}
    for row in sorted(candidates,key=lambda x:(-x[2],x[3])):
        if row[1] not in selected:selected[row[1]]=row
    rows=list(selected.values())
    return np.array([r[0] for r in rows],int),np.array([points_by_id[r[1]] for r in rows],float).reshape(-1,3)


def run(package,masks,destination,camera='front',holdout_times=(4,12,20),bootstrap_artifact=None,sample_hz=1,refine_optics_after_mapping=True,roi_mode='legacy_conservative'):
    import pycolmap as pc
    if camera not in {'front','left_repeater','right_repeater'}:raise ValueError('Rear/pillar optical models are not validated by this experiment')
    if roi_mode not in ('legacy_conservative','camera_aware_v1'):raise ValueError('Unknown optical ROI')
    if not refine_optics_after_mapping and bootstrap_artifact is None:raise ValueError('Fixed optics requires an explicit bootstrap artifact')
    package,masks,destination=Path(package),Path(masks),Path(destination)
    if destination.exists():raise FileExistsError(destination)
    pm,mm=verify(package),verify(masks)
    if not any(i['artifact_id']==pm['artifact_id'] for i in mm['inputs']):raise ValueError('Masks refer to another raw package')
    frames=select_frames(json.loads((package/'frames.json').read_text()),camera,holdout_times,sample_hz)
    if len({(f['width'],f['height']) for f in frames})!=1:raise ValueError('Mixed image dimensions')
    code_hash=digest(Path(__file__).read_bytes())
    work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False)
    obs={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    for folder in ['train','query','masks','sparse']:(work/folder).mkdir()
    for f in frames:
        folder='query' if f['heldout'] else 'train'
        shutil.copyfile(package/f['image'],work/folder/f['sfm_name'])
        mask=np.array(Image.open(masks/obs[f['frame_id']]['static_mask']))
        roi=feature_roi(mask) if roi_mode=='legacy_conservative' else optical_feature_roi(mask,camera)
        Image.fromarray(roi).save(work/'masks'/(f['sfm_name']+'.png'))
    pc.set_random_seed(20261114)
    pc.logging.minloglevel=2
    reader=pc.ImageReaderOptions();reader.camera_model='SIMPLE_RADIAL';reader.mask_path=work/'masks'
    bootstrap=None;bootstrap_manifest=None
    if bootstrap_artifact is not None:
        bootstrap_manifest=verify(bootstrap_artifact)
        bootstrap_report=json.loads((Path(bootstrap_artifact)/'report.json').read_text())
        if bootstrap_report['camera']!=camera:raise ValueError('Bootstrap belongs to another camera')
        bootstrap=bootstrap_report['optical_candidate']
        if bootstrap['model']!='SIMPLE_RADIAL' or [bootstrap['width'],bootstrap['height']]!=[frames[0]['width'],frames[0]['height']]:raise ValueError('Incompatible bootstrap lens')
        if not .5*frames[0]['width']<=bootstrap['params'][0]<=3*frames[0]['width']:raise ValueError('Nonphysical bootstrap focal range')
        reader.camera_params=','.join(map(str,bootstrap['params']))
    extraction=pc.FeatureExtractionOptions();extraction.use_gpu=False;extraction.num_threads=4
    extraction.max_image_size=max(frames[0]['width'],frames[0]['height']);extraction.sift.max_num_features=4096
    matching=pc.FeatureMatchingOptions();matching.use_gpu=False;matching.num_threads=4
    start=time.perf_counter()
    for folder in ['train','query']:
        pc.extract_features(work/(folder+'.db'),work/folder,camera_mode=pc.CameraMode.SINGLE,
                            reader_options=reader,extraction_options=extraction,device=pc.Device.cpu)
    print('SIFT extracted on CPU; holdout images are in a separate database',flush=True)
    pc.match_exhaustive(work/'train.db',matching_options=matching,device=pc.Device.cpu)
    options=pc.IncrementalPipelineOptions();options.num_threads=4;options.random_seed=20261114
    options.ba_use_gpu=False;options.min_model_size=8;options.max_num_models=3
    options.init_num_trials=50;options.mapper.init_min_tri_angle=4;options.max_runtime_seconds=120
    options.ba_refine_sensor_from_rig=False
    options.min_focal_length_ratio=.5;options.max_focal_length_ratio=3
    options.mapper.min_focal_length_ratio=.5;options.mapper.max_focal_length_ratio=3
    if bootstrap is not None:
        options.ba_refine_focal_length=False;options.ba_refine_extra_params=False
        options.mapper.abs_pose_refine_focal_length=False;options.mapper.abs_pose_refine_extra_params=False
    models=pc.incremental_mapping(work/'train.db',work/'train',work/'sparse',options=options)
    report={'camera':camera,'train_images':sum(not f['heldout'] for f in frames),'heldout_images':len(holdout_times),
            'components':[{'id':int(i),'registered_images':m.num_reg_images(),'points3D':m.num_points3D()} for i,m in models.items()],
            'device':'cpu','cuda':False,'metric_scale':'unestablished','body_origin':'unestablished',
            'cross_camera_rig':'NOT_ESTIMATED','query_tests':[],'gate':'FAIL'}
    payloads={'input-index.json':canonical(frames)}
    if models:
        model_id,model=max(models.items(),key=lambda item:item[1].num_reg_images())
        if bootstrap is not None:
            ba=pc.BundleAdjustmentOptions();ba.ceres.use_gpu=False;ba.ceres.solver_options.num_threads=4
            ba.refine_sensor_from_rig=False;ba.print_summary=False
            ba.refine_focal_length=refine_optics_after_mapping;ba.refine_extra_params=refine_optics_after_mapping
            pc.bundle_adjustment(model,ba)
            model.write(work/'sparse'/str(model_id))
        model_path=work/'sparse'/str(model_id)
        for file in model_path.iterdir():
            if file.is_file():payloads['reconstruction/'+file.name]=file.read_bytes()
        camera_model=next(iter(model.cameras.values()))
        report['optical_candidate']={'model':camera_model.model_name,'params':camera_model.params.tolist(),'width':camera_model.width,'height':camera_model.height,'status':'fixed_bootstrap_candidate' if bootstrap and not refine_optics_after_mapping else 'new_sfm_estimate_not_measured'}
        report['optics_refined_after_mapping']=bool(bootstrap and refine_optics_after_mapping)
        report['registered_train_images']=[im.name for im in model.images.values()]
        report['training_mean_reprojection_px']=model.compute_mean_reprojection_error()
        report['intrinsics_bootstrap_then_global_refine']=bootstrap is not None
        reference_views=[];points_by_id={pid:point.xyz for pid,point in model.points3D.items() if point.track.length()>=3}
        frame_times={f['sfm_name']:f['camera_time_s'] for f in frames}
        with pc.Database.open(work/'train.db') as database:
            for im in model.images.values():
                desc=np.asarray(database.read_descriptors(im.image_id).data)
                point_ids=[p.point3D_id for p in im.points2D]
                reference_views.append((im.image_id,desc,point_ids,frame_times[im.name]))
        with pc.Database.open(work/'query.db') as database:
            for query_image in database.read_all_images():
                desc=database.read_descriptors(query_image.image_id).data
                kp=database.read_keypoints(query_image.image_id)[:,:2]
                ids,xyz=consensus_matches(desc,reference_views,points_by_id,frame_times[query_image.name])
                result={'image':query_image.name,'map_matches':len(ids),'gate':'FAIL'}
                if len(ids)>=45:
                    order=np.random.default_rng(20261114).permutation(len(ids));test=order[::3];train=np.setdiff1d(order,test)
                    est=pc.AbsolutePoseEstimationOptions();est.ransac.max_error=3;est.ransac.random_seed=20261114
                    refine=pc.AbsolutePoseRefinementOptions();refine.refine_focal_length=False;refine.refine_extra_params=False
                    pose=pc.estimate_and_refine_absolute_pose(kp[ids[train]],xyz[train],camera_model,est,refine)
                    if pose is not None:
                        prediction=camera_model.img_from_cam(pose['cam_from_world']*xyz[test])
                        errors=np.linalg.norm(prediction-kp[ids[test]],axis=1);finite=np.isfinite(errors)
                        med=float(np.median(errors)) if finite.all() else None
                        p95=float(np.quantile(errors,.95)) if finite.all() else None
                        result.update(test_points=len(test),pose_fit_points=len(train),median_px=med,p95_px=p95,
                                      invalid_test_projections=int((~finite).sum()),gate='PASS' if len(test)>=30 and med is not None and med<=3 and p95<=8 else 'FAIL')
                        im=Image.open(work/'query'/query_image.name).convert('RGB');draw=ImageDraw.Draw(im)
                        for measured,pred,valid in zip(kp[ids[test]],prediction,finite):
                            if not valid:continue
                            x,y=measured;u,v=pred
                            draw.line([x,y,u,v],fill='yellow',width=2);draw.ellipse([x-3,y-3,x+3,y+3],outline='lime',width=2)
                        buf=io.BytesIO();im.save(buf,format='JPEG',quality=90);payloads['query-proof/'+query_image.name]=buf.getvalue()
                        buf=io.BytesIO();np.savez_compressed(buf,pixels_measured=kp[ids[test]],pixels_predicted=prediction,points3D=xyz[test],R_C_W=pose['cam_from_world'].rotation.matrix(),t_C_W=pose['cam_from_world'].translation)
                        payloads['query-data/'+query_image.name+'.npz']=buf.getvalue()
                report['query_tests'].append(result)
        coverage=model.num_reg_images()/report['train_images']
        report['training_registration_fraction']=coverage
        params=camera_model.params
        report['broad_optical_sanity']=bool(.5*camera_model.width<=params[0]<=3*camera_model.width and np.isfinite(params).all() and abs(params[-1])<=1)
        if report['broad_optical_sanity'] and coverage>=.8 and len(report['query_tests'])==len(holdout_times) and all(q['gate']=='PASS' for q in report['query_tests']):report['gate']='PASS_OPTICAL_EXPERIMENT_ONLY'
    report['wall_time_s']=time.perf_counter()-start
    report['limitations']=['Same-recording static feature experiment, not independent journey accuracy','No surveyed scale, body frame or camera rig','Descriptor correspondence errors count in heldout pixel residuals','No metric ego trajectory or actor position accepted']
    payloads['report.json']=canonical(report)
    return publish(destination,stage='masked_optical_sfm',payloads=payloads,
                   inputs=[{'role':'raw_package','artifact_id':pm['artifact_id']},{'role':'dynamic_masks','artifact_id':mm['artifact_id']}]+([{'role':'training_bootstrap_lens','artifact_id':bootstrap_manifest['artifact_id']}] if bootstrap_manifest else []),
                   config={'camera':camera,'holdout_times':list(holdout_times),'model':'SIMPLE_RADIAL','default_focal_factor':1.2,
                           'bootstrap_params':bootstrap['params'] if bootstrap else None,'focal_ratio_range':[.5,3],
                           'refine_optics_after_mapping':refine_optics_after_mapping,
                           'sampling_hz':sample_hz,'roi':roi_mode,'query_matching':'mutual_descriptors_at_least_two_training_views',
                           'query_reference_window_s':3,'require_bracketing_views_when_available':True,
                           'max_features':4096,'min_track_length':3,'query_ratio':.75,'median_gate_px':3,'p95_gate_px':8,'min_test_points':30},
                   code_revision=code_hash,runtime={'pycolmap':pc.__version__,'cuda':False})
