"""Review-only automatic side-road evidence with verified source/frame provenance."""
import argparse,json,math,time
from pathlib import Path
from collections import defaultdict,Counter
import numpy as np
from PIL import Image,ImageDraw
import onnxruntime as ort
from ct_scene.artifacts import verify,canonical
from ct_scene.ingest import file_hash
from ct_scene.mask_prepass import prepare_image
from ct_scene.road_perception import restore_mask
from ct_scene.camera_road_support import below_body_score,summarize
from review_guards import require_input_artifact


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cases',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--backend',choices=['directml','cpu'],default='directml');p.add_argument('--sample-period',type=float,default=.5);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    if a.sample_period<=0:raise ValueError('Positive sampling interval required')
    o=ort.SessionOptions();o.intra_op_num_threads=2;o.inter_op_num_threads=1;o.enable_mem_pattern=False;o.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL;o.enable_profiling=True;o.profile_file_prefix=str(a.output/'ort')
    if a.backend=='directml' and 'DmlExecutionProvider' not in ort.get_available_providers():raise ValueError('DirectML unavailable; explicitly select CPU')
    session=ort.InferenceSession(str(a.model),o,providers=['DmlExecutionProvider','CPUExecutionProvider'] if a.backend=='directml' else ['CPUExecutionProvider']);results=[];started=time.monotonic()
    for case in json.loads(a.cases.read_text()):
        scene=Path(case['scene']);source=Path(case['source']);manifest=verify(scene);source_manifest=verify(source);require_input_artifact(manifest,source_manifest,'source');frames={r['frame_id']:r for r in json.loads((source/'frames.json').read_text())};actors=json.loads((scene/'actors.json').read_text());requests=defaultdict(list);records=defaultdict(list);proof={}
        for actor in actors:
            per_cam=defaultdict(list)
            for row in actor['samples']:
                if row.get('camera_id') in ('left_repeater','right_repeater') and row.get('support')=='detector_observation':per_cam[row['camera_id']].append(row)
            for cam,rows in per_cam.items():
                last=-1e9
                for row in sorted(rows,key=lambda r:r['time_s']):
                    box=row['bbox_xyxy']
                    if row['time_s']-last<a.sample_period or min(box[2]-box[0],box[3]-box[1])<28:continue
                    if row['frame_id'] not in frames or frames[row['frame_id']]['camera_id']!=cam:raise ValueError('Source frame/camera mismatch')
                    last=row['time_s'];requests[row['frame_id']].append((actor.get('global_id',actor['id']),row))
        for fid,items in requests.items():
            f=frames[fid];rgb=np.array(Image.open(source/f['image']).convert('RGB'));tensor,_=prepare_image(rgb);road,lane=session.run(None,{'images':tensor});mask=restore_mask(np.argmax(road[0],axis=0),rgb.shape[:2])
            for key,row in items:
                score=below_body_score(mask,row['bbox_xyxy']);records[key].append(dict(time_s=row['time_s'],camera_id=row['camera_id'],frame_id=fid,bbox_xyxy=row['bbox_xyxy'],road_fraction=score))
                if key not in proof and score is not None and score<=.1:
                    overlay=rgb.copy();overlay[mask>0]=(overlay[mask>0]*.55+np.array([0,190,90])*.45).astype('uint8');im=Image.fromarray(overlay);ImageDraw.Draw(im).rectangle(row['bbox_xyxy'],outline='cyan',width=3);tile=Image.new('RGB',(640,520),'#17191c');tile.paste(im.resize((640,480)),(0,40));ImageDraw.Draw(tile).text((5,5),key+' '+str(round(row['time_s'],3))+'s road '+str(round(score,3)),fill='white');proof[key]=tile
        summaries={key:summarize(rows) for key,rows in records.items()};candidates=[key for key,r in summaries.items() if r['recommendation']=='review_off_carriageway'];tiles=[proof[key] for key in candidates if key in proof]
        if tiles:
            sheet=Image.new('RGB',(1280,520*math.ceil(len(tiles)/2)),'#17191c')
            for i,tile in enumerate(tiles):sheet.paste(tile,((i%2)*640,(i//2)*520))
            sheet.save(a.output/(case['name']+'-review-candidates.jpg'))
        result=dict(case=case['name'],source_artifact_id=source_manifest['artifact_id'],scene_artifact_id=manifest['artifact_id'],inferred_frames=len(requests),actors=summaries,records=dict(records),review_candidates=candidates)
        (a.output/(case['name']+'.json')).write_bytes(canonical(result));results.append(result);print('SIDE ROAD AUDIT',case['name'],'frames',len(requests),'review candidates',candidates,flush=True)
    profile=Path(session.end_profiling());events=json.loads(profile.read_text());kernel_counts=dict(Counter(e.get('args',{}).get('provider') for e in events if e.get('args',{}).get('provider')))
    report=dict(model_sha256=file_hash(a.model),kernel_provider_counts=kernel_counts,cuda=False,sample_period_s=a.sample_period,source_review_required=True,automatically_applied=False,elapsed_s=time.monotonic()-started,scope='Review-only image road support. No deletion, XYZ fitting, identity changes, road truth or calibrated confidence claim.',results=results)
    (a.output/'report.json').write_bytes(canonical(report));print('SIDE ROAD AUDIT COMPLETE',kernel_counts,flush=True)
