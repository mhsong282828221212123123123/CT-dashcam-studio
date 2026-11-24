"""Image-only angle diagnostic at source-labelled anchors; no scene mutation."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from ct_scene.artifacts import verify
from ct_scene.ingest import file_hash
from ct_scene.affine_probe import camera_matrix
from ct_scene.image_vehicle_heading import ImageHeadingModel,camera_heading,wrap
import cv2

ap=argparse.ArgumentParser();ap.add_argument('--requests',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--model',type=Path,required=True);ap.add_argument('--optics',type=Path,default=Path('configs/multicam-optical-candidates.json'));a=ap.parse_args();a.output.mkdir(exist_ok=False)
model=ImageHeadingModel(a.model,a.output/'ort');K,D=camera_matrix(json.loads(a.optics.read_text())['cameras']['front']);results=[];tiles=[];sources={};start=time.perf_counter()
for req in json.loads(a.requests.read_text())['requests']:
    case=Path(req['case']);key=str(case)
    if key not in sources:sources[key]=dict(source=verify(case/'source')['artifact_id'],tracks=verify(case/'tracks')['artifact_id'])
    frames=json.loads((case/'source/frames.json').read_text());frame=min((x for x in frames if x['camera_id']=='front'),key=lambda f:abs(f['camera_time_s']-req['time_s']))
    image=np.asarray(Image.open(case/'source'/frame['image']).convert('RGB'))
    observations=[json.loads(l) for l in (case/'tracks/observations.jsonl').read_text().splitlines()];rows=[r for r in observations if r['frame_id']==frame['frame_id'] and r['support']=='detector_observation' and r['class_id'] in [2,5,7] and (not req.get('local_ids') or r['local_track_id'] in req['local_ids'])]
    for row in rows:
        box=row['bbox_xyxy'];prediction=model.predict(image,box);record=dict(label=req['label'],local_id=row['local_track_id'],camera_time_s=frame['camera_time_s'],frame_id=frame['frame_id'],bbox_xyxy=box,class_id=row['class_id'],**prediction)
        if 'alpha_prime_rad' in prediction:
            point=np.array([[[(box[0]+box[2])/2,(box[1]+box[3])/2]]],float);ray=float(np.arctan(cv2.undistortPoints(point,K,D)[0,0,0]));angle=camera_heading(prediction['alpha_prime_rad'],ray)
            record.update(ray_rad=ray,ego_relative_heading_rad=angle,ego_relative_heading_deg=math.degrees(angle),optics_status='nominal_front_optics; raw_crop_input_not_metric_acceptance')
            if 'expected_relative_heading_deg' in req:record['source_approx_error_deg']=abs(math.degrees(wrap(angle-math.radians(req['expected_relative_heading_deg']))))
        results.append(record)
        if box[2]-box[0]<80 or box[3]-box[1]<60:continue
        tile=Image.new('RGB',(400,300),'#171b20');crop=Image.fromarray(image).crop(tuple(box));crop.thumbnail((390,210));tile.paste(crop,((400-crop.width)//2,75));d=ImageDraw.Draw(tile);d.text((5,5),req['label']+' '+row['local_track_id'],fill='white');d.text((5,25),prediction['reason'],fill='#8fdfad' if prediction['accepted'] else '#f0c36c');d.text((5,45),'heading '+str(round(record.get('ego_relative_heading_deg',float('nan')),1))+' deg / mirror '+str(round(record.get('mirror_error_deg',float('nan')),1)),fill='white');tiles.append(tile)
sheet=Image.new('RGB',(1200,300*max(1,math.ceil(len(tiles)/3))),'#171b20')
for i,tile in enumerate(tiles):sheet.paste(tile,((i%3)*400,(i//3)*300))
sheet.save(a.output/'source-crops.jpg');report=dict(scope='experimental image orientation; all source-labelled, no production adoption or metric XYZ claim',model_sha256=file_hash(a.model),sources=sources,elapsed_s=time.perf_counter()-start,profile=model.finish_profile(),results=results)
(a.output/'review.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(profile=report['profile'],results=[{k:v for k,v in x.items() if k in ['label','local_id','accepted','reason','ego_relative_heading_deg','mirror_error_deg','source_approx_error_deg']} for x in results]),indent=2))
