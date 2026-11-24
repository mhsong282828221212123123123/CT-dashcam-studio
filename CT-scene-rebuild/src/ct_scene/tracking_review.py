"""Raw 4-channel ID/gap review export, explicitly not a 3D reconstruction."""
from pathlib import Path
from fractions import Fraction
from collections import defaultdict
import io
import json
import av
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .artifacts import verify,publish,canonical,digest
from .tracking import iou


def annotate(image,observations):
    image=image.copy();draw=ImageDraw.Draw(image)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
    for row in observations:
        box=row['bbox_xyxy'];support=row['support'];color=(55,225,130) if support=='detector_observation' else (70,200,255) if support=='flow_observation' else (255,190,50)
        draw.rectangle(box,outline=color,width=3)
        name=row['local_track_id'].replace('left_repeater','L').replace('right_repeater','R')+('' if support=='detector_observation' else ' [flow]' if support=='flow_observation' else ' [offline gap]')
        x,y=box[:2];y=max(0,y-22);bounds=draw.textbbox((x,y),name,font=font);draw.rectangle(bounds,fill=(16,23,28));draw.text((x,y),name,font=font,fill=color)
    return image


def run(package,tracks,destination,sparse_references=None,presence_references=None):
    package,tracks,destination=Path(package),Path(tracks),Path(destination)
    if destination.exists():raise FileExistsError(destination)
    pm,tm=verify(package),verify(tracks)
    if not any(i['artifact_id']==pm['artifact_id'] for i in tm['inputs']):raise ValueError('Track/source mismatch')
    frames=json.loads((package/'frames.json').read_text());rows=list(map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()))
    by_frame=defaultdict(list)
    for row in rows:by_frame[row['frame_id']].append(row)
    cameras=['front','back','left_repeater','right_repeater'];sequences={c:sorted([f for f in frames if f['camera_id']==c],key=lambda f:f['relative_request_s']) for c in cameras}
    report={'scope':'2D camera-local IDs and offline raw-box gaps only. No world positioning or camera handoff acceptance.',
            'clock_status':'relative container requests aligned; exposure synchronization unverified','sample_hz':5,'sparse_anchor_checks':[],'seed_track_presence_coverage':[]}
    seed={}
    if sparse_references:
        refs=json.loads(Path(sparse_references).read_text())['reference_labels']
        for ref in refs:
            candidates=[r for r in by_frame[ref['frame_id']] if r['support']=='detector_observation']
            overlap,row=max(((iou(ref['bbox_xyxy'],r['bbox_xyxy']),r) for r in candidates),key=lambda p:p[0]) if candidates else (0,None)
            if row and ref['object_id'] not in seed:seed[ref['object_id']]=row['local_track_id']
            report['sparse_anchor_checks'].append({'object_id':ref['object_id'],'frame_id':ref['frame_id'],'bbox_iou':overlap,
                'local_track_id':row['local_track_id'] if row else None,'same_as_initial_seed':bool(row and seed.get(ref['object_id'])==row['local_track_id'])})
    if presence_references:
        presence=list(map(json.loads,Path(presence_references).read_text().splitlines()))
        for actor,key in seed.items():
            selected=[p for p in presence if p['object_id']==actor];counts=defaultdict(int)
            for p in selected:
                r=next((r for r in by_frame[p['frame_id']] if r['local_track_id']==key),None);counts[r['support'] if r else 'missing']+=1
            report['seed_track_presence_coverage'].append({'object_id':actor,'seed_local_track_id':key,'reference_frames':len(selected),
                'counts':dict(counts),'warning':'Selected actor presence references plus three sparse identity/bbox anchors; not exhaustive GT or full IDF1.'})
    work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False);video=work/'tracking-review.mp4'
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',20);payloads={};snapshots=[]
    with av.open(str(video),'w',options={'movflags':'+faststart'}) as output:
        stream=output.add_stream('libx264',rate=5);stream.width=1280;stream.height=1040;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'}
        for j,front in enumerate(sequences['front']):
            t=front['relative_request_s'];canvas=Image.new('RGB',(1280,1040),(23,28,32));d=ImageDraw.Draw(canvas)
            d.text((15,10),f'2D ID + OFFLINE GAP REVIEW | {t:05.2f}s | green: detector / cyan: flow / amber: gap',font=font,fill='white')
            selected={c:min(seq,key=lambda f:abs(f['relative_request_s']-t)) for c,seq in sequences.items()}
            for k,c in enumerate(cameras):
                f=selected[c];image=annotate(Image.open(package/f['image']).convert('RGB'),by_frame[f['frame_id']])
                if c=='front':canvas.paste(image.resize((960,720)),(160,40))
                tile=image.resize((320,240));td=ImageDraw.Draw(tile);td.rectangle((0,0,320,25),fill=(15,20,25));td.text((4,3),f'{c} {f["camera_time_s"]:.3f}s',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',15),fill='white');canvas.paste(tile,(320*k,760))
            d.text((15,1009),'No 3D positions assigned. Camera exposure clocks unverified. Source sampling: 5 Hz.',font=font,fill=(190,200,208))
            frame=av.VideoFrame.from_image(canvas);frame.pts=j;frame.time_base=Fraction(1,5)
            for packet in stream.encode(frame):output.mux(packet)
            if j in (0,40,80,120,156,165):snapshots.append(canvas.resize((640,520)))
        for packet in stream.encode():output.mux(packet)
    with av.open(str(video)) as check:
        count=sum(1 for _ in check.decode(video=0))
        if count!=len(sequences['front']):raise ValueError('Review export lost frames')
        duration=float(check.duration/av.time_base)
    report.update(video_frames=count,video_duration_s=duration,video_codec='h264',decoded_all_frames=True)
    sheet=Image.new('RGB',(1920,1040),(10,10,10))
    for j,im in enumerate(snapshots):sheet.paste(im,((j%3)*640,(j//3)*520))
    buf=io.BytesIO();sheet.save(buf,format='JPEG',quality=94);payloads['proof/contact.jpg']=buf.getvalue()
    payloads['tracking-review.mp4']=video.read_bytes();payloads['report.json']=canonical(report)
    return publish(destination,stage='tracking_review_export',payloads=payloads,
        inputs=[{'role':'source','artifact_id':pm['artifact_id']},{'role':'tracks','artifact_id':tm['artifact_id']}],
        config={'fps':5,'main_camera':'front','bottom_cameras':cameras,'positions_3d':False},
        code_revision=digest(Path(__file__).read_bytes()),runtime={'av':av.__version__})
