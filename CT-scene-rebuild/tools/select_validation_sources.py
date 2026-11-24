"""Freeze new-date sources using inventory + raw SEI only, before predictions."""
import argparse,json,math,re
from pathlib import Path
import av,numpy as np
from PIL import Image,ImageDraw,ImageFont
from ct_scene.telemetry import nal_units,sei_messages
from ct_scene.ingest import file_hash
from ct_scene.artifacts import canonical,digest


def raw_motion(path):
    speeds=[];motion_times=[]
    with av.open(str(path)) as c:
        stream=c.streams.video[0];extra=stream.codec_context.extradata
        if not extra or extra[0]!=1:return None
        length=(extra[4]&3)+1;duration=float(c.duration/av.time_base) if c.duration else 0
        for packet in c.demux(stream):
            if packet.pts is None:continue
            now=float(packet.pts*packet.time_base)
            try:messages=[m for n in nal_units(bytes(packet),length) for m in sei_messages(n)]
            except ValueError:continue
            for row in messages:
                field=row.get('fields',{}).get('vehicle_speed_mps',{})
                if field.get('valid'):
                    if now<=8:speeds.append(abs(field['value']))
                    if row.get('fields',{}).get('heading_deg',{}).get('valid'):motion_times.append(now)
    stamps=np.unique(motion_times)
    coverage=bool(len(stamps)>1 and stamps[0]<=.1 and duration-stamps[-1]<=.25 and np.max(np.diff(stamps))<=.2)
    return dict(duration_s=duration,speed_p90_mps=float(np.quantile(speeds,.9)) if speeds else None,full_motion_coverage=coverage,
                last_motion_s=float(stamps[-1]) if len(stamps) else None)


def decoded_nearest(container,t):
    previous=None
    for f in container.decode(video=0):
        if f.pts is None:continue
        now=float(f.pts*f.time_base)
        if now>=t:
            if previous is not None and abs(float(previous.pts*previous.time_base)-t)<=abs(now-t):f=previous
            return f.to_image(),f.pts,str(f.time_base)
        previous=f
    if previous:return previous.to_image(),previous.pts,str(previous.time_base)
    raise ValueError('No source frame')


def nearest(path,t):
    with av.open(str(path)) as c:
        stream=c.streams.video[0]
        try:c.seek(int(t/stream.time_base),stream=stream,backward=True)
        except av.FFmpegError:
            # Some valid fragmented clips have no usable seek index. Reopen
            # and decode from the first PTS instead of changing the source.
            with av.open(str(path)) as sequential:return decoded_nearest(sequential,t)
        return decoded_nearest(c,t)


p=argparse.ArgumentParser();p.add_argument('--dates',nargs='+',required=True);p.add_argument('--output',required=True);p.add_argument('--allow-sentry',action='store_true',help='Folder name is not a motion label; retain the same raw-SEI moving-source gate');p.add_argument('--clip-validation',action='store_true',help='Exclude previously extracted source hashes; dates are explicitly exposed, not new-date holdout');args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
inventory=[json.loads(l) for l in Path('runs/inventory-001/sources.jsonl').read_text(encoding='utf-8').splitlines()]
selected=[];rejections=[]
known=set();known_ids=set()
if args.clip_validation:
    for f in Path('runs').glob('**/source/frames.json'):
        with f.open('rb') as stream:prefix=stream.read(2048)
        known.update(x.decode() for x in re.findall(rb'"source_sha256"\s*:\s*"([a-f0-9]{64})"',prefix))
    for f in Path('runs').glob('*selection*/selection.json'):
        for source in json.loads(f.read_text()).get('selected',[]):
            known_ids.add(source['source_id'])
            value=source.get('source_sha256',{}).get('front')
            if value:known.add(value)
    (out/'excluded-known-source-hashes.json').write_bytes(canonical(sorted(known)))
for date in args.dates:
    candidates=[r for r in inventory if r['recording_date']==date and r['split']=='unassigned' and any((args.allow_sentry or 'RecentClips' in c['front']['path'] or 'SavedClips' in c['front']['path']) and {'front','back','left_repeater','right_repeater'}<=c.keys() for c in r['candidate_copies'])]
    if not candidates:rejections.append(dict(date=date,reason='no_unassigned_four_camera_source'));continue
    if args.clip_validation:candidates=[r for r in candidates if r['source_id'] not in known_ids]
    if not candidates:rejections.append(dict(date=date,reason='no_unseen_source_hash_after_exposure_audit'));continue
    order=list(dict.fromkeys([len(candidates)//2,0,len(candidates)-1,len(candidates)//4,3*len(candidates)//4]))
    if args.clip_validation:order+= [i for i in range(len(candidates)) if i not in order]
    found=None
    for i in order:
        r=candidates[i];copy=next((c for c in r['candidate_copies'] if {'front','back','left_repeater','right_repeater'}<=c.keys()),None)
        if not copy:continue
        summary=raw_motion(copy['front']['path'])
        if summary is None or summary['duration_s']<35 or summary['speed_p90_mps'] is None or summary['speed_p90_mps']<2 or not summary['full_motion_coverage']:
            rejections.append(dict(source_id=r['source_id'],reason='duration_or_raw_motion_selection',summary=summary));continue
        actual_front_hash=file_hash(Path(copy['front']['path']))
        if args.clip_validation and actual_front_hash in known:
            rejections.append(dict(source_id=r['source_id'],reason='previously_extracted_full_source_sha256'));continue
        found=dict(source_id=r['source_id'],date=date,source_files={c:v['path'] for c,v in copy.items()},source_sha256={c:file_hash(Path(v['path'])) for c,v in copy.items()},
                   first_s=0.,last_s=summary['duration_s'],selection=summary,prior_exposure='no_known_inventory_analysis; requires_history_audit',status='source_only_selected_before_predictions')
        break
    if found:selected.append(found)
manifest=dict(selected=selected,rejected_during_source_selection=rejections,predictions_viewed=False,
              code_snapshot_sha256=digest(b''.join(f.read_bytes() for f in sorted(Path('src/ct_scene').glob('*.py')))),
              prototype_config_sha256=file_hash(Path('configs/surrounding-prototype.json')),split='provisional_new_date_validation_not_claimed_independent_until_exposure_review')
if args.clip_validation:
    manifest.update(split='new_clip_validation_on_exposed_dates; not_six_new_date_holdout',known_source_hashes_excluded=len(known))
    for source in selected:source['prior_exposure']='source hash absent from v2 extracted sources and previous raw selections; recording date already exposed'
(out/'selection.json').write_bytes(canonical(manifest))
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',14)
for source in selected:
    sheet=Image.new('RGB',(1280,1440),'#171b20');frames=[]
    for i,target in enumerate(np.linspace(0,min(50.,source['last_s']-.25),6)):
        t=float(target)
        for j,camera in enumerate(['front','back','left_repeater','right_repeater']):
            im,pts,base=nearest(source['source_files'][camera],t);tile=im.resize((320,240));d=ImageDraw.Draw(tile);d.rectangle((0,0,320,23),fill='#171b20');d.text((3,3),f'{camera} target {t}s PTS {pts}',font=font,fill='white');sheet.paste(tile,(j*320,i*240))
            frames.append(dict(camera_id=camera,source_sha256=source['source_sha256'][camera],pts=pts,time_base=base,relative_request_s=t,space_id='raw',width=im.width,height=im.height))
    sheet.save(out/(source['source_id']+'-raw.jpg'),quality=93)
    (out/(source['source_id']+'-frames.json')).write_bytes(canonical(frames))
print(json.dumps(dict(selected=[r['source_id'] for r in selected],rejected=len(rejections),predictions_viewed=False),ensure_ascii=False))
