"""Frozen-plane/frozen-optics transfer to later raw side frames.

Tracks are flow proposals requiring image review; they are not manual GT.
"""
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .artifacts import verify,publish,canonical,digest
from .manual_rig_probe import flow_step,project
from .plane_pose_probe import plane_points,best_pose
from .ingest import file_hash


def dense_samples(source,anchor,duration,direction=1):
    import av
    if not np.isfinite(duration) or not 0<duration<=3 or direction not in (-1,1):raise ValueError('Invalid dense probe interval')
    sha=file_hash(Path(source))
    if sha!=anchor['source_sha256']:raise ValueError('Original source hash differs from anchor')
    with av.open(str(source)) as container:
        first=None;started=False;backward=[]
        for frame in container.decode(video=0):
            if frame.pts is None:continue
            t=float(frame.pts*frame.time_base)
            if first is None:first=t
            lower=anchor['camera_time_s']-(duration if direction<0 else 0)
            upper=anchor['camera_time_s']+(duration if direction>0 else 0)
            if t<lower-1e-8:continue
            if direction>0 and not started:
                if frame.pts!=anchor['pts']:raise ValueError('Exact original anchor PTS missing')
                started=True
            if t>upper+1e-8:break
            f=dict(anchor,frame_id=f'{sha}:{frame.pts}:{frame.time_base}',pts=frame.pts,
                time_base_num=frame.time_base.numerator,time_base_den=frame.time_base.denominator,
                camera_time_s=t,relative_request_s=t-first,image=None)
            if direction>0:yield f,frame.to_image().convert('RGB')
            else:
                backward.append((f,frame.to_image().convert('RGB')))
                if frame.pts==anchor['pts']:started=True
        if not started:raise ValueError('Original video ended before anchor')
        if direction<0:
            if not backward or backward[-1][0]['pts']!=anchor['pts']:raise ValueError('Exact backward anchor missing')
            yield from reversed(backward)


def run(package,tracks_artifact,plane_artifact,destination,duration=2,raw_source=None,direction=1):
    package,tracks_artifact,plane_artifact=Path(package),Path(tracks_artifact),Path(plane_artifact)
    if Path(destination).exists():raise FileExistsError(destination)
    pm,tm,cm=[verify(p) for p in (package,tracks_artifact,plane_artifact)]
    if not any(i['artifact_id']==pm['artifact_id'] for i in tm['inputs']):raise ValueError('Track/source mismatch')
    if not any(i['artifact_id']==tm['artifact_id'] for i in cm['inputs']):raise ValueError('Plane/track mismatch')
    if not np.isfinite(duration) or duration<=0 or duration>3:raise ValueError('Use at most three seconds for reviewed transfer probe')
    if direction not in (-1,1) or (direction<0 and raw_source is None):raise ValueError('Backward probe requires original video')
    ref=json.loads((tracks_artifact/'reference.json').read_text());profiles=json.loads((tracks_artifact/'lens-candidates.json').read_text())
    plane=json.loads((plane_artifact/'report.json').read_text());candidate=json.loads((plane_artifact/'target-optical-candidate.json').read_text())
    camera=ref['target_camera'];start=ref['target_request_s'];frames=json.loads((package/'frames.json').read_text())
    sequence=sorted((f for f in frames if f['camera_id']==camera and start-.01<=f['relative_request_s']<=start+duration+.01),key=lambda f:f['camera_time_s'])
    if not sequence or abs(sequence[0]['relative_request_s']-start)>.01:raise ValueError('Anchor missing')
    xyz=plane_points([p['source'] for p in ref['points']],np.array(plane['front_normal']),profiles[ref['source_camera']])
    fit=np.array([p['split']=='fit' for p in ref['points']]);xy=np.array([p['target'] for p in ref['points']],np.float32);active=np.ones(len(xy),bool)
    samples=dense_samples(raw_source,sequence[0],duration,direction) if raw_source else ((f,Image.open(package/f['image']).convert('RGB')) for f in sequence)
    rows=[];tiles=[];previous=None;payloads={};cv2.setNumThreads(4)
    for f,image in samples:
        if raw_source:
            f=dict(f,image=f'original-frames/{len(rows):04d}.jpg');buffer=io.BytesIO();image.save(buffer,format='JPEG',quality=95);payloads[f['image']]=buffer.getvalue()
        gray=np.asarray(image.convert('L'))
        if previous is not None:
            # These specifically identified background corners are visible in the
            # anchor; broad detector boxes would wrongly discard the facade.
            # Later flow is NOT declared visible GT or used as an accepted rig.
            xy,active=flow_step(previous,gray,xy,active,np.full_like(gray,255))
        previous=gray;row={'frame':f,'active':active.tolist(),'points_raw':xy.tolist(),'models':{}}
        overlay=image.copy();draw=ImageDraw.Draw(overlay);predicted=None
        for name,profile in [('prior_fixed',profiles[camera]),('candidate_fixed',candidate)]:
            fit_mask=active&fit;check_mask=active&~fit
            result={'fit_count':int(fit_mask.sum()),'check_count':int(check_mask.sum()),'gate':'NOT_ENOUGH_POINTS'}
            if fit_mask.sum()>=4 and check_mask.sum()>=3:
                try:
                    _,pose=best_pose(xyz[fit_mask],xy[fit_mask],profile)
                    predicted_here=project(xyz[active],pose,profile);errors=np.linalg.norm(predicted_here-xy[active],axis=1)
                    errors_check=errors[(~fit)[active]]
                    result.update(median_px=float(np.median(errors_check)),p95_px=float(np.percentile(errors_check,95)),pose_C_plane=pose.tolist())
                    result['gate']='PASS_CONDITIONAL_FLOW_CHECK_ONLY' if fit_mask.sum()>=6 and check_mask.sum()>=5 and np.median(errors_check)<=3 and np.percentile(errors_check,95)<=8 else 'FAIL'
                    if name=='candidate_fixed':predicted=predicted_here
                except ValueError as exc:result.update(gate='FAIL',reason=str(exc))
            row['models'][name]=result
        indices=np.flatnonzero(active)
        for j,n in enumerate(indices):
            u,v=xy[n];draw.ellipse((u-2,v-2,u+2,v+2),outline='lime',width=1);draw.text((u+2,v-9),str(n),fill='yellow')
            if predicted is not None:pu,pv=predicted[j];draw.line((u,v,pu,pv),fill='red',width=1)
        if len(indices):
            lo=np.maximum(np.floor(xy[indices].min(0)-[25,25]).astype(int),[0,0]);hi=np.minimum(np.ceil(xy[indices].max(0)+[25,25]).astype(int),[image.width,image.height])
            crop=overlay.crop((*lo,*hi));crop.thumbnail((480,280))
        else:crop=overlay.resize((320,240))
        tile=Image.new('RGB',(480,320),'#171717');tile.paste(crop,((480-crop.width)//2,28));d=ImageDraw.Draw(tile)
        result=row['models']['candidate_fixed'];d.text((5,5),f'{f["camera_time_s"]:.3f}s FLOW PROPOSALS / fixed focal',fill='white')
        d.text((5,305),f'fit {result["fit_count"]}, check {result["check_count"]} / {result["gate"]}',fill='white');tiles.append(tile);rows.append(row)
    for page in range((len(tiles)+14)//15):
        page_tiles=tiles[page*15:(page+1)*15];sheet=Image.new('RGB',(1440,((len(page_tiles)+2)//3)*320),'#111')
        for j,tile in enumerate(page_tiles):sheet.paste(tile,((j%3)*480,(j//3)*320))
        buf=io.BytesIO();sheet.save(buf,format='JPEG',quality=95);payloads[f'proof/page-{page:02d}.jpg']=buf.getvalue()
    summaries={name:{'pass_frames':sum(r['models'][name]['gate']=='PASS_CONDITIONAL_FLOW_CHECK_ONLY' for r in rows),
        'frames':len(rows),'after_anchor_pass_frames':sum(r['models'][name]['gate']=='PASS_CONDITIONAL_FLOW_CHECK_ONLY' for r in rows[1:])} for name in ('prior_fixed','candidate_fixed')}
    report={'summaries':summaries,'direction':direction,'focal_fit_time_s':start,'sampling':'all_original_decoded_pts' if raw_source else 'annotation_grid_5hz','no_optical_refit':True,'plane_fixed':True,'scope':'Same development facade, manually seeded raw flow proposals; not independent journey GT or accepted rig',
        'visibility':'Flow FB and bounds only; no bbox mask, requires manual image review','rig':'NOT_ACCEPTED'}
    payloads.update({'report.json':canonical(report),'observations.jsonl':b''.join(canonical(r)+b'\n' for r in rows)})
    return publish(destination,stage='side_optical_transfer_probe',payloads=payloads,
        inputs=[{'role':role,'artifact_id':m['artifact_id']} for role,m in zip(['source','front_tracks','plane_candidate'],[pm,tm,cm])],
        config={'duration_s':duration,'candidate':candidate,'prior':profiles[camera],'no_refit':True,'raw_source':str(raw_source) if raw_source else None,'direction':direction},
        code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__,'device':'cpu'})
