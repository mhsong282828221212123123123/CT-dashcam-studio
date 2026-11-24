"""Explicit human review layer. Never feeds automatic estimation or scoring."""
import copy
import json
import math
from pathlib import Path
import numpy as np
from .artifacts import verify,canonical,publish,digest


def validate(layer,manifest,states):
    if not isinstance(layer,dict):raise ValueError('Correction document must be an object')
    if layer.get('schema')!='ct_scene.manual_corrections.v1':raise ValueError('Unknown correction schema')
    if layer.get('source_artifact_id')!=manifest['artifact_id']:raise ValueError('Correction belongs to a different automatic result')
    known={a['id'] for s in states for a in s['actors']}
    end=max(s['time_s'] for s in states);start=min(s['time_s'] for s in states)
    windows={}
    edits=layer.get('edits')
    if not isinstance(edits,list) or not edits:raise ValueError('At least one explicit edit required')
    for e in edits:
        if not isinstance(e,dict):raise ValueError('Correction edit must be an object')
        if set(e)!={'actor_id','reason','keyframes'}:raise ValueError('Only actor offsets are supported; ego/road changes forbidden')
        if e['actor_id'] not in known:raise ValueError('Unknown actor ID')
        if not isinstance(e['reason'],str) or not e['reason'].strip():raise ValueError('Correction reason required')
        keys=e['keyframes']
        if not isinstance(keys,list) or len(keys)<3:raise ValueError('Use zero-offset endpoints and at least one inner keyframe')
        for k in keys:
            if not isinstance(k,dict):raise ValueError('Correction keyframe must be an object')
            if set(k)!={'time_s','offset_W_xy_m','yaw_delta_deg'}:raise ValueError('Invalid correction keyframe')
            if not isinstance(k['offset_W_xy_m'],list) or len(k['offset_W_xy_m'])!=2:raise ValueError('XY offset required')
            values=[k['time_s'],*k['offset_W_xy_m'],k['yaw_delta_deg']]
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):raise ValueError('Finite numeric keyframes required')
        ts=[k['time_s'] for k in keys]
        if ts[0]<start or ts[-1]>end or any(b<=a for a,b in zip(ts,ts[1:])):raise ValueError('Ordered keyframes inside exported timeline required')
        if any(abs(v)>1e-9 for k in [keys[0],keys[-1]] for v in [*k['offset_W_xy_m'],k['yaw_delta_deg']]):raise ValueError('Endpoints must return to automatic pose to avoid jumps')
        for a,b in windows.setdefault(e['actor_id'],[]):
            if ts[0]<b and ts[-1]>a:raise ValueError('Overlapping edits on same actor')
        if not any(ts[0]<=s['time_s']<=ts[-1] and any(a['id']==e['actor_id'] for a in s['actors']) for s in states):raise ValueError('Actor absent throughout edit interval')
        windows[e['actor_id']].append((ts[0],ts[-1]))
    return layer


def apply(layer,state):
    result=copy.deepcopy(state);t=state['time_s']
    for edit in layer['edits']:
        keys=edit['keyframes'];ts=[k['time_s'] for k in keys]
        if not ts[0]<=t<=ts[-1]:continue
        for actor in result['actors']:
            if actor['id']!=edit['actor_id']:continue
            delta=np.array([np.interp(t,ts,[k['offset_W_xy_m'][i] for k in keys]) for i in range(2)])
            angle=float(np.interp(t,ts,[k['yaw_delta_deg'] for k in keys]))
            if max(abs(delta))<1e-9 and abs(angle)<1e-9:continue
            actor['automatic_position_W']=actor['position_W'][:];actor['automatic_yaw_rad']=actor['yaw_rad']
            actor['position_W']=(np.asarray(actor['position_W'])+np.r_[delta,0]).tolist()
            actor['yaw_rad']+=math.radians(angle);actor['position_support']='manual_review_correction'
            actor['manual_reason']=edit['reason'];result['manual_corrections']=True
    return result


def render(scene,base,layer_path,assets_root,config,output):
    import av
    from PIL import Image,ImageDraw,ImageFont
    from fractions import Fraction
    from .surrounding import SurroundRenderer
    scene,base,output=map(Path,[scene,base,output]);sm=verify(scene);bm=verify(base)
    if sm['stage']!='surrounding_world_prototype':raise ValueError('Select original automatic scene, not a manual derivative')
    if not any(i['artifact_id']==bm['artifact_id'] and i['role']=='frozen_ego_lead' for i in sm['inputs']):raise ValueError('Automatic scene/base mismatch')
    if output.exists():raise FileExistsError(output)
    states=[json.loads(s) for s in (scene/'render-states.jsonl').read_text(encoding='utf-8').splitlines()]
    layer=validate(json.loads(Path(layer_path).read_text(encoding='utf-8')),sm,states)
    corrected=[apply(layer,s) for s in states]
    if not any(s.get('manual_corrections') for s in corrected):raise ValueError('No rendered actor is changed by this layer')
    report=json.loads((scene/'report.json').read_text(encoding='utf-8'))
    trajectory=json.loads((base/'trajectory.json').read_text(encoding='utf-8'));times=[r['camera_time_s'] for r in trajectory]
    actors=json.loads((scene/'actors.json').read_text(encoding='utf-8'));models={a['global_id']:a['model'] for a in actors}
    assets={k:str(Path(assets_root)/('tesla_fsd_'+name+'.glb')) for k,name in [('ego','sedan'),('sedan','sedan'),('van','van'),('suv','suv'),('truck','small_truck'),('box_truck','box_truck'),('bus','bus'),('heavy_truck','heavy_truck')]}
    style=json.loads(Path(config).read_text(encoding='utf-8'))['render_style']
    renderer=SurroundRenderer(assets,report['road'],style)
    work=output.parent/(output.name+'-work');work.mkdir(parents=True,exist_ok=False);video=work/'surrounding-world.mp4'
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18);changed=0
    with av.open(str(scene/'surrounding-world.mp4')) as source,av.open(str(video),'w',options={'movflags':'+faststart'}) as dest:
        ss=source.streams.video[0];fps=ss.average_rate
        stream=dest.add_stream('libx264',rate=fps);stream.width=1280;stream.height=1000;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'}
        count=0
        for j,frame in enumerate(source.decode(video=0)):
            if j>=len(corrected):raise ValueError('Automatic video/state length mismatch')
            s=corrected[j];t=s['time_s'];ep=np.array([np.interp(t,times,[r['ego_position_W'][k] for r in trajectory]) for k in range(3)])
            ey=float(np.interp(t,times,np.unwrap([r['ego_yaw_rad'] for r in trajectory])))
            active=[dict(a,model=models[a['id']],position_W=np.asarray(a['position_W'])) for a in s['actors']]
            view=s['camera_view'];viewargs=dict(view_position=np.asarray(view['position_W']),view_yaw=view['yaw_rad']) if view['weight']>0 else {}
            image=renderer.frame_many(active,ep,ey,lamps=s.get('ego_lamps'),**viewargs)
            canvas=frame.to_image();canvas.paste(image,(0,0));draw=ImageDraw.Draw(canvas)
            draw.text((18,12),f'MANUAL REVIEW COPY | {t:.2f}s | excluded from automatic quality scores',font=font,fill=(255,200,100))
            names=[a['id'] for a in s['actors'] if a.get('manual_reason')]
            draw.text((18,40),'Manual actor offsets: '+(', '.join(names) or 'none at this time'),font=font,fill=(230,230,230))
            changed+=bool(names);outframe=av.VideoFrame.from_image(canvas);outframe.pts=j;outframe.time_base=Fraction(1,1)/fps
            for packet in stream.encode(outframe):dest.mux(packet)
            count+=1
        for packet in stream.encode():dest.mux(packet)
    if count!=len(corrected):raise ValueError('Incomplete manual video')
    with av.open(str(video)) as check:
        if sum(1 for _ in check.decode(video=0))!=count:raise ValueError('Manual video failed decode')
    report.update(manual_corrections=True,automatic_quality_score_eligible=False,manual_changed_frames=changed,
                  original_automatic_artifact_id=sm['artifact_id'],scope='Human-assisted review derivative; automatic source/road/ego preserved',
                  automatic_road_and_lane_audits='original artifact only; manual actor lane membership must be re-reviewed')
    return publish(output,stage='manual_review',payloads={'surrounding-world.mp4':video.read_bytes(),'report.json':canonical(report),
        'render-states.jsonl':b''.join(canonical(s)+b'\n' for s in corrected),'manual-corrections.json':canonical(layer)},
        inputs=[dict(role='automatic_scene',artifact_id=sm['artifact_id']),dict(role='frozen_ego',artifact_id=bm['artifact_id'])],
        config=dict(layer=layer,render_style=style),
        code_revision=digest(b''.join(Path(__file__).with_name(n).read_bytes() for n in ['manual_review.py','surrounding.py','prototype_front.py','road_geometry.py','road_frame.py','pocket_lane.py','ego_lamps.py'])),
        model_hashes=sorted({digest(Path(p).read_bytes()) for p in [*assets.values(),renderer.ego_visual]}),
        runtime=dict(av=av.__version__,render_device=renderer.ctx.info['GL_RENDERER'],cuda=False))
