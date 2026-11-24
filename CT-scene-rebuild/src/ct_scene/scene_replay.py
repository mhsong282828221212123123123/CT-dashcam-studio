"""Render a published world-state timeline without inference or pose changes."""
from pathlib import Path
from fractions import Fraction
import copy,json,math,time
import numpy as np
from .artifacts import verify,publish,canonical,digest
from .ingest import file_hash

ASSETS={'ego':'sedan','sedan':'sedan','van':'van','suv':'suv','truck':'small_truck',
        'box_truck':'box_truck','bus':'bus','heavy_truck':'heavy_truck'}


def validate_states(states,report):
    if not states or len(states)!=report['frames']:raise ValueError('Incomplete frozen scene timeline')
    previous=None
    for state in states:
        if not {'time_s','ego_position_W','ego_yaw_rad','camera_view','actors'}<=state.keys():
            raise ValueError('This older result lacks the frozen ego state required for replay; use a current analysis result')
        t=state['time_s']
        if not math.isfinite(t) or (previous is not None and abs(t-previous-.05)>1e-5):
            raise ValueError('Replay requires a contiguous 20fps scene timeline')
        previous=t
        values=[*state['ego_position_W'],state['ego_yaw_rad']]
        if not np.isfinite(values).all():raise ValueError('Invalid frozen ego pose')
        keys=[a['id'] for a in state['actors']]
        if len(set(keys))!=len(keys):raise ValueError('Duplicate frozen actor ID')
        for actor in state['actors']:
            if actor.get('model') not in ASSETS:raise ValueError('Frozen actor model missing or unknown; use a current analysis result')
            if not np.isfinite([*actor['position_W'],actor['yaw_rad']]).all():raise ValueError('Invalid frozen actor pose')
    return states


def render_state(renderer,state):
    """Only create renderer-owned copies; no geometry stage is called."""
    actors=[dict(copy.deepcopy(a),position_W=np.asarray(a['position_W'],float)) for a in state['actors']]
    view=state['camera_view']
    viewargs=dict(view_position=np.asarray(view['position_W']),view_yaw=view['yaw_rad']) if view['weight']>0 else {}
    return renderer.frame_many(actors,np.asarray(state['ego_position_W']),state['ego_yaw_rad'],
                               lamps=copy.deepcopy(state.get('ego_lamps')),**viewargs)


def run(scene,assets_root,prototype_config,output):
    import av
    from PIL import ImageDraw,ImageFont
    from .surrounding import SurroundRenderer
    scene,assets_root,output=map(Path,[scene,assets_root,output]);manifest=verify(scene)
    if manifest['stage'] not in ('surrounding_world_prototype','scene_replay','road_display_revision'):
        raise ValueError('Replay requires an automatic scene, not a manual correction')
    if output.exists():raise FileExistsError(output)
    states_bytes=(scene/'render-states.jsonl').read_bytes()
    states=validate_states([json.loads(l) for l in states_bytes.splitlines()],json.loads((scene/'report.json').read_text()))
    report=json.loads((scene/'report.json').read_text());style=json.loads(Path(prototype_config).read_text())['render_style']
    assets={key:str(assets_root/('tesla_fsd_'+name+'.glb')) for key,name in ASSETS.items()}
    renderer=SurroundRenderer(assets,copy.deepcopy(report['road']),style)
    initial_state_hash=digest(canonical(states));initial_road_hash=digest(canonical(report['road']))
    work=output.parent/(output.name+'-work');work.mkdir(parents=True,exist_ok=False)
    video=work/'surrounding-world.mp4';strip_records=[];started=time.monotonic();count=0
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
    with av.open(str(scene/'surrounding-world.mp4')) as source,av.open(str(video),'w',options={'movflags':'+faststart'}) as dest:
        ss=source.streams.video[0]
        if ss.average_rate!=20:raise ValueError('Frozen video is not 20fps')
        stream=dest.add_stream('libx264',rate=20);stream.width=1280;stream.height=1000
        stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'}
        for j,frame in enumerate(source.decode(video=0)):
            if j>=len(states):raise ValueError('Frozen video/state length mismatch')
            if abs(float(frame.time)-j/20)>1e-5:raise ValueError('Frozen video/state clock mismatch')
            state=states[j];canvas=frame.to_image()
            if canvas.size!=(1280,1000):raise ValueError('Frozen four-channel composition has unexpected size')
            strip_hash=digest(canvas.crop((0,720,1280,1000)).tobytes())
            canvas.paste(render_state(renderer,state),(0,0))
            if digest(canvas.crop((0,720,1280,1000)).tobytes())!=strip_hash:raise ValueError('Replay changed source strip')
            strip_records.append(dict(time_s=state['time_s'],parent_video_time_s=float(frame.time),decoded_strip_sha256=strip_hash))
            draw=ImageDraw.Draw(canvas)
            draw.text((18,12),f'SURROUNDING WORLD | {state["time_s"]:05.2f}s | {len(state["actors"])} vehicles',font=font,fill=renderer.text_color)
            draw.text((18,40),'Frozen scene replay | camera strip retained | estimated geometry',font=font,fill=renderer.secondary_text_color)
            encoded=av.VideoFrame.from_image(canvas);encoded.pts=j;encoded.time_base=Fraction(1,20)
            for packet in stream.encode(encoded):dest.mux(packet)
            count+=1
            if j%200==0:print('Frozen scene render',j,'/',len(states),flush=True)
        for packet in stream.encode():dest.mux(packet)
    if count!=len(states):raise ValueError('Incomplete frozen video')
    if initial_state_hash!=digest(canonical(states)) or initial_road_hash!=digest(canonical(report['road'])):
        raise ValueError('Renderer modified frozen scene geometry')
    with av.open(str(video)) as check:
        if sum(1 for _ in check.decode(video=0))!=count:raise ValueError('Replay decode incomplete')
    proof=dict(parent_scene_artifact_id=manifest['artifact_id'],frames=count,
               render_states_bytes_sha256=digest(states_bytes),geometry_unchanged=True,
               road_unchanged=True,model_inference_calls=0,render_elapsed_s=time.monotonic()-started,
               strip_role='Parent decoded four-camera strip retained before encoding; lossy re-encode is not byte-identical RGB',
               strip_records=strip_records)
    report.update(replay=True,replay_parent_scene_artifact_id=manifest['artifact_id'],
                  replay_geometry_unchanged=True,replay_style=style,replay_inference_calls=0)
    payloads={'surrounding-world.mp4':video.read_bytes(),'report.json':canonical(report),
              'render-states.jsonl':states_bytes,'replay-review.json':canonical(proof),'display-style.json':canonical(style)}
    for name in ('actors.json','ego-lane-state.json','actor-lane-state.json','prediction-display-review.json','road-display-review.json','blindspot-inference-review.json','branch-hints-review.json','local-road-extent-review.json','camera-crossing-road-review.json'):
        if (scene/name).is_file():payloads[name]=(scene/name).read_bytes()
    return publish(output,stage='scene_replay',payloads=payloads,
                   inputs=[dict(role='frozen_scene',artifact_id=manifest['artifact_id'])],
                   config=dict(render_style=style,parent_scene_artifact_id=manifest['artifact_id']),
                   code_revision=digest(b''.join(Path(__file__).with_name(n).read_bytes() for n in ('scene_replay.py','surrounding.py','prototype_front.py','road_geometry.py','road_frame.py','pocket_lane.py','ego_lamps.py','junction_display.py','junction_branches.py'))),
                   model_hashes=sorted({file_hash(Path(p)) for p in [*assets.values(),renderer.ego_visual]}),
                   runtime=dict(av=av.__version__,render_device=renderer.ctx.info['GL_RENDERER'],cuda=False))
