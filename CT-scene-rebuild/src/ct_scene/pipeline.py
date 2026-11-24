"""Resumable full-source orchestration. Existing editor algorithms are not used."""
from pathlib import Path
import json,time,uuid
from contextlib import contextmanager
import av
from .artifacts import verify,canonical,digest
from .ingest import discover,file_hash


def analysis_revision(directory=None):
    """UI/export-only edits must not invalidate an unchanged analysis cache."""
    directory=Path(directory) if directory is not None else Path(__file__).parent
    presentation={'__init__.py','__main__.py','cli.py','review_app.py','manual_review.py','scene_replay.py'}
    modules={p.name:digest(p.read_bytes()) for p in sorted(directory.glob('*.py')) if p.name not in presentation}
    return digest(canonical(modules))


def write_state(path,state):
    path=Path(path);tmp=path.with_name('.'+path.name+'.'+uuid.uuid4().hex[:8]+'.tmp')
    tmp.write_bytes(canonical(state))
    try:
        for attempt in range(8):
            try:tmp.replace(path);return
            except PermissionError:
                if attempt==7:raise
                time.sleep(.05*(attempt+1))
    finally:
        if tmp.exists():tmp.unlink()


@contextmanager
def output_lock(path):
    """OS lock is released even if a worker is killed; no stale PID guessing."""
    path=Path(path);stream=path.open('a+b');locked=False
    try:
        if path.stat().st_size==0:stream.write(b'0');stream.flush()
        stream.seek(0)
        try:
            if __import__('os').name=='nt':
                import msvcrt
                msvcrt.locking(stream.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as error:raise ValueError('This output already has an active analysis worker') from error
        locked=True;yield
    finally:
        if locked:
            stream.seek(0)
            if __import__('os').name=='nt':
                import msvcrt
                msvcrt.locking(stream.fileno(),msvcrt.LK_UNLCK,1)
            else:
                import fcntl
                fcntl.flock(stream.fileno(),fcntl.LOCK_UN)
        stream.close()


def analyze(front,output,detector,road_model,lens_config,prototype_config,assets_root,osm=None,backend='directml'):
    output=Path(output).resolve();sources=discover(front)
    if any(output==p.parent or output in p.parents for p in sources.values()):raise ValueError('Output cannot contain original video sources')
    output.mkdir(parents=True,exist_ok=True)
    with output_lock(output/'.pipeline.lock'):
        return _analyze(front,output,detector,road_model,lens_config,prototype_config,assets_root,osm,backend)


def _analyze(front,output,detector,road_model,lens_config,prototype_config,assets_root,osm=None,backend='directml'):
    from .annotation import prepare
    from .telemetry import audit_sei
    from .mask_prepass import run as detect
    from .tracking import run as track
    from .full_timeline import run as base
    from .background_rotation import run as own_rotation
    from .image_vehicle_heading import run as body_headings
    from .road_perception import run as roads
    from .osm_context import run as map_context
    from .surrounding import run as surround
    from .quality_review import review
    from .input_preflight import run as preflight,require_supported
    front=Path(front).resolve();output=Path(output).resolve();sources=discover(front)
    if not {'front','back','left_repeater','right_repeater'}<=sources.keys():raise ValueError('Four matching original camera channels required')
    if any(output==p.parent or output in p.parents for p in sources.values()):raise ValueError('Output cannot contain original video sources')
    output.mkdir(parents=True,exist_ok=True)
    orientation_model=Path(detector).resolve().parent/'orientation-review/regressor_mobilenetv3.onnx'
    if not orientation_model.is_file():orientation_model=None
    config=dict(source_hashes={c:file_hash(p) for c,p in sources.items()},detector_sha256=file_hash(Path(detector)),road_model_sha256=file_hash(Path(road_model)),
                lens_sha256=file_hash(Path(lens_config)),prototype_sha256=file_hash(Path(prototype_config)),
                osm_sha256=file_hash(Path(osm)) if osm else None,assets={p.name:file_hash(p) for p in Path(assets_root).glob('*.glb')},backend=backend,
                orientation_model_sha256=file_hash(orientation_model) if orientation_model else None,
                algorithm_revision=analysis_revision())
    key=digest(canonical(config));state_path=output/'pipeline.json'
    if state_path.exists():
        state=json.loads(state_path.read_text(encoding='utf-8'))
        if state['input_config_sha256']!=key:raise ValueError('Different inputs/configuration: use a new output directory')
    else:state=dict(input_config_sha256=key,config=config,completed=[],status='running',quality_gate='NOT_EVALUATED')
    def save():
        write_state(state_path,state)
    save()
    def stage(name,call):
        dest=output/name;state.update(status='running',current_stage=name);save();print(json.dumps(dict(stage=name,state='starting')),flush=True)
        if dest.exists():
            manifest=verify(dest)
            print(json.dumps(dict(stage=name,state='verified_cache',artifact_id=manifest['artifact_id'])),flush=True)
        else:
            work=output/(name+'-work')
            if work.exists():
                if work.is_symlink() or not work.resolve().is_relative_to(output):raise ValueError('Unsafe unfinished work directory')
                work.rename(output/(name+'-interrupted-'+uuid.uuid4().hex[:8]))
            manifest=call(dest)
        state['completed']=list(dict.fromkeys(state['completed']+[name]));save();return dest
    with av.open(str(front)) as c:duration=float(c.duration/av.time_base) if c.duration else 60.
    try:
        telemetry=stage('telemetry',lambda p:audit_sei(front,p,duration))
        coverage=stage('input-preflight',lambda p:preflight(front,telemetry,p,config['source_hashes']))
        require_supported(coverage)
        package=stage('source',lambda p:prepare(front,p,duration,5))
        detections=stage('detections',lambda p:detect(package,detector,p,backend))
        tracks=stage('tracks',lambda p:track(package,detections,p))
        rotation=stage('ego-rotation',lambda p:own_rotation(package,tracks,telemetry,lens_config,p))
        frozen=stage('base',lambda p:base(package,tracks,telemetry,lens_config,front,'auto',p,assets_root,ego_rotation=rotation))
        headings=stage('image-headings',lambda p:body_headings(package,tracks,orientation_model,lens_config,p,backend)) if orientation_model else None
        road_masks=stage('road-masks',lambda p:roads(package,road_model,p,backend))
        context=stage('osm-context',lambda p:map_context(telemetry,osm,p)) if osm else None
        assets={k:str(Path(assets_root)/('tesla_fsd_'+name+'.glb')) for k,name in [('ego','sedan'),('van','van'),('suv','suv'),('sedan','sedan'),('truck','small_truck'),('box_truck','box_truck'),('bus','bus')]}
        result=stage('scene',lambda p:surround(package,tracks,frozen,lens_config,prototype_config,assets,p,road_masks,context,telemetry=telemetry,image_headings=headings))
        quality=output/'numerical-review'
        checks=json.loads((quality/'review.json').read_text()) if quality.exists() else review(result,quality)
        state.update(status='complete',current_stage=None,numerical_gate=checks['numerical_gate'],quality_gate='WORLD_COVERAGE_REVIEW_REQUIRED' if checks.get('source_presence_screen',{}).get('gate')=='REJECTED' else 'SOURCE_REVIEW_REQUIRED',
                     source_presence_screen=checks.get('source_presence_screen',{'gate':'NOT_EVALUATED'}),video=str(result/'surrounding-world.mp4'),completed_utc_s=time.time());save();return state
    except BaseException as error:
        state.update(status='interrupted' if isinstance(error,(KeyboardInterrupt,SystemExit)) else 'failed',error=str(error));save();raise
