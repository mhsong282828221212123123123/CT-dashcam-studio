"""Raw-only annotation preparation. No model predictions or metric assumptions."""
from pathlib import Path
import json
import math
from .artifacts import canonical, digest, publish, verify
from .ingest import discover, file_hash


def prepare(front, destination, duration_s=35, sample_hz=5):
    import av
    if not math.isfinite(duration_s) or duration_s <= 0 or not math.isfinite(sample_hz) or not 0 < sample_hz <= 30:
        raise ValueError('Invalid duration/rate')
    payloads, frames = {}, []
    for camera, path in discover(front).items():
        sha = file_hash(path)
        with av.open(str(path)) as container:
            first, previous, step = None, None, 0
            count = math.ceil(duration_s*sample_hz)
            for frame in container.decode(video=0):
                if frame.pts is None:
                    continue
                t = float(frame.pts*frame.time_base)
                if first is None:
                    first = t
                while step < count and t-first >= step/sample_hz:
                    target = step/sample_hz
                    selected = frame
                    if previous is not None and abs(previous[0]-first-target) <= abs(t-first-target):
                        selected = previous[1]
                    selected_t = float(selected.pts*selected.time_base)
                    index = len(frames)
                    image = f'images/{index:05d}.jpg'
                    import io
                    buffer = io.BytesIO()
                    selected.to_image().save(buffer, format='JPEG', quality=85)
                    payloads[image] = buffer.getvalue()
                    frames.append(dict(frame_id=f'{sha}:{selected.pts}:{selected.time_base}',
                                       image=image, camera_id=camera, source_sha256=sha,
                                       pts=selected.pts, time_base_num=selected.time_base.numerator,
                                       time_base_den=selected.time_base.denominator,
                                       camera_time_s=selected_t, session_time_s=None,
                                       relative_request_s=target, clock_status='unverified',
                                       width=selected.width, height=selected.height, space_id='raw'))
                    step += 1
                previous = (t, frame)
                if step >= count:
                    break
    payloads['frames.json'] = canonical(frames)
    template = Path(__file__).with_name('annotation.html').read_text(encoding='utf-8')
    payloads['annotate.html'] = template.replace('/*FRAME_DATA*/[]', canonical(frames).decode()).encode('utf-8')
    return publish(destination, stage='annotation_source', payloads=payloads, inputs=[],
                   config={'duration_s':duration_s,'sample_hz':sample_hz,
                           'source_hashes':sorted({r['source_sha256'] for r in frames})},
                   code_revision=digest(Path(__file__).read_bytes()+template.encode()), runtime={'av':av.__version__})


def validate_annotations(package, annotations):
    verify(package)
    frames = json.loads((Path(package)/'frames.json').read_text(encoding='utf-8'))
    index = {r['frame_id']:r for r in frames}
    data = json.loads(Path(annotations).read_text(encoding='utf-8'))
    completed = data.get('reviewed_frames', [])
    if len(completed) != len(set(completed)) or any(f not in index for f in completed):
        raise ValueError('Invalid or duplicate reviewed frame ID')
    seen = set()
    for row in data['objects']:
        f = index[row['frame_id']]
        if row['frame_id'] not in completed:
            raise ValueError('Frame not explicitly reviewed')
        key = row['frame_id'], row['object_id']
        if key in seen or not row['object_id'].strip():
            raise ValueError('Duplicate or blank object identity')
        seen.add(key)
        box = row['bbox_xyxy']
        if len(box) != 4 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in box):
            raise ValueError('Invalid raw bbox')
        x1,y1,x2,y2 = box
        if not (0 <= x1 < x2 <= f['width'] and 0 <= y1 < y2 <= f['height']):
            raise ValueError('Raw bbox outside frame')
        if row.get('reviewed') is not True:
            raise ValueError('Unreviewed labels cannot be scored')
    return {'labels':len(seen),'annotated_unique_frames':len({x[0] for x in seen}),
            'explicitly_reviewed_frames':len(completed),
            'source_unique_frames':len(index),'quality_gate':'NOT_EVALUATED'}


def refresh_ui(package, destination):
    """Publish a new UI revision without changing original source artifacts."""
    package=Path(package)
    previous=verify(package)
    frames=json.loads((package/'frames.json').read_text(encoding='utf-8'))
    payloads={r['path']:(package/r['path']).read_bytes() for r in previous['payloads'] if r['path']!='annotate.html'}
    template=Path(__file__).with_name('annotation.html').read_text(encoding='utf-8')
    payloads['annotate.html']=template.replace('/*FRAME_DATA*/[]',canonical(frames).decode()).encode()
    return publish(destination, stage='annotation_source', payloads=payloads,
                   inputs=[{'role':'original_annotation_package','artifact_id':previous['artifact_id']}],
                   config={'source_artifact':previous['artifact_id'],'ui_sha256':digest(template.encode())},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={})
