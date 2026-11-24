"""Read-only original video timing audit. No inferred sensor synchronization."""
from pathlib import Path
import bisect
import math
from .artifacts import canonical, digest, publish

CAMERAS = ('front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar')


def nearest_index(times, t):
    if not times or not math.isfinite(t):
        raise ValueError('No valid times or invalid request')
    i = bisect.bisect_left(times, t)
    return min({max(0, i-1), min(len(times)-1, i)}, key=lambda k: (abs(times[k]-t), k))


def discover(front):
    front = Path(front).resolve()
    if not front.name.endswith('-front.mp4') or not front.is_file():
        raise ValueError('Select an existing *-front.mp4 original')
    prefix = front.name[:-len('-front.mp4')]
    return {c: front.with_name(prefix+'-'+c+'.mp4') for c in CAMERAS
            if front.with_name(prefix+'-'+c+'.mp4').is_file()}


def file_hash(path):
    import hashlib
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def audit_pts(front, destination, duration_s=35, sample_hz=10, cancelled=lambda: False):
    import av
    from .contracts import FrameRecord
    if not math.isfinite(duration_s) or not math.isfinite(sample_hz) or duration_s <= 0 or not 0 < sample_hz <= 30:
        raise ValueError('Invalid audit interval or rate')
    cameras = discover(front)
    payloads, summary = {}, {}
    for name, path in cameras.items():
        if cancelled():
            raise InterruptedError('Timing audit cancelled')
        source_hash = file_hash(path)
        rows = []
        missing_pts = 0
        with av.open(str(path)) as container:
            stream = container.streams.video[0]
            dimensions = [stream.codec_context.width, stream.codec_context.height]
            for decode_index, frame in enumerate(container.decode(stream)):
                if cancelled():
                    raise InterruptedError('Timing audit cancelled')
                if frame.pts is None:
                    missing_pts += 1
                    continue
                base = frame.time_base
                t = float(frame.pts * base)
                FrameRecord(source_hash,name,decode_index,frame.pts,base.numerator,base.denominator)
                if rows and t > rows[0]['camera_time_s'] + duration_s:
                    break
                rows.append(dict(frame_id=f'{source_hash}:{frame.pts}:{base}', source_sha256=source_hash,
                                 camera_id=name, decode_index=decode_index, pts=frame.pts,
                                 time_base_num=base.numerator, time_base_den=base.denominator,
                                 camera_time_s=t, session_time_s=None, clock_status='unverified',
                                 keyframe=bool(frame.key_frame), decode_status='decoded'))
        if not rows:
            raise ValueError('No decoded timestamped frames: '+str(path))
        times = [r['camera_time_s'] for r in rows]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError('Non-increasing decoded PTS: '+str(path))
        selections = []
        for step in range(math.ceil(duration_s*sample_hz)):
            request = times[0] + step/sample_hz
            if request > times[-1]:
                break
            i = nearest_index(times, request)
            selections.append(dict(request_camera_time_s=request, selected_frame_id=rows[i]['frame_id'],
                                   error_s=times[i]-request))
        errors = sorted(abs(r['error_s']) for r in selections)
        summary[name] = dict(path=str(path), source_sha256=source_hash, dimensions=dimensions,
                             decoded_frames=len(rows), missing_pts=missing_pts,
                             first_camera_time_s=times[0], last_camera_time_s=times[-1],
                             samples=len(selections), sampling_error_p95_s=errors[math.ceil(.95*len(errors))-1],
                             sampling_error_max_s=max(errors), exposure_sync='NOT_EVALUATED',
                             session_clock='unverified', sei='NOT_IMPLEMENTED')
        payloads[name+'/frames.jsonl'] = b''.join(canonical(r)+b'\n' for r in rows)
        payloads[name+'/sampling.jsonl'] = b''.join(canonical(r)+b'\n' for r in selections)
    payloads['timing-report.json'] = canonical(summary)
    source_code = Path(__file__).parent
    revision = digest(b''.join(p.name.encode()+p.read_bytes() for p in sorted(source_code.glob('*.py'))))
    return publish(destination, stage='timing_audit', payloads=payloads, inputs=[],
                   config=dict(duration_s=duration_s, sample_hz=sample_hz,
                               sources={c:v['source_sha256'] for c,v in summary.items()}),
                   code_revision=revision, runtime=dict(av=av.__version__), cancelled=cancelled)
