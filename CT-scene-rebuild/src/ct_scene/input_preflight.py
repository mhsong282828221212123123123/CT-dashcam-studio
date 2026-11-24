"""Cheap full-clip coverage checks before extracting frames or running models.

Packet presentation times establish container coverage, not sensor exposure sync.
"""
from pathlib import Path
import json,math,statistics
from .artifacts import canonical,publish,digest,verify
from .ingest import discover,file_hash

REQUIRED=('front','back','left_repeater','right_repeater')


def check_coverage(cameras,telemetry_rows,tolerance_s=.5,max_gap_s=1.):
    failures=[]
    if not set(REQUIRED)<=cameras.keys():
        return dict(gate='FAIL',failures=[dict(reason='missing_camera_channels',
                    cameras=sorted(set(REQUIRED)-cameras.keys()))])
    front=cameras['front'];start=front['first_pts_s'];end=front['end_pts_s']
    if not all(math.isfinite(t) for t in (start,end)) or end<=start:
        return dict(gate='FAIL',failures=[dict(reason='invalid_front_duration')])
    for camera in REQUIRED:
        row=cameras[camera]
        if row.get('missing_pts',0):failures.append(dict(reason='camera_missing_packet_pts',camera=camera))
        if row['first_pts_s']>start+tolerance_s or row['end_pts_s']<end-tolerance_s:
            failures.append(dict(reason='camera_does_not_cover_front',camera=camera,
                                 first_pts_s=row['first_pts_s'],end_pts_s=row['end_pts_s']))
        if row.get('max_gap_s',0)>max_gap_s:
            failures.append(dict(reason='camera_timestamp_gap',camera=camera,max_gap_s=row['max_gap_s']))
    usable=[]
    for row in telemetry_rows:
        if row.get('status')!='decoded_v1':continue
        fields=row.get('fields',{})
        if not all(fields.get(k,{}).get('valid') for k in ('vehicle_speed_mps','heading_deg')):continue
        t=row.get('camera_time_s')
        if isinstance(t,(int,float)) and math.isfinite(t):usable.append(t)
    times=sorted(set(usable));gaps=[b-a for a,b in zip(times,times[1:])]
    if not times:failures.append(dict(reason='no_usable_driving_telemetry'))
    else:
        if times[0]>start+tolerance_s or times[-1]<end-tolerance_s:
            failures.append(dict(reason='telemetry_does_not_cover_front',first_pts_s=times[0],last_pts_s=times[-1],required_end_s=end))
        if max(gaps,default=0)>max_gap_s:
            failures.append(dict(reason='telemetry_timestamp_gap',max_gap_s=max(gaps)))
    return dict(gate='FAIL' if failures else 'PASS',failures=failures,
                front_interval_s=[start,end],usable_telemetry_samples=len(times),
                telemetry_interval_s=[times[0],times[-1]] if times else None,
                tolerance_s=tolerance_s,max_gap_s=max_gap_s,
                scope='container/SEI coverage only; no sensor synchronization or decode quality claim')


def inspect_packets(path):
    import av
    times=[];durations=[];missing=0
    with av.open(str(path)) as container:
        stream=container.streams.video[0]
        for packet in container.demux(stream):
            if not packet.size:continue
            if packet.pts is None:missing+=1;continue
            times.append(float(packet.pts*packet.time_base))
            if packet.duration:durations.append(float(packet.duration*packet.time_base))
    if len(times)<2:raise ValueError('Not enough timestamped packets: '+str(path))
    times=sorted(set(times));gaps=[b-a for a,b in zip(times,times[1:])]
    period=statistics.median(durations or gaps)
    return dict(first_pts_s=times[0],last_pts_s=times[-1],end_pts_s=times[-1]+period,
                packet_samples=len(times),missing_pts=missing,max_gap_s=max(gaps,default=0),
                nominal_period_s=period)


def run(front,telemetry,output,source_hashes=None):
    tm=verify(Path(telemetry));sources=discover(front)
    hashes=source_hashes or {c:file_hash(p) for c,p in sources.items()}
    report=json.loads((Path(telemetry)/'sei-report.json').read_text())
    if hashes.get('front')!=report['source_sha256']:raise ValueError('Preflight telemetry belongs to another source')
    cameras={c:dict(path=str(p),source_sha256=hashes[c],**inspect_packets(p)) for c,p in sources.items() if c in REQUIRED}
    rows=[json.loads(l) for l in (Path(telemetry)/'telemetry.jsonl').read_text().splitlines()]
    result=check_coverage(cameras,rows);result['cameras']=cameras
    return publish(output,stage='input_preflight',payloads={'review.json':canonical(result)},
                   inputs=[dict(role='telemetry',artifact_id=tm['artifact_id'])],
                   config=dict(source_hashes=hashes,tolerance_s=.5,max_gap_s=1.),
                   code_revision=digest(Path(__file__).read_bytes()),runtime=dict(cuda=False))


def require_supported(directory):
    verify(Path(directory));report=json.loads((Path(directory)/'review.json').read_text())
    if report['gate']!='PASS':
        reasons=', '.join(r['reason']+(' ['+r['camera']+']' if 'camera' in r else '') for r in report['failures'])
        raise ValueError('원본/주행 데이터가 전체 구간을 지원하지 않습니다: '+reasons)
    return report
