"""Read-only rejection guards; never fit or render a scene."""
import math


def require_same_base(scene_manifest,base_manifest,allow_change=False):
    expected=[r['artifact_id'] for r in scene_manifest['inputs'] if r['role']=='frozen_ego_lead']
    if len(expected)!=1:raise ValueError('Exactly one frozen ego/lead base required')
    if not allow_change and base_manifest['artifact_id']!=expected[0]:
        raise ValueError('Frozen base differs from scene input; explicit --allow-base-change required')


def require_cases(actual,expected):
    if not expected or len(set(expected))!=len(expected):raise ValueError('Nonempty unique expected cases required')
    if len(set(actual))!=len(actual) or set(actual)!=set(expected):raise ValueError('Missing, duplicate or unexpected cases')


def require_window(states,start,end,step=.05):
    if not math.isfinite(start+end) or end<=start:raise ValueError('Invalid required source interval')
    times=[s['time_s'] for s in states if start-step/2<=s['time_s']<=end+step/2]
    if not times or times[0]>start+step or times[-1]<end-step:raise ValueError('Required source interval missing')
    if any(not math.isfinite(t) for t in times) or any(b<=a or b-a>step*1.51 for a,b in zip(times,times[1:])):raise ValueError('Source interval has gaps or duplicate times')
    return len(times)


def require_full_clock(states,source_report,decoded_frames,fps=20):
    duration=float(source_report['source_duration_s']);start=float(source_report['source_first_pts_s'])
    if not math.isfinite(duration) or duration<=0:raise ValueError('Invalid source duration')
    expected=math.ceil(duration*fps)
    if len(states)!=expected or decoded_frames!=expected:raise ValueError('Export does not cover original source duration')
    for i,s in enumerate(states):
        if not math.isfinite(s['time_s']) or abs(s['time_s']-(start+i/fps))>1e-5:raise ValueError('Export clock differs from source grid')
    tail=max(start,start+duration-5)
    count=require_window(states,tail,start+(expected-1)/fps) if expected>1 else 1
    return dict(source_duration_s=duration,expected_frames=expected,last_five_seconds_interval_s=[tail,start+(expected-1)/fps],last_five_seconds_frames=count)


def source_exit_code(results):
    return 0 if results and all(r.get('gate')=='PASS' for r in results) else 2


def require_input_artifact(scene_manifest,input_manifest,role):
    ids=[r['artifact_id'] for r in scene_manifest.get('inputs',[]) if r['role']==role]
    if len(ids)!=1 or input_manifest.get('artifact_id')!=ids[0]:
        raise ValueError('Wrong '+role+' artifact; same date or filename is insufficient')
