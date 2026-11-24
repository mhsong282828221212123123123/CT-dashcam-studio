"""Whole-export numerical/event review. Does not substitute for source labels."""
from collections import defaultdict
from pathlib import Path
import json,math
import av,numpy as np
from .artifacts import verify,canonical
from .collision_constraints import audit as collision_audit


def source_presence_screen(report,states):
    """Reject a nearly empty world with plentiful camera observations.

    This is a collapse smoke check, not detection recall or source accuracy.
    Distant objects need not all enter the displayed world.
    """
    counts=report.get('observation_world_coverage',{}).get('front',{})
    observed=counts.get('observed_samples');covered=counts.get('covered_in_world_state')
    if observed is None or covered is None or not states:
        return dict(gate='NOT_EVALUATED',reason='source_coverage_counts_unavailable')
    empty_fraction=sum(not row['actors'] for row in states)/len(states)
    coverage=covered/max(1,observed)
    collapse=observed>=100 and coverage<.05 and empty_fraction>=.9
    return dict(gate='REJECTED' if collapse else 'NOT_FLAGGED',
                reason='plentiful_front_observations_but_nearly_empty_world' if collapse else 'no_severe_world_collapse_detected',
                front_observations=observed,front_world_covered=covered,front_coverage_fraction=coverage,
                empty_frame_fraction=empty_fraction,source_accuracy='NOT_EVALUATED')


def review(directory,destination):
    directory=Path(directory);manifest=verify(directory)
    states=[json.loads(l) for l in (directory/'render-states.jsonl').read_text().splitlines()]
    report=json.loads((directory/'report.json').read_text());tracks=defaultdict(list);failures=[]
    timestamps=[r['time_s'] for r in states]
    if not states or not np.isfinite(timestamps).all() or np.any(np.diff(timestamps)<=0):raise ValueError('Invalid render clock')
    for state in states:
        seen=set()
        for a in state['actors']:
            key=a['id'];t=state['time_s'];p=np.array(a['position_W']);yaw=a['yaw_rad']
            if key in seen:failures.append(dict(time_s=t,id=key,reason='duplicate_global_id'))
            seen.add(key)
            if not np.isfinite(p).all() or not math.isfinite(yaw):failures.append(dict(time_s=t,id=key,reason='nonfinite_pose'));continue
            if a.get('source_visible') and a['ghost']:failures.append(dict(time_s=t,id=key,reason='observed_ghost_conflict'))
            tracks[key].append((t,p,yaw))
    worst=[]
    for key,rows in tracks.items():
        for a,b in zip(rows,rows[1:]):
            dt=b[0]-a[0]
            if dt>.11:continue
            speed=float(np.linalg.norm(b[1]-a[1])/dt);dyaw=abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))
            if speed>65 or dyaw>math.radians(25):
                event=dict(time_s=b[0],id=key,reason='motion_discontinuity_review_required',speed_mps=speed,yaw_step_deg=math.degrees(dyaw))
                failures.append(event);worst.append(event)
    with av.open(str(directory/'surrounding-world.mp4')) as c:
        frames=0;last=None
        for frame in c.decode(video=0):frames+=1;last=float(frame.time)
    if frames!=report['frames']:failures.append(dict(reason='incomplete_export',decoded=frames,expected=report['frames']))
    collisions=collision_audit(states,report['vehicle_dimensions_m']) if report.get('vehicle_dimensions_m') else dict(gate='NOT_EVALUATED',reason='Legacy export does not declare model dimensions and ego pose')
    if collisions['gate']=='FAIL':
        failures.extend(dict(reason='vehicle_body_overlap_review_required',**event) for event in collisions['body_overlap_intervals'])
    result=dict(artifact_id=manifest['artifact_id'],frames_decoded=frames,last_frame_s=last,render_frames=len(states),
                last_five_seconds_audited=True,actors=len(tracks),failure_count=len(failures),failures=failures,
                worst_motion_events=sorted(worst,key=lambda r:r['speed_mps']+r['yaw_step_deg'],reverse=True)[:5],
                numerical_gate='PASS' if not failures else 'FAIL',collision_review=collisions,source_presence_screen=source_presence_screen(report,states),source_relation_quality='NOT_EVALUATED',
                warning='Coverage, identity, lane membership and source agreement require independent original-video review.')
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=False);(destination/'review.json').write_bytes(canonical(result))
    return result
