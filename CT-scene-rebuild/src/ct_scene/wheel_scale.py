"""Conditional wheel/visual scale check, never surveyed metric accuracy."""
from pathlib import Path
import json
import io
import numpy as np
from .artifacts import publish,verify,canonical,digest


def unique_motion_rows(rows):
    """One motion estimate per PTS; retain the unmodified raw SEI evidence.

    Multiple near-identical SEI messages in one packet are one observation.
    Median continuous fields only when consistent; conflicting motion/gear
    stays an error. Never invent distinct measurement times for one PTS.
    """
    groups={}
    for row in rows:groups.setdefault(row['camera_time_s'],[]).append(row)
    output=[]
    for t,group in sorted(groups.items()):
        if len(group)==1:output.append(group[0]);continue
        result=dict(group[-1],fields={k:dict(v) for k,v in group[-1]['fields'].items()})
        for key,tolerance in [('vehicle_speed_mps',.5),('heading_deg',2.)]:
            fields=[r.get('fields',{}).get(key,{}) for r in group]
            values=[float(f['value']) for f in fields if f.get('valid')]
            if not values:continue
            if key=='heading_deg':values=np.degrees(np.unwrap(np.radians(values))).tolist()
            if not np.isfinite(values).all() or np.ptp(values)>tolerance:raise ValueError('Conflicting duplicate telemetry: '+key)
            field=next(dict(f) for f in fields if f.get('valid'));field['value']=float(np.median(values));result['fields'][key]=field
        gears={r.get('fields',{}).get('gear_state',{}).get('value') for r in group if r.get('fields',{}).get('gear_state',{}).get('valid')}
        if len(gears)>1:raise ValueError('Conflicting duplicate telemetry: gear_state')
        result['duplicate_motion_messages']=len(group);result['motion_resolution']='median_consistent_continuous_fields_at_one_pts';output.append(result)
    return output


class SpeedTrace:
    def __init__(self,rows):
        self.rows=unique_motion_rows(rows)
        self.times=np.array([r['camera_time_s'] for r in self.rows],float)
        self.speeds=np.array([r['fields']['vehicle_speed_mps']['value'] if r['fields']['vehicle_speed_mps']['valid'] else np.nan for r in self.rows],float)
        if len(self.times)<2 or not np.isfinite(self.times).all() or np.any(np.diff(self.times)<=0):raise ValueError('Invalid/duplicate telemetry times')

    def distance(self,start,end):
        if not self.times[0]<=start<=end<=self.times[-1]:raise ValueError('Telemetry interval outside observed range')
        left=max(0,np.searchsorted(self.times,start,side='right')-1);right=min(len(self.times)-1,np.searchsorted(self.times,end,side='left'))
        if not np.isfinite(self.speeds[left:right+1]).all() or np.any(np.diff(self.times[left:right+1])>.2):raise ValueError('Missing telemetry within interval')
        times=np.r_[start,self.times[(self.times>start)&(self.times<end)],end]
        speed=np.interp(times,self.times,self.speeds)
        return float(np.trapezoid(speed,times))


def run(sfm,telemetry,destination):
    import pycolmap as pc
    sfm,telemetry=Path(sfm),Path(telemetry);sm,tm=verify(sfm),verify(telemetry)
    index=json.loads((sfm/'input-index.json').read_text())
    rows=[json.loads(s) for s in (telemetry/'telemetry.jsonl').read_text().splitlines()]
    if {f['source_sha256'] for f in index}!={r['source_sha256'] for r in rows}:raise ValueError('Wheel trace/source mismatch')
    trace=SpeedTrace(rows)
    model=pc.Reconstruction(sfm/'reconstruction');by_name={f['sfm_name']:f for f in index}
    poses=sorted([(by_name[im.name]['camera_time_s'],im.projection_center(),im.name) for im in model.images.values()])
    intervals=[]
    for (t0,p0,n0),(t1,p1,n1) in zip(poses,poses[1:]):
        wheel=trace.distance(t0,t1);visual=float(np.linalg.norm(p1-p0))
        relevant=[r for r in rows if t0<=r['camera_time_s']<=t1]
        forward=bool(relevant) and all(r['fields']['gear_state']['valid'] and r['fields']['gear_state']['value']==1 for r in relevant)
        eligible=forward and wheel>.25 and wheel/(t1-t0)>.5 and visual>1e-8 and t1-t0<=2
        intervals.append(dict(start_camera_time_s=t0,end_camera_time_s=t1,wheel_signed_travel_m=wheel,
                              camera_path_arbitrary_units=visual,eligible_for_scale_fit=eligible))
    moving=[r for r in intervals if r['eligible_for_scale_fit']]
    groups=[]
    for row in moving:
        if not groups or row['start_camera_time_s']-groups[-1][-1]['end_camera_time_s']>2:groups.append([])
        groups[-1].append(row)
    report={'scale_status':'NOT_ESTIMATED','metric_accuracy':'NOT_EVALUATED','body_trajectory':'NOT_ESTIMATED',
            'warning':'Wheel scale prior and camera-centre consistency only. No surveyed distance, body origin or gravity frame.',
            'moving_groups':len(groups),'source_optical_gate':json.loads((sfm/'report.json').read_text())['gate']}
    if moving:
        fit=groups[0];test=[r for g in groups[1:] for r in g]
        if not test:
            fit=moving[:max(1,len(moving)//2)];test=moving[len(fit):]
        scale=sum(r['wheel_signed_travel_m'] for r in fit)/sum(r['camera_path_arbitrary_units'] for r in fit)
        predicted=sum(r['camera_path_arbitrary_units'] for r in test)*scale
        observed=sum(r['wheel_signed_travel_m'] for r in test)
        report.update(scale_status='estimated_from_wheel_prior',metres_per_sfm_unit=scale,
                      fit_intervals=len(fit),test_intervals=len(test),fit_wheel_travel_m=sum(r['wheel_signed_travel_m'] for r in fit),
                      test_visual_travel_scaled_m=predicted,test_wheel_travel_m=observed,
                      heldout_distance_fraction_error=abs(predicted-observed)/observed if observed>0 else None)
    data=io.BytesIO();np.savez_compressed(data,camera_times=np.array([p[0] for p in poses]),
                                        camera_centres_arbitrary=np.array([p[1] for p in poses]))
    return publish(destination,stage='wheel_visual_scale_check',payloads={'report.json':canonical(report),
                   'intervals.jsonl':b''.join(canonical(r)+b'\n' for r in intervals),'camera-centres.npz':data.getvalue()},
                   inputs=[{'role':'sfm','artifact_id':sm['artifact_id']},{'role':'telemetry','artifact_id':tm['artifact_id']}],
                   config={'fit_min_speed_mps':.5,'min_fit_travel_m':.25,'telemetry_max_gap_s':.2,
                           'speed_rounding':False,'speed_deadband_applied':False},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'pycolmap':pc.__version__})
