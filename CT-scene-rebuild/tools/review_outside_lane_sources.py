"""Two original-image witnesses for the reported road insertion failure."""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import canonical,verify
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_at

p=argparse.ArgumentParser();p.add_argument('--aug20',type=Path,required=True)
p.add_argument('--aug26',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(exist_ok=False)
checks={};details={}
for name,path,key,when in [('gas_station_parked_car',a.aug20,'front-0018',23.3967),
                           ('opposite_approach_car',a.aug26,'front-0006',15.7904)]:
    verify(path);road=json.loads((path/'report.json').read_text())['road']
    actors=json.loads((path/'actors.json').read_text())
    actor=next((x for x in actors if key in x['local_ids']),None)
    rows=actor['samples'] if actor else []
    sample=min(rows,key=lambda r:abs(r['time_s']-when)) if rows else None
    lane=None;offset=None
    if sample:
        q=coordinates(road,sample['position_W']);lane=lane_at(road,float(q[0]),float(q[1]));offset=float(q[1])
    checks[name+'_source_sample_present']=bool(sample and abs(sample['time_s']-when)<.06)
    checks[name+'_not_inserted_into_driving_lane']=bool(sample and lane is None)
    checks[name+'_lane_prior_disabled']=bool(sample and sample.get('lane_prior_applied') is False)
    details[name]=dict(source_track=key,source_time_s=when,predicted_lane=lane,lateral_m=offset,
                       source_evidence='runs/outside-to-lane-source-001.jpg')
road=json.loads((a.aug20/'report.json').read_text())['road']
actors=json.loads((a.aug20/'actors.json').read_text())
linked=next((x for x in actors if {'front-0017','right_repeater-0068'}<=set(x['local_ids'])),None)
sample=min(linked['samples'],key=lambda r:abs(r['time_s']-57.)) if linked else None
lane=None
if sample:
    q=coordinates(road,sample['position_W']);lane=lane_at(road,float(q[0]),float(q[1]))
checks['confirmed_front_to_right_camera_identity_preserved']=linked is not None
checks['source_right_lane_car_not_displaced_outside']=bool(sample and abs(sample['time_s']-57.)<.12 and lane==1)
details['right_lane_negative_control']=dict(source_time_s=57.,predicted_lane=lane,
                                           source_evidence='Original front/right-repeater handoff plus raw4 at 57s; candidate004 visibly moved this real lane car outside')
result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,details=details,
            scope='Two post-failure raw-reviewed development witnesses; not exhaustive source accuracy')
(a.output/'review.json').write_bytes(canonical(result));print(json.dumps(result))
raise SystemExit(0 if result['status']=='PASS' else 2)
