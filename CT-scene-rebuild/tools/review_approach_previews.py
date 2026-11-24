"""Selected source-backed development events, not a whole-scene GT score."""
import argparse,json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,canonical
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import boundaries_at,pocket_width,lane_at
ap=argparse.ArgumentParser();ap.add_argument('--sep07',type=Path,required=True);ap.add_argument('--aug28',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
checks={};records={}
for name,path,when,keys in [('sep07',a.sep07,20.,['front-0015','left_repeater-0012']),('aug28',a.aug28,13.4,['front-0020','front-0033'])]:
    m=verify(path);road=json.loads((path/'report.json').read_text())['road'];actors=json.loads((path/'actors.json').read_text());mapping={key:x for x in actors for key in x['local_ids']};states=[json.loads(s) for s in (path/'render-states.jsonl').read_text().splitlines()];state=min(states,key=lambda r:abs(r['time_s']-when));by={r['id']:r for r in state['actors']};proof=[]
    for key in keys:
        actor=mapping.get(key);row=by.get(actor['global_id']) if actor else None;q=coordinates(road,row['position_W']) if row else None;lane=lane_at(road,float(q[0]),float(q[1])) if row else None
        proof.append(dict(local_id=key,global_id=actor['global_id'] if actor else None,solid=bool(row and not row['ghost']),lane=lane))
    checks[name+'_source_left_vehicles_solid_in_auxiliary_band']=all(r['solid'] and r['lane']==-1 for r in proof)
    checks[name+'_source_time_matches']=abs(state['time_s']-when)<.05
    lead=next((x for x in actors if x.get('frozen_lead')),None)
    baseline='runs/new-date-suite-006/2026-09-07_18-09-51/scene' if name=='sep07' else 'runs/branch-orientation-suite-005/aug28'
    old=next((x for x in json.loads((Path(baseline)/'actors.json').read_text()) if x.get('frozen_lead')),None)
    selected=lambda actor:{r['frame_id']:(r['position_W'],r['yaw_rad']) for r in actor['samples'] if r.get('camera_id')=='front'} if actor else {}
    checks[name+'_trusted_front_lead_unchanged']=selected(lead)==selected(old)
    if name=='sep07':
        ego_station=float(coordinates(road,state['camera_view']['position_W'])[0]);physical=len(boundaries_at(road,ego_station))-1+int(pocket_width(road['left_pockets'],ego_station)>=1.8)
        checks['sep07_source_minimum_five_physical_lanes']=physical>=5
        right=mapping.get('right_repeater-0004');r=by.get(right['global_id']) if right else None;leadrow=by.get(lead['global_id']) if lead else None
        rr=coordinates(road,r['position_W']) if r else None;ll=coordinates(road,leadrow['position_W']) if leadrow else None
        checks['sep07_right_hyundai_still_one_lane_right']=bool(rr is not None and ll is not None and lane_at(road,float(rr[0]),float(rr[1]))-lane_at(road,float(ll[0]),float(ll[1]))==1)
        records['sep07_physical_lanes_at_20s']=physical
    else:
        checks['aug28_selected_opposing_car_excluded']='front-0043' not in mapping
        oldroad=json.loads((Path(baseline)/'report.json').read_text())['road'];extract=lambda r:[(p['start_x'],p['end_x'],p['width_knots']) for p in r['left_pockets']]
        checks['aug28_existing_pocket_geometry_preserved']=extract(road)==extract(oldroad)
    records[name]=dict(artifact_id=m['artifact_id'],targets=proof)
report=dict(scope='selected raw-reviewed development approach events only; no exhaustive recall or metric ground truth claim',checks=checks,records=records,status='PASS' if all(checks.values()) else 'FAIL');(a.output/'review.json').write_bytes(canonical(report));print(json.dumps(dict(status=report['status'],checks=checks),indent=2));raise SystemExit(0 if all(checks.values()) else 2)
