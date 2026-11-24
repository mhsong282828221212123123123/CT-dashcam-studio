"""Selected source witnesses plus trusted-base contracts on the current suites."""
import argparse,json,math
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify,canonical
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import boundaries_at,pocket_width
ap=argparse.ArgumentParser();ap.add_argument('--branch',type=Path);ap.add_argument('--four',type=Path);ap.add_argument('--manifest',type=Path);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text());states=lambda p:[json.loads(s) for s in (p/'render-states.jsonl').read_text().splitlines()]
manifest_cases={r['case_id']:r for r in load(a.manifest)['cases']} if a.manifest else {}
def scene_for(group,key):
    if a.manifest:return Path(manifest_cases[group+'/'+key]['scene'])
    return a.branch/key if group=='branch' else a.four/key/'scene'
def base_for(group,key):
    if a.manifest:return Path(manifest_cases[group+'/'+key]['base'])
    return a.branch/(key+'-base') if group=='branch' else a.four/key/'base'
checks={};details={}
jobs=[(n,scene_for('branch',n),base_for('branch',n)) for n in ['oct26','aug28','aug26','oct22']]+[(sid,scene_for('four',sid),base_for('four',sid)) for sid in ['2026-08-16_14-59-58','2026-08-25_08-20-56','2026-09-02_12-07-46','2026-09-07_18-09-51']]
for name,scene,base in jobs:
    verify(scene);bm=verify(base);actors=load(scene/'actors.json');poses=load(base/'trajectory.json');lead=next((r for r in actors if r.get('frozen_lead')),None);expected={r['frame_id']:r for r in poses if r.get('fitted_actor_position_W') is not None};actual={r['frame_id']:r for r in lead['samples']} if lead else {};errors=[]
    for fid,row in expected.items():
        got=actual.get(fid)
        if got is None or np.linalg.norm(np.array(got['position_W'])-row['fitted_actor_position_W'])>1e-8 or abs(np.angle(np.exp(1j*(got['yaw_rad']-row['actor_yaw_rad']))))>1e-8:errors.append(fid)
    checks[name+'_trusted_base_anchors']=not errors;details[name]=dict(trusted_source_anchors=len(expected),anchor_errors=len(errors),base_artifact=bm['artifact_id'])

scene=scene_for('branch','oct26');actors=load(scene/'actors.json');road=load(scene/'report.json')['road'];truck=next((r for r in actors if 'front-0083' in r['local_ids']),None);window=[s for s in states(scene) if 49<=s['time_s']<=50.2];rows=[next((r for r in s['actors'] if truck and r['id']==truck['global_id']),None) for s in window];present=[r for r in rows if r]
from review_guards import require_window
require_window(states(scene),49,50.2)
checks['oct26_raw_crossing_truck_solid']=bool(truck and len(present)==len(window) and all(not r['ghost'] for r in present))
if present:
    p=np.array([r['position_W'] for r in present]);direction=np.arctan2(*(p[-1]-p[0])[:2][::-1]);alignment=np.cos(np.array([r['yaw_rad'] for r in present])-direction);q=coordinates(road,p);checks['oct26_raw_left_to_right_and_body_direction']=bool(q[-1,1]-q[0,1]<-4 and np.min(alignment)>.7);details['crossing_truck']=dict(samples=len(rows),lateral_delta_m=float(q[-1,1]-q[0,1]),body_alignment_min=float(np.min(alignment)))

scene=scene_for('branch','oct22');actors=load(scene/'actors.json');ids={k for r in actors for k in r['local_ids']};checks['oct22_source_crossing_car_and_bus_retained']={'front-0093','front-0095'}<=ids;checks['oct22_source_opposing_taxi_and_lexus_excluded']=not ids.intersection({'front-0089','front-0022'})
events=load(scene_for('branch','aug28')/'ego-lane-state.json')['events'];checks['aug28_source_own_change_completion_preserved']=any(e.get('direction')=='left' and abs(e.get('completion_s',-100)-26.9949)<.05 for e in events)

scene=scene_for('four','2026-08-25_08-20-56');actors=load(scene/'actors.json');car=next((r for r in actors if {'front-0044','right_repeater-0050'}<=set(r['local_ids'])),None);checks['aug25_source_same_plate_camera_handoff']=car is not None
if car:
    rows=[r for r in car['samples'] if 35.2<=r['time_s']<=55.8];p=np.array([r['position_W'] for r in rows]);yaw=np.unwrap([r['yaw_rad'] for r in rows]);checks['aug25_parked_body_does_not_drift_or_turn']=bool(np.max(np.linalg.norm(p-p[0],axis=1))<.05 and np.ptp(yaw)<np.radians(1));details['parked_source_vehicle']=dict(samples=len(rows),position_span_m=np.ptp(p,axis=0).tolist(),body_yaw_span_deg=float(np.degrees(np.ptp(yaw))))

scene=scene_for('four','2026-09-02_12-07-46');road=load(scene/'report.json')['road'];window=[s for s in states(scene) if s['time_s']<=10];counts=[]
require_window(states(scene),0,10)
for s in window:
    station=float(coordinates(road,s['camera_view']['position_W'])[0]);counts.append(len(boundaries_at(road,station))-1+int(pocket_width(road['left_pockets'],station)>=1.8))
checks['sep02_source_empty_two_lane_approach']=bool(all(c==2 for c in counts) and all(not s['actors'] for s in window))
report=dict(scope='selected previously raw-reviewed development witnesses; not exhaustive recall or metric accuracy',checks=checks,details=details,status='PASS' if all(checks.values()) else 'FAIL');(a.output/'review.json').write_bytes(canonical(report));print(json.dumps(dict(status=report['status'],checks=checks),indent=2));raise SystemExit(0 if all(checks.values()) else 2)
