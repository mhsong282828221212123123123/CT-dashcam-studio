"""Raw-reviewed Oct02 side cars and truck; corrected independent lane labels."""
import argparse,json
from pathlib import Path
from ct_scene.artifacts import verify
from ct_scene.road_frame import coordinates
from ct_scene.road_geometry import lane_at
from review_guards import require_window

p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();manifest=verify(a.scene)
assert any(i['role']=='source' and i['artifact_id']=='66941f33999fa1733fb0624bc0a55be256379e446313eed9e3e0dfa489f0b498' for i in manifest['inputs'])
actors=json.loads((a.scene/'actors.json').read_text());road=json.loads((a.scene/'report.json').read_text())['road']
states=[json.loads(l) for l in (a.scene/'render-states.jsonl').read_text().splitlines()]
events=[('left_repeater-0002',4.1,10.5,0),('left_repeater-0004',10.7,20.1,0),('left_repeater-0015',22.3,27.9,0),('left_repeater-0019',34.1,40.9,1),('left_repeater-0021',34.5,39.3,0),('left_repeater-0010',36.9,39.,1)]
rows=[]
for key,start,end,expected_lane in events:
    actor=next((r for r in actors if key in r['local_ids']),None)
    checks={'source_identity_present':actor is not None}
    if key=='left_repeater-0010':checks['truck_body_class_preserved']=bool(actor and actor['model'] in ('truck','box_truck','heavy_truck'))
    samples=[s for s in actor['samples'] if start<=s['time_s']<=end] if actor else []
    lanes=[lane_at(road,*map(float,coordinates(road,s['position_W'])[:2])) for s in samples]
    checks['observed_contact_samples_in_source_lane']=bool(lanes and all(l==expected_lane for l in lanes))
    require_window(states,start,end)
    world=[next((v for v in s['actors'] if actor and v['id']==actor['global_id']),None) for s in states if start<=s['time_s']<=end]
    checks['full_window_world_presence']=bool(world and all(world))
    checks['observed_windows_solid']=bool(world and all(v and not v['ghost'] for v in world))
    rows.append(dict(source_id=key,source_window_s=[start,end],expected_lane=expected_lane,checks=checks,gate='PASS' if all(checks.values()) else 'FAIL'))
result=dict(gate='PASS' if all(r['gate']=='PASS' for r in rows) else 'FAIL',events=rows,source_evidence=['runs/oct02-left-source-contact-001.jpg','runs/side-lane-independent-review-003.jpg'],annotation_correction='0019 is second lane (index1), not first: original raw white-divider review at 35.2076s. Earlier first-lane PASS is invalid, previous result files preserved.',scope='Six selected side vehicles; four first-lane and two second-lane; not all vehicles or measured accuracy')
a.output.mkdir(exist_ok=False,parents=True);(a.output/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result));raise SystemExit(0 if result['gate']=='PASS' else 2)
