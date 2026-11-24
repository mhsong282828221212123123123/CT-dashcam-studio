"""Check the source-reviewed pocket and preserve main-road/vehicle invariants."""
import json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify
from ct_scene.road_geometry import pocket_width,lane_at


out=Path('runs/pocket-source-review-001');results={}
for name,old_name in [('oct21','oct21-zoom130-001'),('sep17','sep17-actor-lanes-001'),('oct02','oct02-actor-lanes-001')]:
    old=Path('runs')/old_name;new=Path(f'runs/{name}-pocket-003');verify(new)
    load=lambda p:json.loads(p.read_text(encoding='utf-8'))
    before,after=[load(p/'report.json') for p in (old,new)];road=after['road']
    checks=dict(main_boundaries_unchanged=before['road']['boundaries_y']==road['boundaries_y'],
                no_occupancy_rule=not load(new/'left-pocket-evidence.json')['fit']['occupancy_used'],
                no_observed_ghost_conflicts=not load(new/'observation-world-audit.json')['observed_ghost_conflicts'],
                full_export_decoded=after['all_frames_decoded'])
    old_ego,new_ego=[load(p/'ego-lane-state.json') for p in (old,new)]
    checks['ego_lane_ids_unchanged']=[r['lane_id'] for r in old_ego['records']]==[r['lane_id'] for r in new_ego['records']]
    checks['ego_lateral_coordinates_unchanged']=np.allclose([r['road_lateral_m'] for r in old_ego['records']],[r['road_lateral_m'] for r in new_ego['records']],atol=1e-10,rtol=0)
    if name=='oct21':
        checks['one_source_reviewed_pocket']=len(road['left_pockets'])==1
        p=road['left_pockets'][0];knots=p['width_knots']
        checks['taper_growth_and_return']=knots[0][1]==knots[-1][1]==0 and knots[1][1]>1.8 and knots[-2][1]>1.8 and all(b[0]>a[0] for a,b in zip(knots,knots[1:]))
        checks['zero_width_outside_support']=pocket_width(road['left_pockets'],p['start_x']-1)==pocket_width(road['left_pockets'],p['end_x']+1)==0
        checks['multi_frame_evidence']=len(p['evidence_frame_ids'])>=3
        actors=load(new/'actors.json');old_actors=load(old/'actors.json');by_new={a['global_id']:a for a in actors}
        checks['existing_actor_ids_retained']=all(a['global_id'] in by_new for a in old_actors)
        checks['existing_actor_positions_unchanged']=all(np.allclose([r['position_W'] for r in a['samples']],[r['position_W'] for r in by_new[a['global_id']]['samples']],atol=1e-9,rtol=0) for a in old_actors)
        pocket_actors=[a for a in actors if a['lane_index']==-1]
        # These four queues were manually checked against source frames;
        # production code does not contain these fixture IDs.
        checks['four_reviewed_queue_vehicles']={a['global_id'] for a in pocket_actors}=={'front-0062','front-0070','front-0078','front-0082'}
        inside=[lane_at(road,r['position_W'][0],r['position_W'][1]-road['heading_evidence']['slope']*r['position_W'][0])==-1 for a in pocket_actors for r in a['samples']]
        checks['queue_centres_inside_pocket']=all(inside)
        results[name]=dict(scope='source reviewed positive; estimated geometry',checks=checks,pocket_profile=knots,queue_ids=[a['global_id'] for a in pocket_actors])
    else:
        checks['no_added_pocket']=not road['left_pockets']
        results[name]=dict(scope='pocket topology regression only; existing road/pose limitations not fixed',checks=checks)
    assert all(checks.values()),(name,checks)
(out/'review.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
