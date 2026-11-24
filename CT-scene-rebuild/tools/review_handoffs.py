"""Source-reviewed handoff fixtures, coordinate invariants and gap bounds."""
import json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import verify


out=Path('runs/handoff-review-001');results={}
for name,previous,current in [('oct21','oct21-road-lighter-001','oct21-handoff-002'),
                              ('sep17','sep17-rear-reference-003','sep17-handoff-002'),
                              ('pilot','wp06-pilot-regression-002','pilot-handoff-002')]:
    old,new=[Path('runs')/p for p in (previous,current)];verify(new)
    load=lambda p:json.loads(p.read_text(encoding='utf-8'))
    before,after=[load(p/'actors.json') for p in (old,new)]
    old_anchor=next(a for a in before if a['frozen_lead']);anchor=next(a for a in after if a['frozen_lead'])
    samples={r['frame_id']:r for r in anchor['samples'] if r['support']!='offline_two_endpoint_gap'}
    checks=dict(ego_exact=(old/'ego-lane-state.json').read_bytes()==(new/'ego-lane-state.json').read_bytes(),
                road_exact=load(old/'report.json')['road']==load(new/'report.json')['road'],
                anchor_observed_positions_exact=all(samples[r['frame_id']]['position_W']==r['position_W'] for r in old_anchor['samples']),
                no_observed_ghost_conflicts=not load(new/'observation-world-audit.json')['observed_ghost_conflicts'],
                full_export_decoded=load(new/'report.json')['all_frames_decoded'])
    member={key:a['global_id'] for a in after for key in a['local_ids']}
    if name=='oct21':
        checks.update(lead_front_side_front=member['front-0001']==member['right_repeater-0011']==member['front-0060'],
                      white_side_front=member['right_repeater-0010']==member['front-0071'],
                      lead_and_white_distinct=member['front-0001']!=member['front-0071'],
                      different_dark_car_distinct=member['front-0001']!=member['right_repeater-0009'],
                      benz_retained=member['left_repeater-0001']==member['front-0016'])
    if name=='pilot':checks['palisade_retained']=member['front-0003']==member['right_repeater-0010']
    gap_count=0;max_speed=0
    for a in after:
        rows=a['samples']
        for gap in a.get('handoff_gaps',[]):
            gap_count+=1
            segment=[r for r in rows if gap['start_s']-1e-7<=r['time_s']<=gap['end_s']+1e-7]
            points=np.array([r['position_W'] for r in segment]);t=np.array([r['time_s'] for r in segment])
            assert np.isfinite(points).all() and np.all(np.diff(t)>0)
            assert np.all(points>=np.minimum(points[0],points[-1])-1e-6) and np.all(points<=np.maximum(points[0],points[-1])+1e-6)
            speed=np.linalg.norm(np.diff(points,axis=0),axis=1)/np.diff(t)
            max_speed=max(max_speed,float(max(speed)))
            assert max(speed)<45, (name,a['global_id'],gap,max(speed))
    checks['gap_paths_bounded']=True
    results[name]=dict(checks=checks,completed_gaps=gap_count,max_gap_speed_m_s=max_speed,
                       frames=load(new/'report.json')['frames'])
    assert all(checks.values()),results[name]
(out/'review.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
