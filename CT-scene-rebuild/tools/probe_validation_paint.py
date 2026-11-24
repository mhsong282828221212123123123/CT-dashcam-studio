"""Preserve raw camera paint observations for junction development review."""
import json
from collections import defaultdict
from pathlib import Path
import numpy as np
from ct_scene.curved_road import paint_segments
from ct_scene.road_frame import coordinates

out=Path('runs/new-date-paint-probe-001');out.mkdir(exist_ok=False)
cfg=json.loads(Path('configs/surrounding-prototype.json').read_text())
profile=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras']['front']
for sid in ['2026-08-17_12-37-37','2026-10-26_12-01-20']:
    case=Path('runs/new-date-suite-002')/sid;rows=json.loads((case/'base/trajectory.json').read_text())
    t=np.array([r['camera_time_s'] for r in rows]);p=np.array([r['ego_position_W'] for r in rows]);yaw=np.array([r['ego_yaw_rad'] for r in rows])
    obs=defaultdict(list)
    for r in map(json.loads,(case/'tracks/observations.jsonl').read_text().splitlines()):obs[r['frame_id']].append(r)
    segments=paint_segments(case/'source',json.loads((case/'source/frames.json').read_text()),obs,t,p,yaw,profile,cfg['cameras']['front'],case/'road-masks')
    (out/(sid+'.json')).write_text(json.dumps(segments))
    road=json.loads((case/'scene/report.json').read_text())['road'];bins=defaultdict(list)
    for s in segments:
        if s['kind']!='crossbar':continue
        local=coordinates(road,s['points_W_xy']);x=float(np.mean(local[:,0]));d=float(np.mean(local[:,1]))
        if abs(d)>12:continue
        bins[round(x/5)*5].append(s)
    print(sid,[(x,len({r['frame_id'] for r in rs}),round(min(r['time_s'] for r in rs),1),round(max(r['time_s'] for r in rs),1)) for x,rs in sorted(bins.items())],flush=True)
