import json,numpy as np
from pathlib import Path
from collections import defaultdict
from probe_side_yaw import track_axis_candidates
from ct_scene.surrounding import asset_class

P=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras'];C=json.loads(Path('configs/surrounding-prototype.json').read_text())['cameras'];out={}
for case,scene,tracks in [('aug28','midnight-aug28-012','G:/3d/CT-scene-rebuild/runs/new-date-recovery-suite-001/2026-08-28_08-55-51/tracks'),('highway','midnight-highway-011','G:/3d/CT-scene-rebuild/runs/fresh-nov01-highway-001/tracks'),('junction','midnight-junction-011','G:/3d/CT-scene-rebuild/runs/fresh-oct18-junction-002/tracks'),('oct22','midnight-oct22-011','G:/3d/CT-scene-rebuild/runs/new-date-recovery-suite-001/2026-10-22_08-11-29/tracks')]:
    ss=[json.loads(x) for x in Path('runs',scene,'render-states.jsonl').read_text().splitlines()];ts=np.array([s['time_s'] for s in ss]);ys=np.array([s['ego_yaw_rad'] for s in ss]);dims=json.loads(Path('runs',scene,'report.json').read_text())['vehicle_dimensions_m'];groups=defaultdict(list)
    for line in Path(tracks,'observations.jsonl').read_text().splitlines():
        r=json.loads(line);groups[r['local_track_id']].append(r)
    out[case]={}
    for cam in ['left_repeater','right_repeater']:
        rows=[]
        for key,rs in groups.items():
            if rs[0]['camera_id']!=cam or set(r['class_id'] for r in rs)&{0,1,3,15,16}:continue
            model,sup=asset_class(rs,cam,dims)
            windows=track_axis_candidates(rs,P[cam],ts,ys,C[cam]['yaw_deg'],dims[model][2])
            if windows:rows.append(dict(case=case,camera=cam,track_id=key,model=model,median_yaw_deg=float(np.median([r['yaw_deg'] for r in windows])),windows=windows))
        out[case][cam]=rows
        print(case,cam,[(r['track_id'],round(r['median_yaw_deg'],1),len(r['windows'])) for r in rows]);print('MED',float(np.median([r['median_yaw_deg'] for r in rows])) if rows else None)
Path('runs/session-20261126-two-hours/raw-side-yaw-probe.json').write_text(json.dumps(out,indent=2))
