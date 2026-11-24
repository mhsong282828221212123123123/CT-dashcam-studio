"""Source front paint in the existing curved reference; no pose/rig edits."""
import argparse,json
from pathlib import Path
from collections import defaultdict,Counter
import numpy as np
from ct_scene.pocket_lane import paint_observations,stations,fit_pockets

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
frames=load(a.case/'source/frames.json');obs=defaultdict(list)
for r in map(json.loads,(a.case/'tracks/observations.jsonl').read_text().splitlines()):obs[r['frame_id']].append(r)
poses=load(a.case/'base/trajectory.json');times=np.array([p['camera_time_s'] for p in poses]);ego=np.array([p['ego_position_W'] for p in poses]);yaw=np.unwrap([p['ego_yaw_rad'] for p in poses]);road=load(a.scene/'report.json')['road']
profiles=load(Path('configs/multicam-optical-candidates.json'))['cameras'];mounts=load(Path('configs/surrounding-prototype.json'))['cameras']
d=paint_observations(a.case/'source',frames,obs,times,ego,yaw,profiles,mounts,road.get('heading_evidence',{}).get('slope',0.),road=road,road_masks=a.case/'road-masks')
d['stations']=stations(d['points'])
from ct_scene.pocket_lane import fit_curved_pockets
d['fit']=fit_curved_pockets(d,road)
(a.output/'evidence.json').write_text(json.dumps(d),encoding='utf-8')
print(json.dumps(dict(counts=dict(Counter((r['camera_id']+'_'+r['kind']) for r in d['points'])),yellow_stations=[r for r in d['stations'] if r['kind']=='yellow'],fit=d['fit'])))
