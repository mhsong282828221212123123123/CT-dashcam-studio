"""Camera paint and OSM residuals for a reproducible road-section failure."""
import argparse,json
from collections import defaultdict
from pathlib import Path
import numpy as np
from ct_scene.curved_road import paint_segments
from ct_scene.road_frame import coordinates

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
a.output.mkdir(exist_ok=False);case=a.case
cfg=json.loads(Path('configs/surrounding-prototype.json').read_text());profile=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras']['front']
trajectory=json.loads((case/'base/trajectory.json').read_text());t=np.array([r['camera_time_s'] for r in trajectory]);ego=np.array([r['ego_position_W'] for r in trajectory]);yaw=np.array([r['ego_yaw_rad'] for r in trajectory])
obs=defaultdict(list)
for r in map(json.loads,(case/'tracks/observations.jsonl').read_text().splitlines()):obs[r['frame_id']].append(r)
segments=paint_segments(case/'source',json.loads((case/'source/frames.json').read_text()),obs,t,ego,yaw,profile,cfg['cameras']['front'],case/'road-masks')
(a.output/'segments.json').write_text(json.dumps(segments));road=json.loads((case/'scene/report.json').read_text())['road'];maps=json.loads((case/'osm-context/context.json').read_text());elocal=coordinates(road,ego);bins=defaultdict(list)
for r in segments:
    if r['kind']!='crossbar':continue
    p=coordinates(road,r['points_W_xy']);delta=p[1]-p[0];s,d=np.mean(p[:,:2],axis=0)
    if abs(delta[1])<3 or abs(delta[0])>abs(delta[1])*.4 or not road['boundaries_y'][-1]-2<d<road['boundaries_y'][0]+2:continue
    bins[round(s/5)*5].append((float(s),r))
summary=[]
for station,rows in sorted(bins.items()):
    frames={r['frame_id'] for _,r in rows};times=[r['time_s'] for _,r in rows]
    if len(frames)<3:continue
    near=[]
    for m in maps:
        if not min(times)-2<=m['time_s']<=max(times)+2:continue
        es=float(np.interp(m['time_s'],t,elocal[:,0]))
        for j in m['junction_candidates']:
            near.append(dict(node=j['node_id'],time_s=m['time_s'],distance_m=j['distance_m'],station_residual_m=es+j['forward_m']-station))
    summary.append(dict(station=station,frames=len(frames),time_range=[min(times),max(times)],map_nearby=near))
(a.output/'crossbars.json').write_text(json.dumps(summary,indent=2))
print([(r['station'],r['frames'],[round(x,1) for x in r['time_range']],min([round(abs(j['station_residual_m']),1) for j in r['map_nearby']],default=None)) for r in summary])
