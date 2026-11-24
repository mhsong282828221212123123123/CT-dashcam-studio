"""Compare yellow-paint lane-mask veto with road-mask support. Diagnostic only."""
import argparse,json
from pathlib import Path
from collections import defaultdict,Counter
import cv2,numpy as np
from ct_scene.curved_road import paint_segments,supported_boundaries,vehicle_supported_paint_gaps
from ct_scene.road_frame import RoadFrame

ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--yellow-road-mask',action='store_true');ap.add_argument('--lane-road-intersection',action='store_true');a=ap.parse_args();a.output.mkdir(exist_ok=False)
load=lambda p:json.loads(p.read_text(encoding='utf-8'))
frames=load(a.case/'source/frames.json');obs=defaultdict(list)
for r in map(json.loads,(a.case/'tracks/observations.jsonl').read_text().splitlines()):obs[r['frame_id']].append(r)
poses=load(a.case/'base/trajectory.json');times=np.array([p['camera_time_s'] for p in poses]);ego=np.array([p['ego_position_W'] for p in poses]);yaw=np.unwrap([p['ego_yaw_rad'] for p in poses])
profile=load(Path('configs/multicam-optical-candidates.json'))['cameras']['front'];cfg=load(Path('configs/surrounding-prototype.json'))
segments=paint_segments(a.case/'source',frames,obs,times,ego,yaw,profile,cfg['cameras']['front'],a.case/'road-masks',yellow_road_mask=a.yellow_road_mask,lane_road_intersection=a.lane_road_intersection)
(a.output/'segments.json').write_text(json.dumps(segments),encoding='utf-8')
road=load(a.scene/'report.json')['road'];actors=load(a.scene/'pre-road-geometry.json') if (a.scene/'pre-road-geometry.json').exists() else []
line=road.get('reference_line');frame=RoadFrame(line['points_W_xy'],line.get('station_origin_m',0.)) if line else None
if frame is not None:
    bounds,review=supported_boundaries(segments,line,3.5,actors,4,ego_positions=ego)
    if bounds is None:bounds,fallback=vehicle_supported_paint_gaps(review,line,3.5,actors,ego,4)
    else:fallback=None
else:bounds=None;review=dict(status='no_active_curved_reference; candidate_paint_only');fallback=None
yellow=[r for r in segments if r['kind']=='yellow'];groups=defaultdict(list)
for r in yellow:
    if frame is None:continue
    s,d,h=frame.project(np.mean(r['points_W_xy'],axis=0));groups[round(d*2)/2].append(r)
summary=dict(status='DIAGNOSTIC_ONLY_NOT_ADOPTED',options=dict(yellow_road_mask=a.yellow_road_mask,lane_road_intersection=a.lane_road_intersection),segment_counts=dict(Counter(r['kind'] for r in segments)),bounds=bounds,review=review,fallback=fallback,yellow_lateral_bins=[dict(lateral_m=k,frames=len({r['frame_id'] for r in v})) for k,v in sorted(groups.items())])
(a.output/'review.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');print(json.dumps(summary))
by={f['frame_id']:f for f in frames}
for t in [5,10,20,40]:
    f=min([f for f in frames if f['camera_id']=='front'],key=lambda f:abs(f['camera_time_s']-t));im=cv2.imread(str(a.case/'source'/f['image']))
    for r in yellow:
        if r['frame_id']==f['frame_id']:
            x,y,X,Y=r['pixels'];cv2.line(im,(x,y),(X,Y),(255,0,255),3)
    cv2.imwrite(str(a.output/f'yellow-{t}.jpg'),im)
