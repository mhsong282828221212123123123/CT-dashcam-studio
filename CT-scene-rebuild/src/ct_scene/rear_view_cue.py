"""Conservative image cue for a front-camera vehicle's visible rear.

Paired red corner lamps are evidence of rear appearance, not measured brake
or indicator states. Red bodies, a single lamp and white headlamps abstain.
"""
import cv2
import numpy as np
from pathlib import Path
from functools import lru_cache
from .road_frame import coordinates


def paired_red_lamps(image,box):
    h,w=image.shape[:2];x,y,X,Y=np.rint(box).astype(int)
    if min(x,y,w-X,h-Y)<3 or X-x<80 or Y-y<75:return dict(rear_cue=False,reason='partial_or_small_body')
    crop=image[y:Y,x:X];ch,cw=crop.shape[:2];hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    red=(((hsv[:,:,0]<12)|(hsv[:,:,0]>170))&(hsv[:,:,1]>90)&(hsv[:,:,2]>90)).astype('uint8')
    fraction=float(red.mean())
    if fraction>.12:return dict(rear_cue=False,reason='red_body_or_broad_surface',red_fraction=fraction)
    groups=[]
    for a,b in [(0,int(cw*.35)),(int(cw*.65),cw)]:
        mask=red[:,a:b].copy();mask[:int(ch*.15)]=0;mask[int(ch*.85):]=0
        _,labels,stats,centres=cv2.connectedComponentsWithStats(mask)
        rows=[]
        for (rx,ry,rw,rh,area),(cx,cy) in zip(stats[1:],centres[1:]):
            if area>=max(2,ch*cw*.0005) and rw<cw*.3 and rh<ch*.65:
                rows.append(dict(centre_xy=[float(x+a+cx),float(y+cy)],area=int(area),box_xyxy=[int(x+a+rx),int(y+ry),int(x+a+rx+rw),int(y+ry+rh)]))
        groups.append(rows)
    pairs=[(a,b) for a in groups[0] for b in groups[1] if abs(a['centre_xy'][1]-b['centre_xy'][1])<ch*.2]
    if not pairs:return dict(rear_cue=False,reason='no_aligned_red_corner_pair',red_fraction=fraction)
    a,b=max(pairs,key=lambda p:min(p[0]['area'],p[1]['area']))
    return dict(rear_cue=True,reason='paired_red_corner_lamp_cue_not_brake_state',red_fraction=fraction,components=[a,b])


def actor_cues(actors,package,frames,road):
    """Inspect only clear front tracks near the left side of a junction approach."""
    by={f['frame_id']:f for f in frames};result={};width=road.get('lane_width_m_prior',3.5)
    @lru_cache(maxsize=8)
    def image(fid):return cv2.imread(str(Path(package)/by[fid]['image']))
    for actor in actors:
        if actor['camera_id']!='front':continue
        rows=actor['samples'];local=coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in rows]);candidate=False
        for section in road.get('lane_sections') or [dict(station_range_m=road['reference_line']['supported_extent_m'],boundaries_y=road['boundaries_y'])]:
            lo,hi=section['station_range_m'];left=section['boundaries_y'][0]
            if not any(abs(j['road_station_range_m'][0]-hi)<1 for j in road.get('junctions',[])):continue
            if np.count_nonzero((local[:,0]>max(lo,hi-80))&(local[:,0]<hi)&(local[:,1]>left+.5)&(local[:,1]<left+width-.25))>=8:candidate=True
        if not candidate:continue
        samples=[]
        for row in rows:
            if row.get('support')!='detector_observation':continue
            cue=paired_red_lamps(image(row['frame_id']),row['bbox_xyxy'])
            if cue['rear_cue']:samples.append(dict(frame_id=row['frame_id'],time_s=row['time_s'],**cue))
        result[actor['id']]=dict(source='front_paired_red_corner_cue_not_brake_state',samples=samples)
    return result
