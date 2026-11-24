"""Observed rear red appearance, not brake/control or inferred blink phases.

Only repeated clear paired red corners in actual front frames can light a
proxy model. No observation means unknown; gaps and ghosts do not retain light.
"""
from collections import defaultdict
from pathlib import Path
import bisect
import cv2
from .rear_view_cue import paired_red_lamps


def confirmed_samples(rows,window_s=.6):
    rows=sorted(rows,key=lambda r:r['time_s']);out=[]
    for r in rows:
        near=[p for p in rows if abs(p['time_s']-r['time_s'])<=window_s]
        if len({p['frame_id'] for p in near})>=3:
            out.append(dict(r,rear_red_visible=True,brake=None,left=None,right=None,
                            supporting_frame_ids=sorted({p['frame_id'] for p in near}),
                            source='repeated_front_paired_red_appearance; control_state_unknown'))
    return out


def extract(package,frames,observations,membership):
    by=defaultdict(list);groups=defaultdict(list)
    for row in observations:
        if row['camera_id']=='front' and row['support']=='detector_observation' and row['local_track_id'] in membership:
            by[row['frame_id']].append(row)
    for frame in frames:
        rows=by.get(frame['frame_id'])
        if not rows:continue
        image=cv2.imread(str(Path(package)/frame['image']))
        for row in rows:
            cue=paired_red_lamps(image,row['bbox_xyxy'])
            if cue['rear_cue']:
                groups[membership[row['local_track_id']]].append(dict(time_s=row['camera_time_s'],frame_id=row['frame_id'],
                    local_track_id=row['local_track_id'],bbox_xyxy=row['bbox_xyxy'],components=cue['components']))
    return {key:confirmed_samples(rows) for key,rows in groups.items()}


class RearAppearanceTrace:
    def __init__(self,samples,max_age_s=.11):
        self.samples={k:sorted(v,key=lambda r:r['time_s']) for k,v in samples.items()}
        self.times={k:[r['time_s'] for r in v] for k,v in self.samples.items()};self.max_age=max_age_s

    def at(self,key,t,ghost=False):
        unknown=dict(rear_red_visible=None,brake=None,left=None,right=None,source='unknown')
        times=self.times.get(key,[])
        if ghost or not times:return unknown
        i=bisect.bisect_left(times,t);pool=[j for j in (i-1,i) if 0<=j<len(times)]
        nearest=min(pool,key=lambda j:abs(times[j]-t))
        if abs(times[nearest]-t)>self.max_age:return unknown
        row=self.samples[key][nearest]
        return dict(rear_red_visible=True,brake=None,left=None,right=None,source=row['source'],
                    source_time_s=row['time_s'],frame_id=row['frame_id'])


def rear_material(name):return name in ('FSDTaillight','FSDRedSignal')
