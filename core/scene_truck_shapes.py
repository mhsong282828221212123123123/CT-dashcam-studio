"""Track-level representative truck shape selection, not measured dimensions."""
from collections import defaultdict
import numpy as np


def infer_truck_shapes(rows, camera_frames):
    from core.scene_fusion import fit_box
    observations=defaultdict(list)
    for i,row in enumerate(rows):
        for name,data in row.get('cameras',{}).items():
            camera=camera_frames[i].get(name)
            if name=='back' or camera is None or abs(data.get('source_time_s',row['time_s'])-row['time_s'])>.08:
                continue
            for obj in data.get('objects',[]):
                if obj.get('kind')!='truck' or obj.get('track_status')!='confirmed':continue
                if obj.get('geometry_kind'):continue # Explicit shape evidence outranks a box-size prior.
                x,y,w,h=obj['bbox']
                if x<4 or x+w>camera.width-4 or y<4 or y+h>camera.height-4:continue
                contact=(x+w/2,y+h)
                if any(other is not obj and other['bbox'][0]<contact[0]<sum(other['bbox'][::2])
                       and other['bbox'][1]<contact[1]<other['bbox'][1]+other['bbox'][3]-10
                       for other in data.get('objects',[])):continue
                observations[obj['track_id']].append((row['time_s'],camera,obj['bbox']))
    result={}
    for key,items in observations.items():
        if len(items)<8 or items[-1][0]-items[0][0]<.7:continue
        selected=[items[i] for i in np.unique(np.linspace(0,len(items)-1,min(16,len(items))).astype(int))]
        scores={}
        for shape in ('truck','truck_box','truck_medium','truck_heavy'):
            errors=[]
            for _,camera,box in selected:
                fit=fit_box(camera,box,shape)
                errors.append(.5 if fit is None else fit[2])
            scores[shape]=float(np.mean(errors))
        winner=min(scores,key=scores.get)
        if winner=='truck' or scores['truck']-scores[winner]<.04:continue
        if scores[winner]>.2:continue
        result[key]={'shape':winner,'scores':scores,'sample_count':len(selected),
                     'source':'offline_track_box_evidence','dimensions_verified':False}
    return result
