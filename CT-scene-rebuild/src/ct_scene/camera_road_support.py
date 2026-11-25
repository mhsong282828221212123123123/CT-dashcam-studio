"""Review-only side-camera road support; never establishes metric XY or identity."""
import numpy as np


def below_body_score(mask,box):
    h,w=mask.shape[:2];x0,x1=map(int,[box[0]+.25*(box[2]-box[0]),box[0]+.75*(box[2]-box[0])]);y0,y1=map(int,[box[3]+4,box[3]+24])
    if x0<0 or x1>w or y0<0 or y1>h or x1-x0<4 or y1-y0<8:return None
    fraction=float(np.mean(mask[y0:y1,x0:x1]>0))
    # Far silhouettes occupy too few input pixels for a negative road-mask
    # result to establish off-road membership. Positive road support may veto.
    scale=min(640./w,640./h)
    if fraction<=.1 and min(box[2]-box[0],box[3]-box[1])*scale<32:return None
    return fraction


def summarize(rows):
    valid=[r for r in rows if r['road_fraction'] is not None];inside=[r for r in valid if r['road_fraction']>=.6];outside=[r for r in valid if r['road_fraction']<=.1]
    span=max((r['time_s'] for r in valid),default=0)-min((r['time_s'] for r in valid),default=0)
    candidate=len(valid)>=8 and span>=1.4 and not inside and len(outside)>=.9*len(valid)
    return dict(frames=len(valid),span_s=span,on_road_support_frames=len(inside),off_road_support_frames=len(outside),ambiguous_frames=len(valid)-len(inside)-len(outside),recommendation='review_off_carriageway' if candidate else 'insufficient_or_conflicting_evidence',automatically_exclude=False)
