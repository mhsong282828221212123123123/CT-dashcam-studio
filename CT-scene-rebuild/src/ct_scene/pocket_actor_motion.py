"""Fixed main-lane IDs with a time-varying optional left auxiliary lane."""
import numpy as np
from .actor_lane_motion import estimate
from .road_frame import coordinates
from .road_geometry import boundaries_at,section_at,pocket_width,supported


def priors(rows,road,fallback,dwell_s=.6,margin_m=.2):
    local=coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in rows])
    keys=[((section_at(road,float(s)) or {}).get('id'),supported(road,float(s)),pocket_width(road['left_pockets'],float(s))>=1.8) for s in local[:,0]]
    lanes=np.zeros(len(rows),int);targets=local[:,1].copy();reviews=[];i=0
    while i<len(rows):
        j=i+1
        while j<len(rows) and keys[j]==keys[i]:j+=1
        section,known,extra=keys[i];bounds=list(boundaries_at(road,float(local[i,0])))
        if extra:bounds.insert(0,bounds[0]+float(np.median([pocket_width(road['left_pockets'],float(s)) for s in local[i:j,0]])))
        median=float(np.median(local[i:j,1]));candidate=next((k for k in range(len(bounds)-1) if bounds[k+1]<=median<=bounds[k]),None)
        fallback_index=fallback+int(extra);fallback_index=min(max(0,fallback_index),len(bounds)-2)
        part=[dict(r,position_W=[p[0],p[1],0.],unrefined_position_W=[p[0],p[1],0.]) for r,p in zip(rows[i:j],local[i:j])]
        ll,tt,review=estimate(part,bounds,0.,candidate if candidate is not None else fallback_index,dwell_s,margin_m)
        lanes[i:j]=ll-int(extra);targets[i:j]=tt
        if extra:
            for k,index in enumerate(ll):
                if index==0:targets[i+k]=bounds[1]+pocket_width(road['left_pockets'],float(local[i+k,0]))/2
        if not known:
            targets[i:j]=local[i:j,1];review['events']=[];review['support']='unresolved_road_no_actor_lane_prior'
        for event in review['events']:
            for field in ['from_lane','to_lane']:
                if field in event:event[field]-=int(extra)
        reviews.append(dict(review,road_section_id=section,pocket_available=extra,road_supported=known));i=j
    return lanes,targets,dict(events=[dict(e,road_section_id=r['road_section_id']) for r in reviews for e in r['events']],
                             rejected_candidates=[e for r in reviews for e in r['rejected_candidates']],
                             clear_observations=sum(r['clear_observations'] for r in reviews),sections=reviews,
                             support='observed_main_or_auxiliary_membership; fixed_main_ids; no_track_samples_removed',
                             observer='fixed_world_road_frame; ego_lane_not_used')
