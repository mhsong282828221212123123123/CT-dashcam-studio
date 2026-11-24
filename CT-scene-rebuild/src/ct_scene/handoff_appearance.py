"""Repeated clear views supplement appearance after independent motion fits."""
import numpy as np
from .tracking import appearance_distance


def review(first,second):
    pools=[]
    for actor in (first,second):
        rows=list({r['frame_id']:r for r in actor.get('clear_appearance_views',[])}.values())
        if len(rows)<6 or max(r['time_s'] for r in rows)-min(r['time_s'] for r in rows)<1.:
            return None
        pools.append(rows)
    distances=[appearance_distance(a['histogram'],b['histogram']) for a in pools[0] for b in pools[1]]
    return dict(distance=float(np.median(distances)),basis='median_pairwise_clear_source_views',
                frame_ids=[[r['frame_id'] for r in rows] for rows in pools],
                minimum=float(min(distances)),pairs=len(distances))
