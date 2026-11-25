"""Display only the ego carriageway; never pull external traffic into a lane.

Raw measurements remain available. A junction polygon protects actual crossing
traffic. Missing road support abstains; context/opposing display roads do not
extend ego-carriageway membership.
"""
import numpy as np
from .road_frame import coordinates
from .road_geometry import supported,boundaries_at,pocket_width
from .junctions import inside


def point_scope(road,position,margin=1.0):
    p=np.asarray(position,float)
    if not np.isfinite(p).all() or not road or not road.get('boundaries_y'):return 'unknown'
    in_junction=inside(road,p,0.)
    s,d,_=coordinates(road,p[:2])
    if in_junction:
        # Branch-display polygons include long side arms. Only the central
        # crossing region, not every car on those arms, belongs to our scope.
        b=boundaries_at(road,float(s));padding=float(road.get('lane_width_m_prior',3.5))
        high=float(b[0])+pocket_width(road.get('left_pockets',[]),float(s))
        return 'junction' if float(b[-1])-padding<=d<=high+padding else 'outside'
    if road.get('lane_geometry_status')=='unknown_route_display_only' or not supported(road,float(s)):return 'unknown'
    bounds=boundaries_at(road,float(s))
    if len(bounds)<2 or bounds[0]-bounds[-1]<1.5:return 'unknown'
    high=float(bounds[0])+pocket_width(road.get('left_pockets',[]),float(s))
    low=float(bounds[-1])
    return 'outside' if d>high+margin or d<low-margin else 'inside'


def mark_raw_scope(actor,road):
    rows=actor.get('samples',[])
    direct=[]
    for row in rows:
        row['driving_road_scope']='unknown'
        if row.get('support')!='detector_observation' or row.get('camera_id') not in ('front','left_repeater','right_repeater'):continue
        b=row.get('bbox_xyxy');w=row.get('image_width',1280);h=row.get('image_height',960)
        if b is None or min(b[0],b[1],w-b[2],h-b[3])<6 or b[2]-b[0]<50 or b[3]-b[1]<35:continue
        sigma=row.get('position_sigma_m')
        margin=1.0+min(2.,max(0.,float(sigma))*.5) if sigma is not None and np.isfinite(sigma) else 1.25
        state=point_scope(road,row.get('unrefined_position_W',row['position_W']),margin)
        direct.append((float(row['time_s']),state))
    # A weak lateral calibration must not hide clearly forward-moving front
    # traffic merely because its estimated world trace crosses a fitted edge.
    front=[r for r in rows if r.get('camera_id')=='front' and r.get('support')=='detector_observation']
    parallel=False
    if len(front)>=5:
        t=np.array([r['time_s'] for r in front],float)
        if t[-1]-t[0]>=1.:
            q=coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in front])
            v=np.linalg.lstsq(np.c_[t-t.mean(),np.ones(len(t))],q[:,:2],rcond=None)[0][0]
            parallel=bool(6.<v[0]<55. and abs(v[1])<min(2.5,.2*v[0]))
    actor['driving_road_scope_parallel_front_motion']=parallel
    for row in rows:
        t=float(row['time_s']);near=[(u,state) for u,state in direct if abs(u-t)<=.45]
        outside=[u for u,state in near if state=='outside']
        if len(outside)>=3 and max(outside)-min(outside)>=.3 and not any(state in ('inside','junction') for u,state in near):
            row['driving_road_scope']='unresolved_parallel_front_geometry' if parallel else 'outside'
    return dict(samples=len(rows),outside_raw_samples=sum(r['driving_road_scope']=='outside' for r in rows),raw_positions_modified=False)


def exclusion_reason(actor,time,position,road):
    final=point_scope(road,position)
    if final=='junction':return None
    rows=actor.get('samples',[])
    if rows:
        nearest=min(rows,key=lambda r:abs(r['time_s']-time))
        raw=nearest.get('unrefined_position_W',nearest['position_W'])
        # A camera-supported crossing must not be hidden by a displaced estimate.
        if point_scope(road,raw)=='junction':return None
        if abs(nearest['time_s']-time)<=.45 and nearest.get('driving_road_scope')=='outside' and point_scope(road,raw)=='outside':
            return 'persistent_raw_observation_outside_ego_carriageway'
    # A final estimate alone cannot prove an observed car is outside the road.
    # In particular, never conceal a normal road vehicle's calibration error.
    return None


def filter_plan(plan,actors,road):
    by_id={a.get('global_id',a.get('id')):a for a in actors};removed=[]
    for frame in plan:
        kept=[]
        for row in frame['actors']:
            reason=exclusion_reason(by_id.get(row['id'],{}),frame['time_s'],row['position_W'],road)
            if reason:removed.append(dict(time_s=frame['time_s'],id=row['id'],reason=reason,source_visible=row.get('source_visible',False)))
            else:kept.append(row)
        frame['actors']=kept
    return removed
