"""Resolve a local width transition hidden by a long constant road section.

Edges are fitted from paint only. Raw forward traffic merely vetoes an apparent
extra band that could be the opposite carriageway; it never fits edge positions.
"""
import copy,math
import numpy as np
from .road_frame import frame_for
from .curved_road import supported_boundaries


def refine(road,segments,actors,ego,width,max_lanes=8):
    frame=frame_for(road)
    if frame is None or not road.get('lane_sections'):return [],dict(status='no_section_reference')
    paint=[(float(frame.project(np.mean(r['points_W_xy'],axis=0))[0]),r) for r in segments]
    ego=np.asarray(ego);elocal=frame.project(ego);result=[];reviews=[]
    for section in road['lane_sections']:
        base=section['boundaries_y'];start,end=section['station_range_m'];candidates=[];local=[]
        for s in np.arange(math.ceil(start/20)*20,end,20):
            ep=ego[(elocal[:,0]>=s-30)&(elocal[:,0]<=s+30)]
            if len(ep)<3:continue
            bounds,review=supported_boundaries([r for x,r in paint if s-30<=x<=s+30],road['reference_line'],width,[],max_lanes,ego_positions=ep)
            if bounds is None:continue
            local.append((float(s),bounds,review))
            if (review['status']=='repeated_continuous_carriageway_edges' and len(bounds)==len(base)+1
                and .65*width<bounds[0]-base[0]<1.35*width and abs(bounds[-1]-base[-1])<1.
                and all(.75*width<b-a<1.25*width for b,a in zip(bounds,bounds[1:]))):
                candidates.append((float(s),bounds,review))
        runs=[]
        for row in candidates:
            if not runs or row[0]-runs[-1][-1][0]>21:runs.append([])
            runs[-1].append(row)
        runs=[r for r in runs if len(r)>=3 and r[-1][0]-r[0][0]>=40]
        if not runs:result.append(copy.deepcopy(section));continue
        run=max(runs,key=len);a,b=run[0][0],run[-1][0]
        after=[r for r in local if b+20<=r[0]<=b+140 and len(r[1])==len(base)
               and abs(r[1][0]-base[0])<.7 and abs(r[1][-1]-base[-1])<1.
               and any(p['kind']=='yellow_divider' for p in r[2].get('peaks',[]))]
        if not after:result.append(copy.deepcopy(section));continue
        # A directly observed yellow divider inside the proposed added lane
        # contradicts expansion. Do not count opposing pavement as our road.
        yellow=[r for x,r in paint if a-30<=x<=b+30 and r['kind']=='yellow']
        contradiction={r['frame_id'] for r in yellow
                       if base[0]-.4<float(frame.project(np.mean(r['points_W_xy'],axis=0))[1])<run[0][1][0]-.4}
        if len(contradiction)>=3:result.append(copy.deepcopy(section));continue
        traffic=[]
        for actor in actors:
            if actor.get('camera_id')!='front':continue
            rows=[r for r in actor['samples'] if r.get('support')=='detector_observation' and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=60]
            if len(rows)<10:continue
            q=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows])
            mask=(q[:,0]>=a-30)&(q[:,0]<=b+30)&(q[:,1]>base[0]+.35)&(q[:,1]<run[0][1][0]-.35)
            chosen=np.flatnonzero(mask)
            if len(chosen)<10:continue
            first,last=chosen[0],chosen[-1];dt=rows[last]['time_s']-rows[first]['time_s']
            if dt<1.5:continue
            speed=(q[last,0]-q[first,0])/dt
            if 2<speed<55:traffic.append(dict(local_id=actor['id'],frames=len(chosen),span_s=dt,role='forward_scope_veto_only_not_edge_fit'))
        if not traffic:result.append(copy.deepcopy(section));continue
        finish,final,proof=after[0];begin=max(start,a-20)
        if begin>start:
            prefix=copy.deepcopy(section);prefix.update(id=section['id']+'-before-local',station_range_m=[start,begin]);result.append(prefix)
        patched=copy.deepcopy(section)
        knots=[dict(station_m=begin,boundaries_y=[base[0],*base])]
        knots.extend(dict(station_m=s,boundaries_y=list(bounds)) for s,bounds,_ in run)
        knots.append(dict(station_m=finish,boundaries_y=[*final,final[-1]]))
        if end>finish:knots.append(dict(station_m=end,boundaries_y=[*base,base[-1]]))
        patched.update(id=section['id']+'-local-extents',station_range_m=[begin,end],boundary_profile=knots,
                       left_added_lane=True,minimum_profile_lane_width_m=.8,
                       support='local_repeated_outer_paint_then_yellow_divider_transition')
        result.append(patched);reviews.append(dict(section_id=section['id'],paint_window_centres_m=[r[0] for r in run],
                                                  return_yellow_station_m=finish,traffic_scope=traffic,
                                                  edge_fit_uses_vehicles=False,ego_or_actor_pose_modified=False))
    return result,dict(status='local_paint_extent_transition' if reviews else 'unchanged_no_independent_transition',changes=reviews)
