"""Exclude clearly unrelated vehicles from the inferred host-road display.

This is a conservative display scope, not a semantic median/barrier detector.
Preserve established road members through noisy poses and visible junctions.
"""
from collections import defaultdict
import cv2
import numpy as np
from core.scene_vehicle_road import VEHICLE_KINDS


VEHICLES=VEHICLE_KINDS


def exclude_unrelated_vehicles(scenes,audits):
    if len(scenes)!=len(audits):raise ValueError('Road scope timeline mismatch')
    references=set();inside=defaultdict(list)
    def coordinates(scene,audit,actor):
        road=audit.get('travel_corridors',{});view=getattr(scene,'road_scope_view',getattr(scene,'road_view',None))
        indices=road.get('occupied_lane_indices')
        if not view or not indices:return None
        width=road['lane_width_assumed_m'];segment=road['segment_start_s']
        point=np.asarray(view['rotation'])@np.array([actor.x,actor.z])+view['offset']
        return segment,point,(min(indices)-.5)*width,(max(indices)+.5)*width,width
    for scene,audit in zip(scenes,audits):
        road=audit.get('travel_corridors',{})
        if road.get('phase_source')=='camera_lane_phase':references.add(road['segment_start_s'])
        for actor in scene.actors:
            data=coordinates(scene,audit,actor)
            if data is None or actor.kind not in VEHICLES or actor.tracking_state!='observed':continue
            segment,point,left,right,_=data
            if abs(point[1])<=45 and left<=point[0]<=right:
                inside[segment,actor.key].append(scene.time)
    members={key for key,times in inside.items() if len(times)>=3 and times[-1]-times[0]>=.19}
    for scene,audit in zip(scenes,audits):
        kept=[];excluded=[]
        for actor in scene.actors:
            data=coordinates(scene,audit,actor)
            if data is None or actor.kind not in VEHICLES:
                kept.append(actor);continue
            segment,point,left,right,width=data
            # A guessed single road with no visual reference cannot classify
            # someone else's road. Never erase a real passing/lane-changing
            # vehicle just because its later pose drifts on a held straight grid.
            # The corridor count itself is inferred and may omit real adjacent
            # lanes. A tight footprint gate erased a genuine overtaking sedan
            # in the 8/18 source review. Without a measured separator, only
            # exclude traffic separated by a large lateral gap; do not use
            # the rendered outer stripe as a verified physical barrier.
            margin=max(2.*width,actor.width/2+.75)
            outside=point[0]<left-margin or point[0]>right+margin
            context=getattr(scene,'road_context',None) or {}
            junction=(audit.get('camera_junction',{}).get('active',False)
                or audit.get('road_junction',{}).get('state')=='inside'
                or context.get('nearby_junction_candidate',False))
            in_branch=any(cv2.pointPolygonTest(np.asarray(p,np.float32),
                          (float(actor.x),float(actor.z)),False)>=0
                          for p in getattr(scene,'road_surfaces',[]) if len(p)>=3)
            if segment in references and outside and (segment,actor.key) not in members and not junction and not in_branch:
                excluded.append(dict(id=actor.key,sources=list(actor.sources),
                    position=[actor.x,actor.z],lateral_margin_assumed_m=margin,
                    reason='clearly separated from host road; no established road membership'))
            else:kept.append(actor)
        scene.actors=kept
        audit['road_scope_excluded']=excluded
        audit['visible_ids']=[actor.key for actor in kept]
        audit['world_tracks']=[record for record in audit.get('world_tracks',[]) if record['id'] in audit['visible_ids']]
