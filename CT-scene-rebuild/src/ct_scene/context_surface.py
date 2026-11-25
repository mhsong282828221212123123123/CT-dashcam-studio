"""Approximate neighboring OSM pavement, never a vehicle/lane constraint."""
from collections import Counter
import json,math
from pathlib import Path
import numpy as np
from .artifacts import canonical,digest
from .osm_context import load_roads,xy,level
from .road_frame import coordinates,world
from .road_geometry import supported,boundaries_at


def registration(gps_xy,world_xy,heading_deg):
    g=np.asarray(gps_xy,float);w=np.asarray(world_xy,float)
    angle=math.radians(heading_deg)
    R=np.array([[math.sin(angle),-math.cos(angle)],[math.cos(angle),math.sin(angle)]])
    if len(g)>=6 and np.linalg.norm(np.ptp(g,axis=0))>=20:
        U,_,V=np.linalg.svd((g-g.mean(0)).T@(w-w.mean(0)))
        R=U@np.diag([1.,np.linalg.det(U@V)])@V
    offset=np.median(w-g@R,axis=0)
    error=np.linalg.norm(g@R+offset-w,axis=1)
    return R,offset,float(np.quantile(error,.9))


def fit_display_shift(anchors,allow_short=False):
    """Fit only the observed road-normal components, retaining ambiguity."""
    if len(anchors)<6:return None
    span=float(np.linalg.norm(np.ptp([a['position'] for a in anchors],axis=0)))
    normals=np.array([a['normal'] for a in anchors]);target=np.array([a['target'] for a in anchors])
    short=span<15
    if short:
        # A short, nearly straight approach still observes lateral offset.
        # Require stable repeated evidence, retain the longitudinal null space,
        # and do not extend this exception to stationary/jittering/turning data.
        duration=max(a['time_s'] for a in anchors)-min(a['time_s'] for a in anchors)
        stable=bool(np.min(normals@normals[0])>=math.cos(math.radians(3)) and np.ptp(target)<=1.)
        if not (allow_short and span>=3 and duration>=3 and stable):return None
    shift=np.linalg.lstsq(normals,target,rcond=.1)[0]
    for _ in range(6):
        residual=normals@shift-target;weight=np.minimum(1,1./np.maximum(abs(residual),1e-6))
        shift=np.linalg.lstsq(normals*weight[:,None],target*weight,rcond=.1)[0]
    residual=np.abs(normals@shift-target);consistent=residual<=3.5
    accepted=np.linalg.norm(shift)<=25 and np.median(residual)<=1.75 and np.mean(consistent)>=.7 and sum(consistent)>=6
    if short:accepted=accepted and np.quantile(residual,.9)<=.5
    return dict(accepted=bool(accepted),translation=shift.tolist(),span_m=span,
                residual_median_m=float(np.median(residual)),residual_p90_m=float(np.quantile(residual,.9)),
                consistent=consistent.tolist(),consistent_fraction=float(np.mean(consistent)),
                support_kind='short_stable_lateral_only' if short else 'extended_road_support',
                longitudinal_correction_fitted=False if short else 'road_normals_only')


def warp_context_points(points,regions):
    """A shared spatial field keeps connected OSM nodes connected at turns."""
    points=np.asarray(points,float)
    distances=np.array([np.min(np.linalg.norm(points[:,None,:]-np.asarray(r['map_anchor_positions_W_xy'])[None,:,:],axis=2),axis=1) for r in regions]).T
    # Subtract the closest squared distance to avoid underflow far from support.
    weights=np.exp(-(distances**2-np.min(distances**2,axis=1)[:,None])/(2*15.**2))
    shifts=np.array([r['display_translation_W_xy'] for r in regions])
    return points+(weights@shifts)/weights.sum(axis=1)[:,None],distances.min(axis=1)


def align_main_road(ways,origin,R,offset,times,ego,records,road):
    """Move the map display to the camera road, never move the ego to the map.

    Use road normals, so a straight road cannot invent longitudinal alignment.
    The camera corridor centre is independent of the ego's particular lane.
    """
    by_id={w['id']:w for w in ways};anchors=[];votes=Counter()
    for record in records:
        candidates=record.get('way_candidates',[])
        if not candidates or candidates[0]['way_id'] not in by_id:continue
        wid=candidates[0]['way_id'];way=by_id[wid];t=record['time_s']
        votes[wid]+=1
        ep=np.array([np.interp(t,times,ego[:,k]) for k in range(2)])
        s,d,heading=coordinates(road,ep)
        if not supported(road,float(s)):continue
        boundaries=boundaries_at(road,float(s))
        target=world(road,float(s),(boundaries[0]+boundaries[-1])/2)
        tangent=np.array([math.cos(heading),math.sin(heading)])
        normal=np.array([-tangent[1],tangent[0]])
        p=xy(way['geometry'],origin)@R+offset
        a=p[:-1];delta=np.diff(p,axis=0);length=np.linalg.norm(delta,axis=1)
        valid=length>.1
        a,delta,length=a[valid],delta[valid],length[valid]
        if not len(a):continue
        parallel=np.abs((delta/length[:,None])@tangent)>.8
        u=np.clip(np.einsum('ij,ij->i',target-a,delta)/length**2,0,1)
        points=a+u[:,None]*delta;distance=np.linalg.norm(points-target,axis=1)
        eligible=np.flatnonzero(parallel & (distance<40))
        if not len(eligible):continue
        i=min(eligible,key=lambda i:distance[i])
        anchors.append(dict(normal=normal,target=float(normal@(target-points[i])),
                            time_s=t,way_id=wid,position=target,map_position=points[i]))
    selected=[wid for wid,count in votes.items() if count>=2]
    anchors=[a for a in anchors if a['way_id'] in selected]
    review=dict(accepted=False,main_way_ids=selected,anchor_count=len(anchors),
                target='camera_road_centre_not_ego_lane',ego_actor_or_lane_modified=False)
    if len(anchors)<6:return np.zeros(2),dict(review,status='insufficient_camera_road_anchors')
    span=np.linalg.norm(np.ptp([a['position'] for a in anchors],axis=0))
    if span<15:return np.zeros(2),dict(review,status='insufficient_road_span')
    fit=fit_display_shift(anchors);shift=np.asarray(fit['translation']);consistent=np.asarray(fit['consistent'])
    # GPS/road support may fail locally during a turn. Keep a clear majority
    # alignment and omit map-only pavement near rejected anchors instead of
    # turning the entire driven road into a parallel strip.
    fraction=fit['consistent_fraction'];accepted=fit['accepted']
    review.update(accepted=bool(accepted),status='map_display_aligned_to_camera_main_road' if accepted else 'inconsistent_main_road_alignment',
                  display_translation_W_xy=shift.tolist(),normal_residual_median_m=fit['residual_median_m'],
                  normal_residual_p90_m=fit['residual_p90_m'],road_span_m=float(span))
    review.update(consistent_anchor_fraction=fraction,
                  supported_anchor_positions_W_xy=[a['position'].tolist() for a,ok in zip(anchors,consistent) if ok],
                  rejected_anchor_times_s=[a['time_s'] for a,ok in zip(anchors,consistent) if not ok])
    if not accepted:
        regions=[]
        for wid in selected:
            local=[a for a in anchors if a['way_id']==wid];local_fit=fit_display_shift(local,allow_short=True)
            if not local_fit or not local_fit['accepted']:continue
            good=[a for a,ok in zip(local,local_fit['consistent']) if ok]
            regions.append(dict(way_id=wid,display_translation_W_xy=local_fit['translation'],
                                camera_anchor_positions_W_xy=[a['position'].tolist() for a in good],
                                map_anchor_positions_W_xy=[a['map_position'].tolist() for a in good],
                                time_interval_s=[min(a['time_s'] for a in good),max(a['time_s'] for a in good)],
                                fit=local_fit))
        if regions:
            review.update(accepted=True,status='locally_supported_map_display_after_turn',
                          local_regions=regions,global_alignment_accepted=False,
                          interpolation='shared_spatial_display_field; not metric registration')
            return np.zeros(2),review
    return shift if accepted else np.zeros(2),review


def build(ways,gps_rows,times,ego,records,road=None):
    valid=[r for r in gps_rows if all(r['fields'][k]['valid'] for k in ('latitude_deg','longitude_deg','heading_deg'))]
    if len(valid)<3:return dict(polygons_W_xy=[],status='insufficient_GPS_for_context_only')
    origin=[valid[0]['fields'][k]['value'] for k in ('latitude_deg','longitude_deg')]
    geo=np.array([[r['fields'][k]['value'] for k in ('latitude_deg','longitude_deg')] for r in valid])
    positions=np.array([[np.interp(r['camera_time_s'],times,ego[:,k]) for k in range(2)] for r in valid])
    R,offset,error=registration(xy(geo,origin),positions,valid[0]['fields']['heading_deg']['value'])
    by_id={w['id']:w for w in ways};grades=Counter()
    for record in records:
        candidates=record.get('way_candidates',[])
        if candidates and candidates[0]['way_id'] in by_id:grades[level(by_id[candidates[0]['way_id']]['tags'])]+=1
    if not grades:return dict(polygons_W_xy=[],status='no_supported_map_grade')
    grade,count=grades.most_common(1)[0]
    if count/sum(grades.values())<.7:return dict(polygons_W_xy=[],status='ambiguous_map_grade')
    alignment=None;main_ids=set()
    if road is not None:
        shift,alignment=align_main_road([w for w in ways if level(w['tags'])==grade],origin,R,offset,times,ego,records,road)
        main_ids=set(alignment['main_way_ids'])
        # An unmatched map must not masquerade as a second road beside the car.
        if not alignment['accepted']:
            return dict(polygons_W_xy=[],status='map_context_alignment_unresolved',main_road_alignment=alignment,
                        main_road_display='existing_camera_corridor',ego_actor_or_lane_modified=False)
        offset=offset+shift
    polygons=[];used=[]
    for way in ways:
        if level(way['tags'])!=grade:continue
        # This is the road we already render from camera evidence, not a
        # neighbouring asphalt strip. Keep its topology but do not draw it twice.
        if way['id'] in main_ids:continue
        p=xy(way['geometry'],origin)@R+offset
        regions=alignment.get('local_regions',[]) if alignment is not None else []
        if regions:
            # Short segments allow local support clipping and avoid giant
            # polygons when a road spans both sides of a turn.
            chunks=[]
            for start,end in zip(p,p[1:]):
                count=max(1,math.ceil(np.linalg.norm(end-start)/6.))
                chunks.extend(start+(end-start)*j/count for j in range(count))
            p=np.array([*chunks,p[-1]])
            p,region_distances=warp_context_points(p,regions)
        kind=way['tags']['highway']
        width=12. if kind in ('primary','secondary','trunk','motorway') else 5. if kind.endswith('_link') else 4. if kind in ('service','living_street') else 7.
        accepted=False
        for segment,(a,b) in enumerate(zip(p,p[1:])):
            d=b-a;length=np.linalg.norm(d)
            if length<.1:continue
            if regions and max(region_distances[segment:segment+2])>35:continue
            if not regions and alignment is not None and alignment.get('rejected_anchor_times_s'):
                anchors=np.asarray(alignment['supported_anchor_positions_W_xy'])
                u=np.clip(((anchors-a)@d)/(length*length),0,1)
                if np.min(np.linalg.norm(anchors-(a+u[:,None]*d),axis=1))>35:continue
            u=np.clip(((positions-a)@d)/(length*length),0,1)
            if np.min(np.linalg.norm(positions-(a+u[:,None]*d),axis=1))>70:continue
            normal=np.array([-d[1],d[0]])/length*width/2
            polygons.append([list(a-normal),list(b-normal),list(b+normal),list(a+normal)])
            for centre in (a,b):
                angles=np.arange(12)*2*np.pi/12
                polygons.append((centre+np.c_[np.cos(angles),np.sin(angles)]*width/2).tolist())
            accepted=True
        if accepted:used.append(way['id'])
    result=dict(polygons_W_xy=polygons,status='approximate_neighboring_road_context',way_ids=used,
                map_display_registration_p90_m=error,map_grade=list(grade),scale_fitted=False,
                width_source='coarse_display_prior_by_road_class; no_OSM_lane_or_width_tags',
                registration='rigid_map_to_fixed_ego_trace; no_ego_actor_or_lane_modification',
                use='display_context_only; GPS_accuracy_does_not_assign_a_lane')
    if alignment is not None:
        total_lanes=[]
        per_dir=[]
        for wid in main_ids:
            if wid not in by_id:continue
            t=by_id[wid]['tags']
            if 'lanes' in t:
                try:
                    n=int(str(t['lanes']).split(';')[0])
                    total_lanes.append(n)
                    is_1w=t.get('oneway') in ('yes','1','true','-1') or t.get('highway') in ('motorway','motorway_link','trunk_link')
                    per_dir.append(n if is_1w else max(1,n//2))
                except ValueError:pass
        min_total=min(total_lanes) if total_lanes else None
        two_way=(min_total is None or min_total>=2) and any(
            by_id[wid]['tags'].get('oneway') not in ('yes','1','true','-1')
            and by_id[wid]['tags'].get('highway') not in ('motorway','motorway_link','trunk_link','service')
            for wid in main_ids if wid in by_id)
        result.update(main_road_alignment=alignment,main_road_display='fused_with_camera_corridor',
                      suppressed_duplicate_main_way_ids=sorted(main_ids),main_way_two_way=two_way,
                      main_way_total_lanes=min_total,
                      main_way_lanes_per_dir=min(per_dir) if per_dir else None)
    if alignment is not None and alignment.get('local_regions'):result['registration']='local_camera_road_display_field; no_ego_actor_lane_or_metric_registration'
    return result


def extract(map_manifest,telemetry,times,ego,records,cache_roots,road=None,osm_path=None):
    candidates_paths=[]
    if osm_path and Path(osm_path).is_file():candidates_paths.append(Path(osm_path))
    for root in cache_roots:
        if Path(root).is_dir():candidates_paths.extend(Path(root).glob('*.osm'))
    for path in candidates_paths:
        if digest(canonical(dict(osm_sha256=digest(path.read_bytes()))))!=map_manifest['config_sha256']:continue
        rows=[json.loads(s) for s in (Path(telemetry)/'telemetry.jsonl').read_text().splitlines()]
        result=build(load_roads(path),rows,times,ego,records,road)
        result.update(osm_sha256=digest(path.read_bytes()),network_requests=0)
        return result
    return dict(polygons_W_xy=[],status='matching_cached_OSM_unavailable',network_requests=0)
