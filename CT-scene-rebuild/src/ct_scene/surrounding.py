"""Multi-vehicle prototype: estimated camera geometry, frozen ego/lead,
offline association, and an evidence-labelled road corridor.

This does not claim a measured rig or full junction reconstruction.
"""
from pathlib import Path
from collections import defaultdict
from fractions import Fraction
import io
import json
import math
import av
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .artifacts import verify,publish,canonical,digest
from .affine_probe import camera_matrix
from .prototype_front import Renderer,model_vertices,rotation
from .offline_paths import smooth_positions,WorldAnchor,complete_gap,augment_frozen_geometry
from .tracking import appearance,appearance_distance
from .ingest import file_hash
from .identity_graph import select_handoffs,identity_evidence_sufficient
from .lane_state import timeline as lane_timeline
from .scene_audit import observation_coverage,visible_ids_at
from .rear_reference import associate as associate_rear,reference_presence
from .actor_lane_motion import estimate as estimate_actor_lanes,observation_usable as lane_observation_usable
from .pocket_lane import paint_observations,stations as paint_stations,fit_pockets,fit_curved_pockets,vehicle_supported_curved_pockets
from .rear_view_cue import actor_cues
from .image_median_gate import extract_lines as image_median_lines,review as image_median_review
from .surrounding_lamps import extract as observed_rear_lamps,RearAppearanceTrace,rear_material
from .unresolved_road import route_display as unresolved_route_display
from .actor_rear_axis import align_actor as align_actor_rear_axis
from .image_heading_refine import refine_actor as refine_image_heading
from .clear_handoff import fit as fit_clear_handoff,repair as repair_clear_handoff
from .local_solid_corridor import fit as fit_local_solid_corridor,unresolved_side_height_position
from .source_strip import status as source_strip_status
from .right_edge_profile import refine as refine_right_edge_profile
from .blindspot_continuity import build as build_blindspot_coasts,at as blindspot_coasts_at
from .junction_display import display_bounds,corridor_gaps,mouth_edges,branch_edges,junction_markings,opposing_width
from .camera_junction_roads import supplement as camera_crossing_roads
from .static_actor_refine import refine_actor as refine_static_actor
from .left_median_membership import review as left_median_membership_review
from .lane_prior_geometry import refine as refine_lane_geometry,confirmed_handoff_targets
from .handoff_appearance import review as review_handoff_appearance
from .observed_surface import extract as observed_road_surface
from .context_surface import extract as context_road_surface
from .side_contact_recovery import recover as recover_side_contacts
from .collision_constraints import constrain as constrain_collisions, omit_conflicting_predictions
from .side_paint_geometry import extract as extract_side_paint,apply as apply_side_paint,extend_calibrated_tail
from .inside_median_pockets import extend as inside_median_approach_bands
from .pocket_actor_motion import priors as pocket_actor_priors
from .junction_branches import hints as junction_branch_hints,opening_display_polygons
from .road_geometry import pocket_width,lane_at,lane_bounds,supported as road_supported,boundaries_at,section_at,profile_boundaries
from .road_sections import fit as fit_road_sections
from .local_road_extents import refine as refine_local_road_extents
from .road_continuation import support_gaps
from .ego_maneuver import windows as ego_maneuver_windows,compensate_actors,camera_view
from .road_frame import coordinates as road_coordinates,world as road_world
from .curved_road import paint_segments,side_yellow_segments,fit_reference,supported_boundaries,phase_protection_windows,continuous_paint_excludes_lane_jump,vehicle_supported_paint_gaps
from .junctions import infer as infer_junctions,inside as inside_junction,straight_openings,crossing_motion
from .actor_heading import estimate as motion_headings,resolve_axis_flips
from .ego_lamps import LampTrace,role as lamp_role


def project_contact(box,profile,mount,max_depth=70.,minimum_ray_y=.015,contact_uv=None):
    K,D=camera_matrix(profile)
    if contact_uv is None:
        pixel=np.array([[[(box[0]+box[2])/2,box[3]]]],float)
    else:
        pixel=np.array([[[box[0]+float(contact_uv[0])*(box[2]-box[0]),box[1]+float(contact_uv[1])*(box[3]-box[1])]]],float)
    ray=cv2.undistortPoints(pixel,K,D).reshape(2)
    if ray[1]<=minimum_ray_y:raise ValueError('horizon_or_invalid_contact')
    depth=mount['height_m']/ray[1]
    if not .4<depth<max_depth:raise ValueError('contact_outside_prototype_depth')
    return rotation(np.radians(mount['yaw_deg']))@np.array([depth,-depth*ray[0],0])+mount['offset_W_ego_m']


def project_vehicle_height(box,profile,mount,vehicle_height):
    """Side vehicle centre from apparent body height, not silhouette bottom.

    Close side boxes do not identify a wheel/ground contact; their lower edge
    moves between wheels, bumper and truncated body. Keep this size prior
    explicit instead of deleting the car after a false ground intersection.
    """
    if not np.isfinite(vehicle_height) or vehicle_height<=0:raise ValueError('Positive vehicle height required')
    K,D=camera_matrix(profile);cx=(box[0]+box[2])/2
    rays=cv2.undistortPoints(np.array([[[cx,box[1]]],[[cx,box[3]]]],float),K,D).reshape(2,2)
    apparent=rays[1,1]-rays[0,1]
    if not np.isfinite(rays).all() or apparent<=.02:raise ValueError('Unresolved apparent vehicle height')
    depth=vehicle_height/apparent
    if not .4<depth<70:raise ValueError('Apparent height outside prototype depth')
    return rotation(np.radians(mount['yaw_deg']))@np.array([depth,-depth*float(rays[:,0].mean()),0])+mount['offset_W_ego_m']


def project_front(row,profile,mount,vehicle_height,image_width=1280,image_height=960,contact_uv=None):
    b=row['bbox_xyxy']
    clear=row['support']=='detector_observation' and 'truncated' not in row.get('quality_flags',[]) and b[3]-b[1]>=25 and b[2]-b[0]>=25 and min(b[0],b[1],image_width-b[2],image_height-b[3])>=6
    try:
        p=project_contact(b,profile,mount,max_depth=140.,minimum_ray_y=.003,contact_uv=contact_uv)
        if p[0]>70 and not clear:raise ValueError('far_contact_requires_clear_direct_observation')
        return p,'estimated_ground_contact' if p[0]<=70 else 'far_ground_contact_low_confidence'
    except ValueError as error:
        # A depth budget is not evidence for changing the distance model.
        # Switching at 70m previously inverted the order of adjacent trucks.
        if str(error)!='horizon_or_invalid_contact' or not clear:raise error
        return project_vehicle_height(b,profile,mount,vehicle_height),'observed_height_prior_after_unresolved_contact'


def asset_class(rows,camera,available,image_width=1280,image_height=960):
    clear=[r for r in rows if r['support']=='detector_observation' and 'truncated' not in r.get('quality_flags',[]) and min(r['bbox_xyxy'][0],r['bbox_xyxy'][1],image_width-r['bbox_xyxy'][2],image_height-r['bbox_xyxy'][3])>6]
    strong_truck=[r for r in rows if r['support']=='detector_observation' and r['class_id']==7 and r.get('score',0)>=.75]
    if len(clear)>=2 and len(strong_truck)>=5 and all(r['class_id']==7 for r in clear):
        aspect=np.median([(r['bbox_xyxy'][2]-r['bbox_xyxy'][0])/max(1,r['bbox_xyxy'][3]-r['bbox_xyxy'][1]) for r in clear])
        if aspect>2 and camera=='front':return ('heavy_truck' if 'heavy_truck' in available else 'box_truck'),'two_clear_side_views_plus_repeated_high_confidence_truck'
    if len(clear)<3:return 'sedan','unresolved_partial_body_conservative_car_prior'
    classes=[r['class_id'] for r in clear];dominant=max(set(classes),key=classes.count)
    # Many tiny distant boxes must not outvote repeated larger, clearer body
    # views later in this offline track. A wrong body height also changes depth.
    close=sorted(clear,key=lambda r:(r['bbox_xyxy'][2]-r['bbox_xyxy'][0])*(r['bbox_xyxy'][3]-r['bbox_xyxy'][1]),reverse=True)[:max(5,math.ceil(len(clear)*.2))]
    reliable=[r for r in close if r.get('score',0)>=.65 and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=60 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=50]
    if len(reliable)>=5:
        winner=max(set(r['class_id'] for r in reliable),key=lambda k:sum(r['class_id']==k for r in reliable))
        if winner in (5,7) and sum(r['class_id']==winner for r in reliable)/len(reliable)>=.9:
            dominant=winner
    if dominant==5 and 'bus' in available:return 'bus','repeated_clear_bus_class'
    if dominant==7:
        return ('box_truck' if camera=='front' and 'box_truck' in available else 'truck'),'repeated_clear_truck_class'
    boxes=np.array([r['bbox_xyxy'] for r in clear]);large=(boxes[:,2]-boxes[:,0])>100
    if camera=='front' and sum(large)>=3 and np.median((boxes[large,3]-boxes[large,1])/(boxes[large,2]-boxes[large,0]))>.9:
        return 'suv','clear_front_body_aspect_prior'
    return 'sedan','repeated_clear_car_class'


def short_crossing_candidate(rows):
    observed=[r for r in rows if r['support']=='detector_observation' and r.get('score',0)>=.6
              and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=70 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=35]
    return len(observed)>=3 and observed[-1]['camera_time_s']-observed[0]['camera_time_s']>=.4


def trajectory_at(actor,t,restore_maneuver=False):
    times=np.array([r['time_s'] for r in actor['samples']]);positions=np.array([r.get('position_before_ego_maneuver',r['position_W']) if restore_maneuver else r['position_W'] for r in actor['samples']])
    return np.array([np.interp(t,times,positions[:,k]) for k in range(3)])


def export_frame_count(base_report,times,fps):
    duration=base_report.get('source_duration_s')
    if duration is not None:
        if not np.isfinite(duration) or duration<=0:raise ValueError('Invalid source duration')
        return math.ceil(duration*fps)
    return int((times[-1]-times[0])*fps)+1


def merge_anchor_samples(anchor,members):
    outside=[r for a in members if a is not anchor for r in a['samples'] if r['time_s']<anchor['start_s'] or r['time_s']>anchor['end_s']]
    bins={}
    for r in outside:bins.setdefault(round(r['time_s'],6),r)
    return sorted(anchor['samples']+list(bins.values()),key=lambda r:r['time_s'])


def corridor_heading(velocity,road_slope,last):
    if np.linalg.norm(velocity[:2])<=1.5:return last
    angle=math.atan2(velocity[1],velocity[0]);angle=(angle+np.pi/2)%np.pi-np.pi/2
    tangent=math.atan(road_slope)
    # These actors have only same-direction corridor support. Near-zero
    # longitudinal motion plus contact jitter does not establish a 90deg turn.
    if velocity[0]<=1.5 or abs(angle-tangent)>np.pi/4:return tangent
    return angle


def lane_from_boundaries(y,boundaries):
    boundaries=np.asarray(boundaries,float)
    if len(boundaries)<2 or not np.isfinite(boundaries).all() or np.any(np.diff(boundaries)>=0):raise ValueError('Ordered road boundaries required')
    if y>boundaries[0]+.2 or y<boundaries[-1]-.2:return None
    for i in range(len(boundaries)-1):
        if boundaries[i+1]<=y<=boundaries[i]:return i
    return 0 if y>boundaries[0] else len(boundaries)-2


def side_fragment_has_geometry(rows,image_width,image_height):
    boxes=np.asarray([r['bbox_xyxy'] for r in rows]);w=boxes[:,2]-boxes[:,0];h=boxes[:,3]-boxes[:,1]
    narrow=np.median(w)<image_width*.25 and np.median(w/np.maximum(h,1))<.5
    if not narrow:return True
    # A tall border sliver (often a cropped parked body or false detection)
    # cannot supply full vehicle height. Wait for actual fuller body evidence
    # somewhere in the same offline track rather than snapping it into a lane.
    fuller=(w/image_width>.12)&(w/np.maximum(h,1)>.6)&(h/image_height<.9)
    return bool(np.count_nonzero(fuller)>=3)


def unresolved_front_border_fragment(rows,image_width):
    observed=[r for r in rows if r.get('support')=='detector_observation']
    if not observed:return False
    boxes=np.array([r['bbox_xyxy'] for r in observed]);widths=boxes[:,2]-boxes[:,0]
    border=(boxes[:,0]<=6)|(image_width-boxes[:,2]<=6)
    clear=(~border)&(widths>=80)&(boxes[:,3]-boxes[:,1]>=60)
    return bool(np.mean(border)>=.8 and np.max(widths)<image_width*.35 and np.count_nonzero(clear)<3)


def actor_corridor_lane(actor,boundaries,road_slope,road=None):
    local=road_coordinates(road or {'heading_evidence':{'slope':road_slope}},[r['position_W'] for r in actor['samples']])
    y=float(np.median(local[:,1]))
    if actor.get('side_paint_geometry',{}).get('occluded_contact_lane_supported'):
        return actor['side_paint_geometry']['lane'],'repeated_same_camera_occlusion_with_paint_supported_foreground_lane',y
    if road and road.get('lane_geometry_status')=='unknown_route_display_only':
        clear=[r for r in actor['samples'] if r.get('support')=='detector_observation' and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>80 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>75]
        return (-2,'unresolved_road_scope_no_lane_prior',y) if len(clear)>=3 else (None,'unresolved_road_small_unclear_actor',y)
    if road and crossing_motion(actor,road):return -2,'camera_motion_crosses_supported_junction',y
    if road and road.get('left_pockets'):
        membership=[lane_at(road,float(q[0]),float(q[1])) for q in local];known=sum(lane is not None for lane in membership)
        inside=[(r,q) for r,q,lane in zip(actor['samples'],local,membership) if lane==-1]
        if len(inside)>=8 and len(inside)>=known*.6:
            t=np.array([r['time_s'] for r,q in inside]);s=np.array([q[0] for r,q in inside])
            velocity=float(np.linalg.lstsq(np.c_[t-t[0],np.ones(len(t))],s,rcond=None)[0][0])
            if velocity>=-1.5:return -1,'dominant_known_membership_in_local_left_pocket',y
    if road and actor['camera_id']=='front' and road.get('left_boundary_role')=='camera_observed_yellow_divider_estimated_geometry':
        beyond=np.array([d-boundaries_at(road,float(s))[0] for s,d,h in local])
        if float(np.median(beyond))>.25:
            return None,'beyond_observed_divider_without_supported_auxiliary_membership',y
    if road and road.get('lane_sections'):
        lanes=[lane_at(road,float(s),float(d)) for s,d,_ in local]
        valid=[lane for lane in lanes if lane is not None and lane>=0]
        if len(valid)>=max(3,len(lanes)*.2):return max(set(valid),key=valid.count),'inside_supported_local_road_sections',y
    lane=lane_from_boundaries(y,boundaries)
    if lane is not None:return lane,'inside_corridor',y
    if road and road.get('junctions'):
        clear=[r for r in actor['samples'] if inside_junction(road,r['position_W'],5.) and r.get('support')=='detector_observation' and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>70]
        if len(clear)>=8:return -2,'observed_junction_crossing_unconstrained_by_corridor',y
    if road and road.get('left_pockets'):
        rows=actor['samples'];inside=[(r,q) for r,q in zip(rows,local) if lane_at(road,float(q[0]),float(q[1]))==-1]
        if len(inside)>=8 and len(inside)>=len(rows)*.6:
            t=np.array([r['time_s'] for r,q in inside]);x=np.array([q[0] for r,q in inside])
            velocity=float(np.linalg.lstsq(np.c_[t-t[0],np.ones(len(t))],x,rcond=None)[0][0])
            if velocity>=-1.5:return -1,'inside_local_left_pocket',y
    if road and road.get('reference_line') and boundaries[-1]-road.get('lane_width_m_prior',3.5)<=y<boundaries[-1]:
        # The painted driving lane is not a vehicle visibility boundary. A
        # clear car parked at its right curb must remain in the world without
        # inventing an extra lane or pulling that car to the lane centre.
        clear=[r for r in actor['samples'] if r.get('support')=='detector_observation'
               and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=100 and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=75
               and min(r['bbox_xyxy'][0],r['bbox_xyxy'][1],r.get('image_width',1280)-r['bbox_xyxy'][2],r.get('image_height',960)-r['bbox_xyxy'][3])>5]
        if len(clear)>=8 and clear[-1]['time_s']-clear[0]['time_s']>=1.2:
            return -2,'repeated_clear_roadside_vehicle; no_extra_lane_or_lane_prior',y
    if actor['camera_id'] not in ('left_repeater','right_repeater'):return None,'outside_corridor',y
    boxes=np.array([r['bbox_xyxy'] for r in actor['samples']]);widths=boxes[:,2]-boxes[:,0];heights=boxes[:,3]-boxes[:,1]
    # A modest side contact uncertainty must not delete a large visible car.
    # Small background detections remain outside the road scope.
    foreground=np.median(widths)>180 and np.median(heights)>130
    if foreground and boundaries[-1]-1.0<=y<=boundaries[0]+1.0:
        return (0 if y>boundaries[0] else len(boundaries)-2),'foreground_side_boundary_uncertainty',y
    return None,'outside_corridor',y


def actor_lane_priors(rows,boundaries,road_slope,fallback_lane,dwell_s=.6,margin_m=.2,return_review=False,road=None):
    if road and sum(bool(r.get('paint_lane_supported')) for r in rows)>=len(rows)*.8:
        local=road_coordinates(road,[r['position_W'] for r in rows]);lanes=np.array([r.get('paint_lane_index',fallback_lane) for r in rows],int)
        targets=np.array([sum(lane_bounds(road,float(q[0]),int(lane)))/2 for q,lane in zip(local,lanes)])
        review=dict(events=[],support='independent_side_paint_geometry_with_explicit_occlusion_uncertainty',clear_observations=sum(bool(r.get('paint_lane_supported')) for r in rows))
        return (lanes,targets,review) if return_review else (lanes,targets)
    if road and road.get('reference_line') and road.get('left_pockets'):
        measured=road_coordinates(road,[r['position_W'] for r in rows])
        pocket_samples=sum(lane_at(road,float(s),float(d))==-1 for s,d,h in measured)
        if fallback_lane==-1 or pocket_samples>=max(3,len(rows)*.2):
            result=pocket_actor_priors(rows,road,fallback_lane,dwell_s,margin_m)
            return result if return_review else result[:2]
    if road and road.get('lane_sections'):
        local=road_coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in rows]);lanes=np.zeros(len(rows),int);targets=local[:,1].copy();reviews=[];i=0
        section_ids=[(section_at(road,float(s)) or {}).get('id') for s in local[:,0]]
        while i<len(rows):
            j=i+1
            while j<len(rows) and section_ids[j]==section_ids[i]:j+=1
            bounds=boundaries_at(road,float(local[i,0]));part=[dict(r,position_W=[p[0],p[1],0.],unrefined_position_W=[p[0],p[1],0.]) for r,p in zip(rows[i:j],local[i:j])]
            candidate=lane_from_boundaries(float(np.median(local[i:j,1])),bounds)
            fallback=candidate if candidate is not None else min(max(0,fallback_lane),len(bounds)-2)
            ll,tt,rr=estimate_actor_lanes(part,bounds,0.,fallback,dwell_s,margin_m)
            lanes[i:j]=ll;targets[i:j]=tt;reviews.append(dict(rr,road_section_id=section_ids[i]));i=j
        review=dict(events=[dict(e,road_section_id=r['road_section_id']) for r in reviews for e in r['events']],
                    rejected_candidates=[e for r in reviews for e in r['rejected_candidates']],clear_observations=sum(r['clear_observations'] for r in reviews),
                    support='section_local_clear_observations_or_explicit_prior',observer='fixed_world_road_frame; ego_lane_not_used',sections=reviews)
        return (lanes,targets,review) if return_review else (lanes,targets)
    if road and road.get('reference_line'):
        local=road_coordinates(road,[r.get('unrefined_position_W',r['position_W']) for r in rows])
        rows=[dict(r,position_W=[p[0],p[1],0.],unrefined_position_W=[p[0],p[1],0.]) for r,p in zip(rows,local)]
        road_slope=0.
    lanes,targets,review=estimate_actor_lanes(rows,boundaries,road_slope,fallback_lane,dwell_s,margin_m)
    return (lanes,targets,review) if return_review else (lanes,targets)


def compatible_fov_exit(first,second):
    a,b=first['samples'][-1],second['samples'][0]
    if 'bbox_xyxy' not in a or 'bbox_xyxy' not in b:return False
    ca,cb=first['camera_id'],second['camera_id'];wa=a['image_width'];wb=b['image_width']
    ba,bb=a['bbox_xyxy'],b['bbox_xyxy']
    if ca=='front' and cb=='right_repeater':return ba[2]>=wa*.98 and bb[0]<=wb*.02
    if ca=='right_repeater' and cb=='front':return ba[0]<=wa*.02 and bb[2]>=wb*.98
    if ca=='front' and cb=='left_repeater':return ba[0]<=wa*.02 and bb[2]>=wb*.98
    if ca=='left_repeater' and cb=='front':return ba[2]>=wa*.98 and bb[0]<=wb*.02
    return False


def box_iou(a,b):
    a,b=np.asarray(a),np.asarray(b);size=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]));intersection=float(np.prod(size))
    return intersection/max(1,float(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2]))-intersection)


def identity_observation_usable(row,width,height):
    b=row['bbox_xyxy']
    return row['support']=='detector_observation' and 'truncated' not in row.get('quality_flags',[]) and min(b[0],b[1],width-b[2],height-b[3])>max(2,min(width,height)*.005) and b[2]-b[0]>=40 and b[3]-b[1]>=24


def consolidate_camera_fragments(actors):
    """Join only image-overlapping fragments, independently of world priors.

    A large nearby vehicle can be split into car/truck detections. Colour or
    shared lane alone is insufficient: require repeated box agreement, or a
    short endpoint gap with substantial box overlap.
    """
    parent={a['id']:a['id'] for a in actors};evidence=[]
    def root(k):
        while parent[k]!=k:k=parent[k]
        return k
    for i,a in enumerate(actors):
        for b in actors[i+1:]:
            if a['camera_id']!=b['camera_id'] or a['frozen_lead'] or b['frozen_lead']:continue
            first,second=sorted([a,b],key=lambda x:x['start_s']);overlaps=[];observed_conflicts=0
            for r in first['samples']:
                s=min(second['samples'],key=lambda s:abs(s['time_s']-r['time_s']))
                if abs(s['time_s']-r['time_s'])<.06:
                    overlap=box_iou(r['bbox_xyxy'],s['bbox_xyxy']);overlaps.append(overlap)
                    if r['support']=='detector_observation' and s['support']=='detector_observation' and overlap<.35:observed_conflicts+=1
            repeated=len(overlaps)>=3 and np.median(overlaps)>.65
            gap=second['start_s']-first['end_s']
            endpoint=0<=gap<=.45 and box_iou(first['samples'][-1]['bbox_xyxy'],second['samples'][0]['bbox_xyxy'])>.5
            # Shared boxes followed by divergent simultaneous boxes indicate
            # a stolen identity, not two interchangeable whole trajectories.
            contradictory=observed_conflicts>=2
            identical_detection=bool(overlaps and max(overlaps)>.98 and not contradictory)
            if not contradictory and (identical_detection or ((repeated or endpoint) and appearance_distance(a['appearance'],b['appearance'])<.5)):
                parent[root(b['id'])]=root(a['id']);evidence.append({'a':a['id'],'b':b['id'],'kind':'same_camera_box_continuity','selected':True})
    groups=defaultdict(list)
    for a in actors:groups[root(a['id'])].append(a)
    result=[]
    for members in groups.values():
        out=dict(next((a for a in members if not a.get('scope_pending')),members[0]));out['id']=members[0]['id'];bins=defaultdict(list)
        out['scope_pending']=all(a.get('scope_pending',False) for a in members)
        for a in members:
            for s in a['samples']:bins[round(s['time_s'],2)].append(s)
        out['samples']=[max(rs,key=lambda r:(r['support']=='detector_observation',r.get('quality',1))) for _,rs in sorted(bins.items())]
        out.update(start_s=min(a['start_s'] for a in members),end_s=max(a['end_s'] for a in members),local_ids=[a['id'] for a in members])
        if any('identity_evidence' in a for a in members):out['identity_evidence']={'usable_observations':sum(a.get('identity_evidence',{}).get('usable_observations',0) for a in members),
            'observed_samples':sum(a.get('identity_evidence',{}).get('observed_samples',0) for a in members),'observed_span_s':out['end_s']-out['start_s'],
            'wide_side_body_observations':sum(a.get('identity_evidence',{}).get('wide_side_body_observations',0) for a in members),
            'wide_side_body_span_s':max(a.get('identity_evidence',{}).get('wide_side_body_span_s',0) for a in members)}
        result.append(out)
    return result,evidence


def clear_future_motion(first,second):
    """Use a short, clear future view instead of a clipped entry contact.

    Bounded extrapolation for a side->front handoff, never a camera offset.
    """
    if first['camera_id'] not in ('left_repeater','right_repeater') or second['camera_id']!='front':return None
    clear=[r for r in second['samples'] if identity_observation_usable(r,r.get('image_width',1280),r.get('image_height',960))]
    if not clear or clear[0]['time_s']-second['start_s']>3.2 or clear[0]['time_s']-first['end_s']>4.5:return None
    rows=[r for r in clear if r['time_s']<=clear[0]['time_s']+1.1]
    if len(rows)<5 or rows[-1]['time_s']-rows[0]['time_s']<.8:return None
    t=np.array([r['time_s']-rows[0]['time_s'] for r in rows]);p=np.array([r['position_W'] for r in rows]);X=np.c_[t,np.ones(len(t))]
    fit=np.linalg.lstsq(X,p,rcond=None)[0];residual=float(np.quantile(np.linalg.norm(X@fit-p,axis=1),.9))
    if residual>.75 or np.linalg.norm(fit[0,:2])>=45:return None
    expected=fit[1]+fit[0]*(first['end_s']-rows[0]['time_s'])
    return dict(first_clear_s=rows[0]['time_s'],velocity_W=fit[0].tolist(),residual_p90_m=residual,
                distance_m=float(np.linalg.norm(expected-first['samples'][-1]['position_W'])))


def reconstruct_handoff(first,second,fit):
    """Correct only clipped prefix geometry; keep every clear sample exact."""
    if fit.get('kind')=='bidirectional_clear_windows':
        repair_clear_handoff(first,second,fit);return
    boundary=fit['first_clear_s'];anchor=next(r for r in second['samples'] if r['time_s']==boundary)
    for row in second['samples']:
        if row['time_s']>=boundary:break
        row['position_W']=list(row['position_W'])
        row['position_W'][0]=anchor['position_W'][0]+fit['velocity_W'][0]*(row['time_s']-boundary)
        row.update(support='offline_future_clear_geometry',uses_future_observations=True,
                   geometry_reconstruction='bounded_future_clear_view_velocity; lateral_prior_unchanged')


def fill_handoff_gaps(actor,budget):
    rows=actor['samples'];added=[];review=[]
    for i,(a,b) in enumerate(zip(rows,rows[1:])):
        gap=b['time_s']-a['time_s']
        if not .45<gap<=budget or a['camera_id']==b['camera_id']:continue
        def velocity(pool,anchor):
            pool=[r for r in pool if r['camera_id']==anchor['camera_id'] and abs(r['time_s']-anchor['time_s'])<=1.1]
            if len(pool)<3:return None
            t=np.array([r['time_s']-anchor['time_s'] for r in pool]);p=np.array([r['position_W'] for r in pool])
            return np.linalg.lstsq(np.c_[t,np.ones(len(t))],p,rcond=None)[0][0].tolist()
        anchors=[WorldAnchor(actor['global_id'],r['time_s'],tuple(r['position_W']),velocity(pool,r),r.get('lane_index'),(r['frame_id'],)) for r,pool in [(a,rows[:i+1]),(b,rows[i+1:])]]
        times=np.arange(a['time_s']+.1,b['time_s']-.00001,.1)
        body_delta=float(np.arctan2(np.sin(b['yaw_rad']-a['yaw_rad']),np.cos(b['yaw_rad']-a['yaw_rad'])))
        for r in complete_gap(*anchors,times,max_gap_s=budget):
            r.update(support='offline_two_endpoint_gap',camera_id='reconstructed',frame_id=None,
                     lane_index=a.get('lane_index') if a.get('lane_index')==b.get('lane_index') else None,
                     yaw_rad=float(a['yaw_rad']+body_delta*(r['time_s']-a['time_s'])/gap),
                     heading_support='two_observed_body_directions_not_gap_velocity')
            if a.get('stationary_handoff_support') and b.get('stationary_handoff_support'):r['stationary_handoff_support']=True
            added.append(r)
        review.append(dict(from_camera=a['camera_id'],to_camera=b['camera_id'],start_s=a['time_s'],end_s=b['time_s'],added_samples=len(times)))
    actor['samples']=sorted(rows+added,key=lambda r:r['time_s']);actor['handoff_gaps']=review


def support_intermediate_bridges(edges,actors,policy=None):
    """A clear later front view can resolve an earlier partial side view.

    Require both FOV transitions, an independently selected side->front edge,
    broad sustained side body support and agreement of the two front views.
    No relaxation for an isolated cropped future glimpse.
    """
    by_id={a['id']:a for a in actors}
    confirmed={e['from']:e for e in edges if e.get('selected') and e.get('future_clear_motion') and e['kind']=='future_reappearance_with_fov_boundary'}
    for edge in edges:
        if edge.get('identity_eligible') or edge['kind']!='future_reappearance_with_fov_boundary':continue
        first,middle=by_id[edge['from']],by_id[edge['to']];next_edge=confirmed.get(middle['id']);proof=middle.get('identity_evidence',{})
        if first['camera_id']!='front' or not next_edge or proof.get('wide_side_body_observations',0)<8 or proof.get('wide_side_body_span_s',0)<1.4:continue
        last=by_id[next_edge['to']]
        if last['camera_id']!='front' or first['end_s']>=last['start_s'] or first.get('identity_evidence',{}).get('usable_observations',0)<3:continue
        app=appearance_distance(first['appearance'],last['appearance'])
        if app>=.4:continue
        edge.update(identity_eligible=True,bridge_future_clear_id=last['id'],bridge_front_appearance_distance=app)
    return select_handoffs(edges,actors,margin=(policy or {}).get('ambiguity_margin',.15))


def association_edges(actors,policy=None):
    """Whole-tracklet motion agreement; weak colour alone cannot merge IDs."""
    edges=[]
    for i,a in enumerate(actors):
        for b in actors[i+1:]:
            if a['camera_id']==b['camera_id']:continue
            first,second=(a,b) if a['start_s']<b['start_s'] else (b,a)
            pending_a=bool(a.get('scope_pending'));pending_b=bool(b.get('scope_pending'))
            if pending_a and pending_b:continue
            deferred_scope_link=pending_a!=pending_b
            exit_lane=first.get('association_lane_end',first['lane_index']);entry_lane=second.get('association_lane_start',second['lane_index'])
            different_sections=first.get('association_section_end') is not None and second.get('association_section_start') is not None and first['association_section_end']!=second['association_section_start']
            lane_incompatible=exit_lane is None or entry_lane is None or (exit_lane!=entry_lane and not different_sections)
            if lane_incompatible and not deferred_scope_link:continue
            if ('bus' in (a.get('model'),b.get('model'))) and a.get('model')!=b.get('model'):continue
            start=max(a['start_s'],b['start_s']);end=min(a['end_s'],b['end_s'])
            app=appearance_distance(a['appearance'],b['appearance']);original_app=app;appearance_review=None
            if end-start>=.4:
                # Co-visible similar vehicles cannot establish a shared ID
                # with the uncalibrated front/side rig. Use actual handoffs.
                continue
            else:
                first,second=(a,b) if a['start_s']<b['start_s'] else (b,a)
                gap=second['start_s']-first['end_s']
                if not -.4<=gap<=4:continue
                points=first['samples'];v=(np.array(points[-1]['position_W'])-np.array(points[max(0,len(points)-6)]['position_W']))/max(.2,points[-1]['time_s']-points[max(0,len(points)-6)]['time_s'])
                expected=np.array(points[-1]['position_W'])+v*gap
                moving_distance=float(np.linalg.norm(expected-second['samples'][0]['position_W']))
                stopped_distance=float(np.linalg.norm(np.array(points[-1]['position_W'])-second['samples'][0]['position_W']))
                # A vehicle may accelerate while crossing the pillar blind
                # region. Use the next camera's future motion as well as the
                # previous camera's truncated-box velocity.
                future=[r for r in second['samples'] if r['time_s']<=second['start_s']+3.1]
                ft=np.array([r['time_s']-future[0]['time_s'] for r in future]);fp=np.array([r['position_W'] for r in future])
                fv=np.linalg.lstsq(np.column_stack([ft,np.ones(len(ft))]),fp,rcond=None)[0][0] if len(future)>=3 else np.zeros(3)
                backward_distance=float(np.linalg.norm(np.array(points[-1]['position_W'])-(fp[0]-fv*gap))) if np.linalg.norm(fv)<45 else float('inf')
                fov=compatible_fov_exit(first,second)
                clear_fit=clear_future_motion(first,second)
                distance=clear_fit['distance_m'] if clear_fit else min(moving_distance,stopped_distance,backward_distance);p95=distance
                bidirectional=fit_clear_handoff(first,second,identity_observation_usable) if fov else None
                if bidirectional and bidirectional['distance_m']<distance:
                    clear_fit=bidirectional;distance=clear_fit['distance_m'];p95=distance
                strong_clear_link=bool(fov and bidirectional and bidirectional['distance_m']<3.
                                       and bidirectional['velocity_disagreement_mps']<2.)
                if deferred_scope_link and not strong_clear_link:continue
                if strong_clear_link and app>=.65:
                    appearance_review=review_handoff_appearance(first,second)
                    if appearance_review is not None:app=appearance_review['distance']
                limit=6 if fov else 3
                eligible=distance<limit and app<(.65 if fov else .32);kind='future_reappearance_with_fov_boundary' if fov else 'future_reappearance_motion_agreement';cost=distance/limit+app+.1*max(gap,0)
            if eligible:edges.append({'a':a['id'],'b':b['id'],'from':first['id'],'to':second['id'],'kind':kind,'distance_m_prior':distance,'p95_m_prior':p95,'appearance_distance':app,'cost':cost,
                'geometry_basis':'pre_road_prior_positions','exit_lane':exit_lane,'entry_lane':entry_lane,
                'scope_deferred_until_identity':deferred_scope_link,'original_appearance_distance':original_app,
                'clear_view_appearance_review':appearance_review,
                'future_clear_motion':clear_fit,'identity_eligible':identity_evidence_sufficient(first,second,policy)})
    edges=select_handoffs(edges,actors,margin=(policy or {}).get('ambiguity_margin',.15))
    return support_intermediate_bridges(edges,actors,policy)


def persistent_yellow_boundary(points):
    bins=defaultdict(set)
    for y,fid in points:bins[round(y/.25)*.25].add(fid)
    persistent=[key for key in bins if len(bins[key])>=5]
    if not persistent:return None
    mode=max(persistent,key=lambda key:sum(abs(y-key)<.125 for y,_ in points))
    support={fid for y,fid in points if abs(y-mode)<.3};values=[y for y,_ in points if abs(y-mode)<.3]
    if len(support)<5 or len(values)<100:return None
    return {'boundary_y':float(np.median(values)),'support_frames':len(support),'pixel_samples':len(values),'camera':'left_repeater'}


def side_boundary_evidence(package,frames,observations,ego_times,ego_positions,ego_yaw,profile,mount):
    """Fallback for a front centreline occluded by queues, using side paint.

    Require persistent yellow ground pixels after dynamic-object masking.
    This establishes a corridor limit, not a new lane from an isolated car.
    """
    points=[]
    for frame in frames[::5]:
        image=np.asarray(Image.open(package/frame['image']).convert('RGB'));h,w=image.shape[:2]
        hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV);mask=(hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>70)&(hsv[:,:,2]>60);mask[:int(h*.5)]=False
        for row in observations[frame['frame_id']]:
            x1,y1,x2,y2=np.rint(row['bbox_xyxy']).astype(int);mask[max(0,y1-4):min(h,y2+4),max(0,x1-4):min(w,x2+4)]=False
        ys,xs=np.where(mask);step=max(1,len(xs)//200);t=frame['camera_time_s']
        ep=np.array([np.interp(t,ego_times,ego_positions[:,k]) for k in range(3)]);yaw=np.interp(t,ego_times,ego_yaw)
        for x,y in zip(xs[::step],ys[::step]):
            try:world=ep+rotation(yaw)@project_contact([x,y,x,y],profile,mount)
            except ValueError:continue
            if .5<world[1]<14 and np.linalg.norm(world-ep)<20:
                points.append((float(world[1]),frame['frame_id']))
    return persistent_yellow_boundary(points)


def road_heading_from_stations(points,left_boundary):
    """Independent paint alignment, requiring longitudinal coverage.

    Equal weight per road station avoids a long stationary queue dominating
    the fit. A short clip cannot establish road heading and retains its prior.
    """
    bins=defaultdict(list)
    for x,y,fid,camera in points:bins[(int(round(x/20))*20,camera)].append((y,fid))
    stations=[]
    for x in sorted({x for x,_ in bins}):
        if x<30:continue
        choices=[]
        for camera in ('front','left_repeater'):
            rows=bins.get((x,camera),[])
            if len(rows)>=20 and len({fid for _,fid in rows})>=2:choices.append((camera,rows))
        if not choices:continue
        camera,rows=choices[0];stations.append({'x':x,'y':float(np.percentile([y for y,_ in rows],25)),'camera':camera,'frames':len({fid for _,fid in rows})})
    if len(stations)<4 or stations[-1]['x']-stations[0]['x']<60:return {'slope':0.,'support_stations':stations,'status':'insufficient_station_span'}
    slopes=[(s['y']-left_boundary)/s['x'] for s in stations];slope=float(np.median(slopes))
    if abs(slope)>.08:return {'slope':0.,'support_stations':stations,'status':'outside_straight_corridor_scope'}
    return {'slope':slope,'support_stations':stations,'status':'estimated_from_camera_ground_paint'}


def ground_road_heading(package,frames,observations,times,ego,yaw,profiles,mounts,left_boundary):
    points=[]
    for camera in ('front','left_repeater'):
        for f in [f for f in frames if f['camera_id']==camera][::5]:
            image=np.asarray(Image.open(package/f['image']).convert('RGB'));h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
            mask=(hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>70)&(hsv[:,:,2]>60);mask[:int(h*.65)]=False;mask[int(h*.96):]=False
            for r in observations[f['frame_id']]:
                x1,y1,x2,y2=np.rint(r['bbox_xyxy']).astype(int);mask[max(0,y1-5):min(h,y2+5),max(0,x1-5):min(w,x2+5)]=False
            ys,xs=np.where(mask);step=max(1,len(xs)//200);t=f['camera_time_s'];ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)]);angle=np.interp(t,times,yaw)
            for x,y in zip(xs[::step],ys[::step]):
                try:world=ep+rotation(angle)@project_contact([x,y,x,y],profiles[camera],mounts[camera])
                except ValueError:continue
                if .5<world[1]-ep[1]<8 and np.linalg.norm(world-ep)<25:points.append((float(world[0]),float(world[1]),f['frame_id'],camera))
    return road_heading_from_stations(points,left_boundary)


def fit_right_edge(points):
    bins=defaultdict(list)
    for x,y,fid in points:bins[round(x/40)*40].append((y,fid))
    stations=[(x,float(np.median([y for y,_ in rs]))) for x,rs in sorted(bins.items()) if len(rs)>=30 and len({fid for _,fid in rs})>=2]
    if len(stations)<4 or stations[-1][0]-stations[0][0]<80:return None
    a=np.asarray(stations);design=np.column_stack([a[:,0]/100,np.ones(len(a))]);weights=np.ones(len(a))
    for _ in range(8):
        coef=np.linalg.lstsq(design*weights[:,None]**.5,a[:,1]*weights**.5,rcond=None)[0];residual=abs(a[:,1]-design@coef);weights=np.minimum(1,.5/np.maximum(residual,1e-6))
    slope=float(coef[0]/100);edge=float(coef[1])
    if abs(slope)>.08:return None
    return {'slope':slope,'right_boundary_y':edge,'support_stations':stations,'status':'camera_right_ground_edge'}


def right_edge_evidence(package,frames,observations,times,ego,yaw,profile,mount):
    points=[]
    for f in frames[::5]:
        im=np.asarray(Image.open(package/f['image']).convert('RGB'));h,w=im.shape[:2];hsv=cv2.cvtColor(im,cv2.COLOR_RGB2HSV)
        mask=(hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>70)&(hsv[:,:,2]>60);mask[:int(h*.6)]=False;mask[int(h*.97):]=False;mask[:,:int(w*.52)]=False
        for r in observations[f['frame_id']]:
            x1,y1,x2,y2=np.rint(r['bbox_xyxy']).astype(int);mask[max(0,y1-5):min(h,y2+5),max(0,x1-5):min(w,x2+5)]=False
        ys,xs=np.where(mask);step=max(1,len(xs)//150);t=f['camera_time_s'];ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)]);angle=np.interp(t,times,yaw)
        for x,y in zip(xs[::step],ys[::step]):
            try:relative=project_contact([x,y,x,y],profile,mount)
            except ValueError:continue
            if 3<relative[0]<25 and -8<relative[1]<-.5:
                p=ep+rotation(angle)@relative;points.append((float(p[0]),float(p[1]),f['frame_id']))
    return fit_right_edge(points)


def road_evidence(package,frames,observations,ego_times,ego_positions,ego_yaw,profile,width,side_boundary=None):
    K,D=camera_matrix(profile);yellow_y=[];white_y=[];stations=[]
    for frame in frames[::5]:
        image=np.asarray(Image.open(package/frame['image']).convert('RGB'));h,w=image.shape[:2]
        hsv=cv2.cvtColor(image,cv2.COLOR_RGB2HSV);allowed=np.zeros((h,w),np.uint8);allowed[int(h*.56):int(h*.96),:]=255
        for row in observations[frame['frame_id']]:
            x1,y1,x2,y2=np.rint(row['bbox_xyxy']).astype(int);allowed[max(0,y1-4):min(h,y2+4),max(0,x1-4):min(w,x2+4)]=0
        yellow=((hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>70)&(hsv[:,:,2]>60)&(allowed>0));yellow[:,int(w*.5):]=False
        ys,xs=np.where(yellow);take=np.arange(0,len(xs),max(1,len(xs)//200))
        t=frame['camera_time_s'];ey=np.interp(t,ego_times,ego_yaw);ep=np.array([np.interp(t,ego_times,ego_positions[:,k]) for k in range(3)])
        for x,y in zip(xs[take],ys[take]):
            ray=cv2.undistortPoints(np.array([[[x,y]]],float),K,D).reshape(2)
            if ray[1]<=.02:continue
            z=1.3/ray[1]
            if 3<z<25:
                world=ep+rotation(ey)@np.array([z,-z*ray[0],0])
                if .5<world[1]<width*4:yellow_y.append(float(world[1]))
        white=((hsv[:,:,1]<60)&(hsv[:,:,2]>145)&(allowed>0)).astype(np.uint8)*255
        lines=cv2.HoughLinesP(cv2.Canny(white,60,160),1,np.pi/180,threshold=60,minLineLength=120,maxLineGap=15)
        if lines is not None:
            for x1,y1,x2,y2 in lines.reshape(-1,4):
                if abs(y2-y1)>abs(x2-x1)*.4:
                    for u in np.linspace(0,1,8):
                        ray=cv2.undistortPoints(np.array([[[x1+u*(x2-x1),y1+u*(y2-y1)]]],float),K,D).reshape(2)
                        if ray[1]<=.02:continue
                        z=1.3/ray[1];world=ep+rotation(ey)@np.array([z,-z*ray[0],0])
                        if 3<z<25 and abs(world[1])<10:white_y.append((float(world[1]),frame['frame_id']))
                if abs(y2-y1)>abs(x2-x1)*.15:continue
                ray=cv2.undistortPoints(np.array([[[(x1+x2)/2,(y1+y2)/2]]],float),K,D).reshape(2)
                if ray[1]<=.03:continue
                world=ep+rotation(ey)@np.array([1.3/ray[1],-1.3*ray[0]/ray[1],0])
                if 5<world[0]<70:stations.append((float(world[0]),frame['frame_id']))
    boundary=float(np.median(yellow_y)) if len(yellow_y)>=50 and np.percentile(yellow_y,75)-np.percentile(yellow_y,25)<2 else width/2
    boundary_support='camera_yellow_pixels' if boundary!=width/2 else 'initial_lane_width_prior'
    if boundary_support=='initial_lane_width_prior' and side_boundary is not None:
        boundary=side_boundary['boundary_y'];boundary_support='left_repeater_yellow_ground_pixels'
    bins=defaultdict(set)
    for s,fid in stations:bins[int(round(s/2))*2].add(fid)
    marking=max(bins,key=lambda b:len(bins[b])) if bins and max(map(len,bins.values()))>=3 else None
    white_bins=defaultdict(set)
    for y,fid in white_y:white_bins[round(y/.2)*.2].add(fid)
    def divider(expected):
        options=[y for y,support in white_bins.items() if abs(y-expected)<.8 and len(support)>=3]
        return max(options,key=lambda y:len(white_bins[y])) if options else expected
    inner_left=divider(width/2);inner_right=divider(-width/2)
    boundaries=[boundary]
    if boundary-inner_left>width*.7:boundaries.append(inner_left)
    boundaries.append(inner_right)
    # A width prior supplies scale, not evidence for an additional lane. The
    # old unconditional outer stripe created a second lane on every narrow
    # yellow/white street (and a through road inside parking areas).
    outer=inner_right-width
    outer_options=[y for y,support in white_bins.items() if abs(y-outer)<.6 and len(support)>=5]
    if outer_options:
        boundaries.append(max(outer_options,key=lambda y:len(white_bins[y])))
    return {'left_boundary_y':boundary,'boundary_support':boundary_support,'side_boundary_evidence':side_boundary,
        'yellow_pixel_samples':len(yellow_y),'lane_width_m_prior':width,'marking_station_x':marking,
        'marking_support_frames':len(bins[marking]) if marking is not None else 0,'marking_bins':[{'x':x,'frames':len(s)} for x,s in sorted(bins.items())],
        'white_boundary_support':[{'y':y,'frames':len(s)} for y,s in sorted(white_bins.items()) if len(s)>=3],
        'boundaries_y':boundaries,'topology':'estimated_corridor_with_visual_marking_gap_not_osm_junction'}


class SurroundRenderer(Renderer):
    def __init__(self,assets,road,render_style=None):
        style=render_style or {}
        super().__init__(assets['ego'],assets['van'],style.get('camera_eye',[-30.,0.,30.]),style.get('camera_target',[0.,0.,0.]),camera_zoom=style.get('camera_zoom',1.3))
        visual=Path(style.get('ego_visual_model',str(Path(assets['ego']).with_name('tesla_ego_model3.glb'))))
        self.ego_visual=visual if visual.is_file() else Path(assets['ego']);self.ego_lamps={}
        body=model_vertices(self.ego_visual,node_filter=lambda node,material:lamp_role(node,material) is None)
        if len(body):self.models[0]=self.mesh(body)
        for group,on,off in [('brake',[1.,.025,.025],[.12,.012,.012]),('left',[1.,.45,.015],[.12,.055,.003]),('right',[1.,.45,.015],[.12,.055,.003]),('reverse',[.9,.95,1.],[.08,.08,.08])]:
            select=lambda node,material,g=group:lamp_role(node,material)==g
            dark=model_vertices(self.ego_visual,node_filter=select,color_override=off)
            if len(dark):self.ego_lamps[group]=(self.mesh(dark),self.mesh(model_vertices(self.ego_visual,node_filter=select,color_override=on)))
        clay=style.get('surrounding_clay_rgb',[.70,.71,.73])
        road_color=np.asarray(style.get('road_rgb',[.035,.04,.05]),float)
        lane_color=style.get('lane_rgb',[.78,.80,.83])
        background=np.asarray(style.get('background_rgb',[.13,.14,.16]),float)
        apron_color=np.asarray(style.get('surrounding_pavement_rgb',(road_color+background)*.5),float)
        self.background_rgb=background*(.55+.45/np.linalg.norm([-.4,-.6,1]))
        self.text_color=tuple(round(c*255) for c in style.get('text_rgb',[.88,.90,.93]))
        self.secondary_text_color=tuple(round(c*255) for c in style.get('secondary_text_rgb',[.65,.69,.74]))
        self.ground=self.mesh(np.asarray([[x,y,-.03,0,0,1,*background] for x,y in [(-1000,-1000),(1000,-1000),(1000,1000),(-1000,-1000),(1000,1000),(-1000,1000)]],'f4'))
        # Blend contact shadows into this road colour instead of leaving
        # the old pale ellipses conspicuous on the darker surface.
        shadow=[]
        for i in range(40):
            a,b=i*2*np.pi/40,(i+1)*2*np.pi/40
            shadow.extend([[0,0,0,0,0,1,*(road_color*.65)],
                [2.8*np.cos(a),1.1*np.sin(a),0,0,0,1,*road_color],
                [2.8*np.cos(b),1.1*np.sin(b),0,0,0,1,*road_color]])
        self.shadow=self.mesh(np.asarray(shadow,'f4'))
        self.asset_mesh={};self.actor_rear_mesh={}
        for name,path in assets.items():
            if name=='ego':continue
            tail=model_vertices(path,node_filter=lambda node,material:rear_material(material),color_override=[.75,.04,.045])
            if len(tail):
                body=model_vertices(path,clay_color=clay,node_filter=lambda node,material:not rear_material(material))
                self.actor_rear_mesh[name]=(self.mesh(model_vertices(path,clay_color=clay,node_filter=lambda node,material:rear_material(material))),self.mesh(tail))
            else:body=model_vertices(path,clay_color=clay)
            self.asset_mesh[name]=self.mesh(body)
        vertices=[]
        surface_mesh=road.get('display_surface',{}).get('mesh',{})
        points=surface_mesh.get('vertices_W_xy',[])
        for triangle in surface_mesh.get('triangles_indices',[]):
            for i in triangle:
                x,y=points[i];vertices.append([x,y,-.025,0,0,1,*apron_color])
        for polygon in road.get('context_surface',{}).get('polygons_W_xy',[]):
            polygon=np.asarray(polygon);centre=polygon.mean(axis=0)
            for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
                for x,y in (centre,a,b):vertices.append([x,y,-.027,0,0,1,*apron_color])
        for triangle in road.get('display_surface',{}).get('triangles_W_xy',[]):
            for x,y in triangle:vertices.append([x,y,-.025,0,0,1,*apron_color])
        for x0,y0,x1,y1 in road.get('display_surface',{}).get('rectangles_W_xy',[]):
            for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y0),(x1,y1),(x0,y1)]:
                vertices.append([x,y,-.025,0,0,1,*apron_color])
        def quad(x1,x2,lo1,hi1,lo2,hi2,z,color,display_override=False):
            midpoint=(x1+x2)/2
            if not road_supported(road,midpoint):
                extent=road.get('reference_line',{}).get('supported_extent_m')
                if (z>-.015 and not display_override) or not extent or not extent[0]<=midpoint<=extent[1]:return
            for s,d in [(x1,lo1),(x2,lo2),(x2,hi2),(x1,lo1),(x2,hi2),(x1,hi1)]:
                x,y=road_world(road,s,d);vertices.append([x,y,z,0,0,1,*color])
        def rect(x1,x2,y1,y2,z,color):
            cuts=np.r_[np.arange(x1,x2,3.),x2] if road.get('reference_line') else [x1,x2]
            for a,b in zip(cuts,cuts[1:]):quad(a,b,y1,y2,y1,y2,z,color)
        bounds=road['boundaries_y'];left,right=bounds[0],bounds[-1];mark=road['marking_station_x']
        post=road.get('post_marking_boundaries_y',bounds)
        road_end=max(1000.,road.get('reference_line',{}).get('supported_extent_m',[0,0])[-1])
        terminals=[j['terminal_display_station_m'] for j in road.get('junctions',[]) if 'terminal_display_station_m' in j]
        if terminals:road_end=min(road_end,min(terminals))
        zones=[(s['station_range_m'][0],s['station_range_m'][1],s['boundaries_y']) for s in road['lane_sections'] if not s.get('boundary_profile')] if road.get('lane_sections') else [(-100,road_end,bounds)] if mark is None else [(-100,mark-2,bounds),(mark+12,road_end,post)]
        opp_cfg=road.get('opposing_display') or {}
        opp_enabled=bool(opp_cfg.get('enabled'))
        max_same_dir=opp_cfg.get('max_same_dir_lanes')
        single_lane_total=opp_cfg.get('total_lanes')==1
        for x1,x2,zone_bounds in zones:
            x2=min(x2,road_end)
            if x2<=x1:continue
            if single_lane_total and len(zone_bounds)>=3:
                # Single-lane street (OSM lanes=1): keep only the ego-side lane and discard phantom outer sidewalk/curb bands.
                zone_bounds=zone_bounds[-2:]
            left,right=zone_bounds[0],zone_bounds[-1]
            # When OSM confirms a narrower per-direction lane count (e.g. 1+1 two-way street)
            # and camera paint included the opposing lane beyond an unconfirmed centerline, split the
            # section cleanly at the inner divider instead of mirroring the already-included opposing lane.
            split_inside=(opp_enabled and max_same_dir is not None and road.get('left_boundary_role')=='inferred_extent_not_confirmed_centreline'
                          and len(zone_bounds)-1>max_same_dir and len(zone_bounds)>=3)
            if split_inside:
                split_idx=len(zone_bounds)-1-max_same_dir
                center_y=zone_bounds[split_idx]
                rect(x1,x2,right,left,-.02,road_color)
                rect(x1,x2,center_y-.075,center_y-.005,-.012,[.85,.68,.22])
                rect(x1,x2,center_y-.215,center_y-.145,-.012,[.85,.68,.22])
                rect(x1,x2,left-.045,left+.045,-.012,lane_color)
                rect(x1,x2,right-.045,right+.045,-.012,lane_color)
                for y in zone_bounds[1:-1]:
                    if abs(y-center_y)<1e-6:continue
                    for x in np.arange(-100,road_end,6):
                        a,b=max(x,x1),min(x+3,x2)
                        if b>a:rect(a,b,y-.055,y+.055,-.012,lane_color)
                continue
            zone_opp_w=max(3.0,float(left-right)) if opp_enabled else 0.
            rect(x1,x2,right,left,-.02,road_color)
            has_yellow=opp_enabled or road.get('left_boundary_role')!='inferred_extent_not_confirmed_centreline'
            left_lines=(left-.04,left-.18) if has_yellow else (left-.04,)
            left_color=[.85,.68,.22] if has_yellow else lane_color
            pockets=road.get('left_pockets',[])
            cuts=sorted({x1,x2,*[float(x) for x in np.arange(-100,road_end,3) if x1<x<x2],*[k[0] for p in pockets for k in p['width_knots'] if x1<k[0]<x2]})
            for a,b in zip(cuts,cuts[1:]):
                wa,wb=pocket_width(pockets,a),pocket_width(pockets,b)
                if max(wa,wb)>0:quad(a,b,left,left+wa,left,left+wb,-.02,road_color)
                if opp_enabled:
                    quad(a,b,left+wa,left+wa+zone_opp_w,left+wb,left+wb+zone_opp_w,-.0205,road_color)
                    quad(a,b,left+wa+zone_opp_w-.045,left+wa+zone_opp_w+.045,left+wb+zone_opp_w-.045,left+wb+zone_opp_w+.045,-.012,lane_color)
                    if int(np.floor((a+100)/3))%2==0:
                        for y_same in zone_bounds[1:-1]:
                            oy=left-y_same
                            quad(a,b,left+wa+oy-.055,left+wa+oy+.055,left+wb+oy-.055,left+wb+oy+.055,-.012,lane_color)
                for y in left_lines:quad(a,b,y+wa-.035,y+wa+.035,y+wb-.035,y+wb+.035,-.012,left_color)
                if max(wa,wb)>.3 and int(np.floor((a+100)/3))%2==0:
                    rect(a,b,left-.045,left+.045,-.012,lane_color)
            rect(x1,x2,right-.045,right+.045,-.012,lane_color)
            for y in zone_bounds[1:-1]:
                for x in np.arange(-100,road_end,6):
                    a,b=max(x,x1),min(x+3,x2)
                    if b>a:rect(a,b,y-.055,y+.055,-.012,lane_color)
        for section in road.get('lane_sections',[]):
            if not section.get('boundary_profile'):continue
            start,end=section['station_range_m'];end=min(end,road_end)
            cuts=sorted({start,end,*np.arange(start,end,3.),*[k['station_m'] for k in section['boundary_profile'] if start<k['station_m']<end]})
            for a,b in zip(cuts,cuts[1:]):
                ba,bb=profile_boundaries(section,a),profile_boundaries(section,b)
                quad(a,b,ba[-1],ba[0],bb[-1],bb[0],-.02,road_color)
                if opp_enabled:
                    owa,owb=max(3.0,float(ba[0]-ba[-1])),max(3.0,float(bb[0]-bb[-1]))
                    quad(a,b,ba[0],ba[0]+owa,bb[0],bb[0]+owb,-.0205,road_color)
                    quad(a,b,ba[0]+owa-.045,ba[0]+owa+.045,bb[0]+owb-.045,bb[0]+owb+.045,-.012,lane_color)
                    if int(np.floor(((a+b)/2+100)/3))%2==0:
                        for k in range(1,len(ba)-1):
                            if min(ba[k-1]-ba[k],ba[k]-ba[k+1],bb[k-1]-bb[k],bb[k]-bb[k+1])<.8:continue
                            oya,oyb=ba[0]-ba[k],bb[0]-bb[k]
                            quad(a,b,ba[0]+oya-.055,ba[0]+oya+.055,bb[0]+oyb-.055,bb[0]+oyb+.055,-.012,lane_color)
                has_yellow=opp_enabled or road.get('left_boundary_role')!='inferred_extent_not_confirmed_centreline'
                color=[.85,.68,.22] if has_yellow else lane_color
                if has_yellow:
                    quad(a,b,ba[0]-.075,ba[0]-.005,bb[0]-.075,bb[0]-.005,-.012,color)
                    quad(a,b,ba[0]-.215,ba[0]-.145,bb[0]-.215,bb[0]-.145,-.012,color)
                else:
                    quad(a,b,ba[0]-.05,ba[0]+.05,bb[0]-.05,bb[0]+.05,-.012,color)
                quad(a,b,ba[-1]-.045,ba[-1]+.045,bb[-1]-.045,bb[-1]+.045,-.012,lane_color)
                if int(np.floor(((a+b)/2+100)/3))%2==0:
                    for k in range(1,len(ba)-1):
                        if min(ba[k-1]-ba[k],ba[k]-ba[k+1],bb[k-1]-bb[k],bb[k]-bb[k+1])<.8:continue
                        quad(a,b,ba[k]-.055,ba[k]+.055,bb[k]-.055,bb[k]+.055,-.012,lane_color)
        if mark is not None:
            rect(mark-2,mark+12,bounds[-1]-5,bounds[0]+5,-.02,road_color);rect(mark-2.15,mark-1.85,bounds[-1],bounds[0],-.012,lane_color)
        # Fill the main corridor across broad uncertain junction association
        # scopes. This mesh never changes lane scopes or vehicle coordinates.
        for start,end in corridor_gaps(road):
            end=min(end,road_end)
            if end<=start:continue
            cuts=np.r_[np.arange(start,end,2.),end]
            for a,b in zip(cuts,cuts[1:]):
                ba,bb=display_bounds(road,a+1e-5),display_bounds(road,b-1e-5)
                owa=opposing_width(road,a+1e-5);owb=opposing_width(road,b-1e-5)
                quad(a,b,ba[-1],ba[0]+owa,bb[-1],bb[0]+owb,-.02,road_color,True)
                show_left,show_right=mouth_edges(road,(a+b)/2)
                if show_left:quad(a,b,ba[0]+owa-.045,ba[0]+owa+.045,bb[0]+owb-.045,bb[0]+owb+.045,-.011,lane_color,True)
                if show_right:quad(a,b,ba[-1]-.045,ba[-1]+.045,bb[-1]-.045,bb[-1]+.045,-.011,lane_color,True)
        for junction in road.get('junctions',[]):
            for outline in opening_display_polygons(road,junction):
                polygon=np.asarray(outline);centre=polygon.mean(axis=0)
                for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
                    for x,y in (centre,a,b):vertices.append([x,y,-.019,0,0,1,*road_color])
            for branch in junction.get('branch_display_hints',[]):
                outline=np.asarray(branch['polygon_W_xy']);origin=outline.mean(axis=0)
                for a,b in zip(outline,np.roll(outline,-1,axis=0)):
                    for x,y in (origin,a,b):vertices.append([x,y,-.02,0,0,1,*road_color])
        for start,end in branch_edges(road):
            delta=np.asarray(end)-start;normal=np.array([-delta[1],delta[0]])/np.linalg.norm(delta)*.045
            for point in [start-normal,end-normal,end+normal,start-normal,end+normal,start+normal]:
                vertices.append([*point,-.011,0,0,1,*lane_color])
        jmarks=junction_markings(road)
        yellow_line=[.85,.68,.22]
        for hub in jmarks['hubs']:
            poly=np.asarray(hub);c=poly.mean(axis=0)
            for a,b in zip(poly,np.roll(poly,-1,axis=0)):
                for x,y in (c,a,b):vertices.append([x,y,-.0195,0,0,1,*road_color])
        for bar in jmarks['stop_bars']+jmarks['crosswalks']:
            q=np.asarray(bar)
            for x,y in (q[0],q[1],q[2],q[0],q[2],q[3]):vertices.append([x,y,-.0105,0,0,1,*lane_color])
        for seg in jmarks['branch_centers']:
            q=np.asarray(seg)
            for x,y in (q[0],q[1],q[2],q[0],q[2],q[3]):vertices.append([x,y,-.0105,0,0,1,*yellow_line])
        self.road=self.mesh(np.asarray(vertices,'f4'))
    def frame_many(self,actors,ego_position,ego_yaw,view_position=None,view_yaw=None,lamps=None):
        self.fbo.use();self.ctx.enable(self.gl.DEPTH_TEST);self.fbo.clear(*self.background_rgb,1,depth=1);self.draw(self.ground)
        if view_position is None:
            view_position=np.asarray(ego_position);view_yaw=ego_yaw;ego_relative=np.array([-1.2,0,0]);ego_angle=0.
        else:
            ego_angle=ego_yaw-view_yaw
            ego_relative=rotation(-view_yaw)@(np.asarray(ego_position)-view_position)+rotation(ego_angle)@np.array([-1.2,0,0])
        translation=rotation(-view_yaw)@(-np.array(view_position));self.draw(self.road,translation,-view_yaw)
        self.draw(self.shadow,ego_relative,ego_angle);self.draw(self.models[0],ego_relative,ego_angle)
        state=lamps or {}
        for group,(off,on) in self.ego_lamps.items():
            lit=state.get(group+'_lit') is True if group in ('left','right') else state.get(group) is True
            self.draw(on if lit else off,ego_relative,ego_angle,emissive=lit)
        for actor in actors:
            relative=rotation(-view_yaw)@(np.array(actor['position_W'])-view_position)
            self.draw(self.shadow,relative,actor['yaw_rad']-view_yaw,actor['ghost'])
            self.draw(self.asset_mesh[actor['model']],relative,actor['yaw_rad']-view_yaw,actor['ghost'])
            if actor['model'] in self.actor_rear_mesh:
                dark,red=self.actor_rear_mesh[actor['model']]
                observed=not actor['ghost'] and actor.get('rear_appearance',{}).get('rear_red_visible') is True
                self.draw(red if observed else dark,relative,actor['yaw_rad']-view_yaw,actor['ghost'])
        return Image.fromarray(np.frombuffer(self.fbo.read(components=3,alignment=1),np.uint8).reshape(720,1280,3)[::-1].copy())


def run(package,tracks,base,lens_config,prototype_config,assets,destination,road_masks=None,osm_context=None,preview_times=None,telemetry=None,image_headings=None,contact_adapter=None):
    revision_names=['surrounding.py','prototype_front.py','identity_graph.py','scene_audit.py','rear_reference.py','offline_paths.py','actor_lane_motion.py','pocket_lane.py','rear_view_cue.py','image_median_gate.py','surrounding_lamps.py','unresolved_road.py','actor_rear_axis.py','image_heading_refine.py','clear_handoff.py','lane_state.py','pocket_actor_motion.py','junction_branches.py','road_geometry.py','road_sections.py','road_continuation.py','ego_maneuver.py','road_frame.py','curved_road.py','junctions.py','actor_heading.py','ego_lamps.py']
    revision=lambda:digest(b''.join(Path(__file__).with_name(name).read_bytes() for name in revision_names))
    revision_names.append('static_actor_refine.py')
    revision_names.append('left_median_membership.py')
    revision_names.append('side_contact_recovery.py')
    revision_names.append('collision_constraints.py')
    revision_names.append('side_paint_geometry.py')
    revision_names.append('inside_median_pockets.py')
    revision_names.append('lane_prior_geometry.py')
    revision_names.append('handoff_appearance.py')
    revision_names.append('observed_surface.py')
    revision_names.append('context_surface.py')
    revision_names.append('local_road_extents.py')
    revision_names.append('blindspot_continuity.py')
    revision_names.append('junction_display.py')
    revision_names.append('pillar_bridges.py')
    revision_names.append('camera_junction_roads.py')
    revision_names.append('right_edge_profile.py')
    revision_names.append('source_strip.py')
    revision_names.append('local_solid_corridor.py')
    started_revision=revision()
    package,tracks,base,destination=map(Path,[package,tracks,base,destination])
    if destination.exists():raise FileExistsError(destination)
    assets=dict(assets)
    heavy=Path(assets['ego']).with_name('tesla_fsd_heavy_truck.glb')
    if heavy.is_file():assets.setdefault('heavy_truck',str(heavy))
    pm,tm,bm=[verify(p) for p in (package,tracks,base)]
    lamp_manifest=verify(Path(telemetry)) if telemetry else None
    lamp_trace=LampTrace.from_artifact(telemetry) if telemetry else None
    if lamp_manifest and not any(i['artifact_id']==lamp_manifest['artifact_id'] for i in bm['inputs']):raise ValueError('Lamp telemetry/base mismatch')
    road_mask_manifest=verify(Path(road_masks)) if road_masks else None
    if road_mask_manifest and not any(i['artifact_id']==pm['artifact_id'] for i in road_mask_manifest['inputs']):raise ValueError('Road masks/source mismatch')
    map_manifest=verify(Path(osm_context)) if osm_context else None
    map_records=[]
    if map_manifest:
        base_inputs={i['artifact_id'] for i in bm['inputs']}
        if not any(i['artifact_id'] in base_inputs for i in map_manifest['inputs']):raise ValueError('Map telemetry/base mismatch')
        map_records=json.loads((Path(osm_context)/'context.json').read_text(encoding='utf-8'))
    if not any(i['artifact_id']==pm['artifact_id'] for i in tm['inputs']) or not any(i['artifact_id']==tm['artifact_id'] for i in bm['inputs']):raise ValueError('Source/track/base mismatch')
    cfg=json.loads(Path(prototype_config).read_text());profiles=json.loads(Path(lens_config).read_text())['cameras']
    image_heading_manifest=None;image_heading_predictions=[]
    if image_headings:
        image_heading_manifest=verify(Path(image_headings))
        heading_inputs={r['artifact_id'] for r in image_heading_manifest['inputs']}
        heading_report=json.loads((Path(image_headings)/'report.json').read_text())
        if image_heading_manifest['stage']!='vehicle_image_heading' or not {pm['artifact_id'],tm['artifact_id']}<=heading_inputs:raise ValueError('Image heading/source/track mismatch')
        if heading_report.get('optics_sha256')!=digest(canonical(profiles['front'])):raise ValueError('Image heading optics mismatch')
        image_heading_predictions=[json.loads(l) for l in (Path(image_headings)/'predictions.jsonl').read_text().splitlines()]
    source_coverage=json.loads((package/'camera-coverage.json').read_text()) if (package/'camera-coverage.json').is_file() else {}
    source_strip_missing=defaultdict(int)
    frames=json.loads((package/'frames.json').read_text());by_frame={f['frame_id']:f for f in frames};observations=defaultdict(list);groups=defaultdict(list)
    for row in map(json.loads,(tracks/'observations.jsonl').read_text().splitlines()):observations[row['frame_id']].append(row);groups[row['local_track_id']].append(row)
    frozen=json.loads((base/'trajectory.json').read_text());base_report=json.loads((base/'report.json').read_text());lead_id=base_report['selected_local_track']
    lead_model=base_report.get('actor_model_kind','van')
    if lead_model not in assets or lead_model=='ego':raise ValueError('Unsupported frozen lead asset class')
    # Repair detector fragmentation in image space before rejecting uncertain
    # ground contacts. A truncated fragment must not disappear before its later,
    # fully visible observations can constrain it.
    raw_actors=[]
    for key,rows in groups.items():
        rows=sorted(rows,key=lambda r:r['camera_time_s']);middle=rows[len(rows)//2];f=by_frame[middle['frame_id']]
        if middle['camera_id']=='back' or set(r['class_id'] for r in rows)&{0,1,3,15,16}:continue
        hist=appearance(np.asarray(Image.open(package/f['image']).convert('RGB')),middle['bbox_xyxy'])
        raw_actors.append({'id':key,'camera_id':middle['camera_id'],'frozen_lead':key==lead_id,'start_s':rows[0]['camera_time_s'],'end_s':rows[-1]['camera_time_s'],
            'appearance':hist,'samples':[dict(r,time_s=r['camera_time_s']) for r in rows]})
    raw_merged,raw_edges=consolidate_camera_fragments(raw_actors)
    groups={a['id']:a['samples'] for a in raw_merged};original_ids={a['id']:a['local_ids'] for a in raw_merged}
    times=np.array([r['camera_time_s'] for r in frozen]);ego=np.array([r['ego_position_W'] for r in frozen]);eyaw=np.array([r['ego_yaw_rad'] for r in frozen])
    front=sorted([f for f in frames if f['camera_id']=='front'],key=lambda f:f['camera_time_s']);width=cfg['lane_width_m']
    side_frames=sorted([f for f in frames if f['camera_id']=='left_repeater'],key=lambda f:f['camera_time_s'])
    side_boundary=side_boundary_evidence(package,side_frames,observations,times,ego,eyaw,profiles['left_repeater'],cfg['cameras']['left_repeater'])
    road=road_evidence(package,front,observations,times,ego,eyaw,profiles['front'],width,side_boundary);left=road['left_boundary_y'];actors=[];excluded=[]
    road['heading_evidence']=ground_road_heading(package,frames,observations,times,ego,eyaw,profiles,cfg['cameras'],left)
    right_edge=None
    if road['heading_evidence']['status']!='estimated_from_camera_ground_paint' and left<=width*.65:
        right_edge=right_edge_evidence(package,front,observations,times,ego,eyaw,profiles['front'],cfg['cameras']['front'])
        if right_edge:road['heading_evidence']=right_edge
    road_slope=road['heading_evidence']['slope']
    curve_review={'status':'straight_corridor_retained'};segments=[];side_candidates=[]
    straight_lateral=ego[:,1]-road_slope*ego[:,0]
    if np.ptp(straight_lateral)>cfg.get('curved_road_policy',{}).get('minimum_straight_frame_drift_m',2.):
        segments=paint_segments(package,frames,observations,times,ego,eyaw,profiles['front'],cfg['cameras']['front'],road_masks)
        side_candidates=side_yellow_segments(package,frames,observations,times,ego,eyaw,profiles,cfg['cameras'])
        reference,curve_review=fit_reference(segments,ego,road['boundaries_y'],width,yaws=eyaw)
        curve_review['segments']=len(segments)
        if reference:
            old_lanes=lane_timeline(times,ego,eyaw,road,4.7,2.,centre_x_offset=-1.2)
            candidate_windows=ego_maneuver_windows(old_lanes['events'],times,ego,eyaw,road)
            protected=phase_protection_windows(candidate_windows,times,eyaw,road)
            paint_refuted=[w for w in protected if continuous_paint_excludes_lane_jump(w,times,ego,eyaw,reference,width)]
            protected=[w for w in protected if w not in paint_refuted]
            curve_review['dense_camera_paint_refuted_straight_events']=len(paint_refuted)
            curve_review['old_straight_events']=len(candidate_windows)
            curve_review['heading_aligned_protected_events']=len(protected)
            new_lanes=lane_timeline(times,ego,eyaw,dict(road,reference_line=reference,marking_station_x=None),4.7,2.,centre_x_offset=-1.2)
            if protected and (curve_review.get('supported_fraction',1)<.8 or any(not any(e['direction']==('left' if w['to_lane']<w['from_lane'] else 'right') and abs(e['completion_s']-w['completion_s'])<1.5 for e in new_lanes['events'] if e['completion_s'] is not None) for w in protected)):
                reference=None;curve_review['status']='rejected_reference_phase_alias_at_supported_ego_lane_change'
        if reference:
            road['reference_line']=reference;road['marking_station_x']=None
            road['topology']='camera_supported_piecewise_curved_corridor'
        elif curve_review['status']!='rejected_reference_phase_alias_at_supported_ego_lane_change':
            display_reference=unresolved_route_display(ego,eyaw)
            if display_reference:
                road.update(reference_line=display_reference,marking_station_x=None,
                            lane_geometry_status='unknown_route_display_only',topology='ego_route_display_camera_lanes_unknown',
                            boundaries_y=[width/2,-width/2],left_boundary_y=width/2,
                            left_boundary_role='inferred_extent_not_confirmed_centreline')
                curve_review['unresolved_route_display']='ego_route_only; no_lane_priors_or_paint_claim'
    dimensions={name:np.ptp(model_vertices(path)[:,:3],axis=0) for name,path in assets.items()}
    lengths={name:float(d[0]) for name,d in dimensions.items()}
    contact_model=None;contact_img_cache={};contact_stats={'enabled':bool(contact_adapter),'model_dir':str(contact_adapter) if contact_adapter else None,'applied_front':0,'applied_side':0,'du_abs':[],'dv_abs':[],'cuda':False}
    if contact_adapter is not None:
        from .learned_contact import LearnedContact
        contact_model=LearnedContact(contact_adapter)
    def predict_learned_uv(row,f,camera):
        if contact_model is None or row['support']!='detector_observation' or 'truncated' in row.get('quality_flags',[]):return None
        b=row['bbox_xyxy'];w,h=b[2]-b[0],b[3]-b[1]
        if w<85 or h<55 or min(b[0],b[1],f['width']-b[2],f['height']-b[3])<8:return None
        fid=f['frame_id']
        if fid not in contact_img_cache:
            if len(contact_img_cache)>64:contact_img_cache.clear()
            contact_img_cache[fid]=np.array(Image.open(package/f['image']).convert('RGB'))
        im=contact_img_cache[fid]
        rect=np.array([b[0]-.12*w,b[1]-.12*h,b[2]+.12*w,b[3]+.16*h]).round().astype(int)
        rect[[0,2]]=rect[[0,2]].clip(0,f['width']);rect[[1,3]]=rect[[1,3]].clip(0,f['height'])
        x,y,X,Y=rect
        if X-x<16 or Y-y<16:return None
        out=contact_model.predict(im[y:Y,x:X],dict(camera=camera,bbox_xyxy=b,crop_rect_xyxy=rect.tolist()))
        uv=out['contact_uv']
        if not np.isfinite(uv).all():return None
        contact_stats['du_abs'].append(abs(float(uv[0])-0.5));contact_stats['dv_abs'].append(abs(float(uv[1])-1.0))
        return uv
    for key,rows in groups.items():
        camera=rows[0]['camera_id'];rows=sorted(rows,key=lambda r:r['camera_time_s'])
        if camera=='back':continue
        short=len(rows)<cfg['min_track_samples']
        if short and (camera!='front' or not short_crossing_candidate(rows)):
            excluded.append({'id':key,'reason':'short_track'});continue
        if key==lead_id:
            anchor_rows=[r for r in frozen if r['fitted_actor_position_W'] is not None]
            row_by_frame={r['frame_id']:r for r in rows}
            trusted=[{'time_s':r['camera_time_s'],'position_W':r['fitted_actor_position_W'],'yaw_rad':r['actor_yaw_rad'],'support':r['source_support'],'frame_id':r['frame_id'],
                'bbox_xyxy':row_by_frame[r['frame_id']]['bbox_xyxy'],'image_width':by_frame[r['frame_id']]['width'],'camera_id':camera,'quality':1.,
                'unrefined_position_W':r.get('raw_actor_position_W') or r['fitted_actor_position_W']} for r in anchor_rows]
            candidates=[];known={r['frame_id'] for r in trusted}
            for row in rows:
                if row['frame_id'] in known:continue
                f=by_frame[row['frame_id']];t=f['camera_time_s']
                try:relative,support=project_front(row,profiles[camera],cfg['cameras'][camera],float(dimensions[lead_model][2]),f['width'],f['height'])
                except ValueError:continue
                relative[0]+=lengths[lead_model]/2;yaw=float(np.interp(t,times,eyaw));ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)]);p=ep+rotation(yaw)@relative
                candidates.append(dict(row,time_s=t,position_W=p.tolist(),unrefined_position_W=p.tolist(),projection_support=support,quality=.2,image_width=f['width'],image_height=f['height']))
            completed=augment_frozen_geometry(trusted,candidates)
            if completed:actors.append({'id':key,'camera_id':camera,'model':lead_model,'frozen_lead':True,'samples':completed})
            continue
        box=np.median([r['bbox_xyxy'] for r in rows],axis=0);classes=[r['class_id'] for r in rows]
        if camera in ('left_repeater','right_repeater') and not side_fragment_has_geometry(rows,by_frame[rows[0]['frame_id']]['width'],by_frame[rows[0]['frame_id']]['height']):
            excluded.append({'id':key,'reason':'border_sliver_without_future_full_body_geometry'});continue
        if camera!='front' and box[2]-box[0]<by_frame[rows[0]['frame_id']]['width']*.1 and (box[2]-box[0])/max(1,box[3]-box[1])<.7:
            excluded.append({'id':key,'reason':'narrow_occluded_side_fragment_no_reliable_ground_contact'});continue
        if set(classes)&{0,1,3,15,16}:excluded.append({'id':key,'reason':'non_four_wheeler_prototype_scope'});continue
        f0=by_frame[rows[0]['frame_id']]
        if camera=='front' and unresolved_front_border_fragment(rows,f0['width']):
            excluded.append({'id':key,'reason':'border_sliver_without_clear_front_body_geometry; source_2D_tracking_preserved'});continue
        model,class_support=asset_class(rows,camera,assets,f0['width'],f0['height'])
        samples=[];hist=[]
        for row in rows:
            f=by_frame[row['frame_id']];t=f['camera_time_s'];yaw=float(np.interp(t,times,eyaw));ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)])
            learned_uv=predict_learned_uv(row,f,camera) if camera=='front' else None
            try:
                if camera in ('left_repeater','right_repeater'):
                    relative=project_vehicle_height(row['bbox_xyxy'],profiles[camera],cfg['cameras'][camera],float(dimensions[model][2]));projection_support='apparent_vehicle_height_prior'
                else:
                    relative,projection_support=project_front(row,profiles[camera],cfg['cameras'][camera],float(dimensions[model][2]),f['width'],f['height'],contact_uv=learned_uv)
                    if learned_uv is not None:contact_stats['applied_front']+=1
            except ValueError:continue
            if camera=='front':
                side_body=class_support=='two_clear_side_views_plus_repeated_high_confidence_truck'
                relative[0]+=float(dimensions[model][1] if side_body else lengths[model])/2
            position=ep+rotation(yaw)@relative
            if not -45<relative[0]<(143 if camera=='front' else 70):continue
            quality=.3 if camera!='front' or 'truncated' in row['quality_flags'] else 1.
            if projection_support=='observed_height_prior_after_unresolved_contact':quality=.2
            samples.append({'time_s':float(t),'position_W':position.tolist(),'unrefined_position_W':position.tolist(),'frame_id':f['frame_id'],'support':row['support'],'quality':quality,
                'bbox_xyxy':row['bbox_xyxy'],'image_width':f['width'],'image_height':f['height'],'camera_id':camera,
                'projection_support':projection_support})
            box=row['bbox_xyxy']
            if camera in ('left_repeater','right_repeater') and row['support']=='detector_observation' and 'truncated' not in row.get('quality_flags',[]) and box[2]-box[0]>=60 and box[3]-box[1]>=45 and min(box[0],box[1],f['width']-box[2],f['height']-box[3])>8:
                try:
                    side_uv=predict_learned_uv(row,f,camera)
                    contact=project_contact(box,profiles[camera],cfg['cameras'][camera],contact_uv=side_uv)
                    if side_uv is not None:contact_stats['applied_side']+=1
                    samples[-1]['side_ground_contact_W']=(ep+rotation(yaw)@contact).tolist()
                except ValueError:pass
        if len(samples)<(3 if short else 8):excluded.append({'id':key,'reason':'insufficient_ground_contacts'});continue
        actors.append({'id':key,'camera_id':camera,'model':model,'asset_class_support':class_support,'frozen_lead':False,'samples':samples,'short_crossing_candidate':short})
    left_median_membership=left_median_membership_review(package,frames,observations)
    for actor in actors:
        source_ids=[r['frame_id'] for key in original_ids.get(actor['id'],[actor['id']]) for r in left_median_membership.get(key,{}).get('samples',[])]
        if source_ids:actor['inside_median_frame_ids']=source_ids
    raw_geometry_payload=canonical(actors)
    if road.get('reference_line') and road_masks and road.get('lane_geometry_status')!='unknown_route_display_only':
        visual_bounds,bound_review=supported_boundaries(segments,road['reference_line'],width,actors,cfg['max_lanes'],ego_positions=ego)
        curve_review['boundary_review']=bound_review
        if visual_bounds is None:
            visual_bounds,traffic_review=vehicle_supported_paint_gaps(bound_review,road['reference_line'],width,actors,ego,cfg['max_lanes'])
            curve_review['traffic_supported_boundary_review']=traffic_review
        if visual_bounds:
            road['boundaries_y']=visual_bounds;road['left_boundary_y']=visual_bounds[0]
            road['left_boundary_role']='inferred_extent_not_confirmed_centreline'
            if any(p.get('kind')=='yellow_divider' and abs(p['y']-visual_bounds[0])<.1 for p in bound_review.get('peaks',[])):
                road['left_boundary_role']='camera_observed_yellow_divider_estimated_geometry'
            road['boundary_support']='semantic_camera_paint_with_labelled_outer_vehicle_extent'
    road['junctions']=infer_junctions(times,ego,eyaw,road,map_records,sorted({r['time_s'] for r in segments if r['kind']=='crossbar'}))
    road['junctions'].extend(straight_openings(times,ego,road,segments,map_records))
    sections,section_review=([],{'status':'lane_geometry_unresolved'}) if road.get('lane_geometry_status')=='unknown_route_display_only' else fit_road_sections(road,segments,actors,ego,width,cfg['max_lanes'])
    curve_review['section_review']=section_review
    solid_sections,solid_corridor_review=fit_local_solid_corridor(road,segments,actors,ego,width,cfg['max_lanes']) if not sections else ([],dict(status='existing_sections_retained',changes=[]))
    if solid_sections:sections=solid_sections
    if sections:
        road['lane_sections']=sections;road['lane_count_mode']='section_local';road['topology']='camera_supported_piecewise_lane_sections_with_junction_openings'
    if road.get('reference_line') and road.get('lane_geometry_status')!='unknown_route_display_only':
        reference,continuation_review=support_gaps(road,segments)
        road['reference_line']=reference;curve_review['reference_continuation_review']=continuation_review
    # Rejected experiment: co-visible similar cars are not enough to learn a
    # 13m per-track camera offset. Restore side cars with independent evidence;
    # never fit a camera contact bias by first assuming the identity is true.
    # Road scope is independent of ego pose. A persistent front track in the
    # adjacent corridor supports a lane candidate; it does not prove its width.
    if right_edge and not road.get('reference_line'):
        right=right_edge['right_boundary_y'];required=1
        for actor in actors:
            rs=actor['samples'];duration=rs[-1]['time_s']-rs[0]['time_s']
            if actor['camera_id']!='front' or duration<3:continue
            progress=(rs[-1]['position_W'][0]-rs[0]['position_W'][0])/duration
            if not 2<progress<45:continue
            y=float(np.median([r['position_W'][1]-road_slope*r['position_W'][0] for r in rs]))
            required=max(required,min(cfg['max_lanes'],int(np.floor((y-right)/width))+1))
        road['boundaries_y']=[right+width*i for i in range(required,-1,-1)]
        road['left_boundary_y']=road['boundaries_y'][0];road['boundary_support']='right_camera_paint_edge_plus_persistent_forward_vehicle_corridors'
        road['left_boundary_role']='inferred_extent_not_confirmed_centreline';road['right_edge_evidence']=right_edge
    pocket_evidence=paint_observations(package,frames,observations,times,ego,eyaw,profiles,cfg['cameras'],road_slope,road=road,road_masks=road_masks)
    pocket_evidence['stations']=paint_stations(pocket_evidence['points'])
    unresolved_lanes=road.get('lane_geometry_status')=='unknown_route_display_only'
    pocket_evidence['fit']=dict(pockets=[],occupancy_used=False,status='unknown_lane_geometry') if unresolved_lanes else fit_curved_pockets(pocket_evidence,road) if road.get('reference_line') else fit_pockets(pocket_evidence['stations'],road)
    if road.get('reference_line') and not unresolved_lanes:
        pocket_evidence['rear_view_cues']=actor_cues(actors,package,frames,road)
        pocket_evidence['fit']=vehicle_supported_curved_pockets(pocket_evidence,road,actors,pocket_evidence['rear_view_cues'],pocket_evidence['fit'])
        pocket_evidence['fit'],pocket_evidence['inside_median_approach_review']=inside_median_approach_bands(road,actors,pocket_evidence['fit'],width)
    road['left_pockets']=pocket_evidence['fit']['pockets']
    if road.get('reference_line'):pocket_evidence['curved_pocket_status']='front_only_local_road_frame; no_side_metric_projection'
    road['pocket_support']='front rear-appearance vehicle plus parallel ground separators; auxiliary lane inferred' if pocket_evidence['fit']['occupancy_used'] else 'local paired paint / ground-contact geometry; occupancy rule disabled'
    side_paint=extract_side_paint(package,frames,observations,times,ego,eyaw,profiles,cfg['cameras'],road,project_contact,rotation)
    side_paint_review=apply_side_paint(actors,road,side_paint)
    bounds=road['boundaries_y'];centres=[(a+b)/2 for a,b in zip(bounds,bounds[1:])];scoped=[]
    pixel_median_lines=image_median_lines(package,front,observations)
    pixel_median_review=image_median_review(actors,pixel_median_lines,times,eyaw)
    for actor in actors:
        if not actor.get('frozen_lead') and pixel_median_review.get(actor['id'],{}).get('excluded'):
            excluded.append({'id':actor['id'],'reason':'repeated_raw_image_beyond_yellow_median_no_crossing_evidence'});continue
        if actor.get('short_crossing_candidate') and not crossing_motion(actor,road):
            excluded.append({'id':actor['id'],'reason':'short_track_without_supported_crossing_motion'});continue
        lane,scope_support,y=actor_corridor_lane(actor,bounds,road_slope,road)
        if lane is None:
            recovered=recover_side_contacts(actor,road)
            if recovered:
                actor['samples']=recovered['samples']
                actor['side_contact_recovery']=recovered['evidence']
                lane=recovered['lane'];scope_support='repeated_clear_ground_contacts_inside_existing_road; height_prior_rejected'
        if lane is None:
            actor.update(scope_pending=True,pending_scope_reason=scope_support,pending_scope_median_y=y)
            lane=-2;scope_support='identity_candidate_only_until_linked_to_admitted_actor'
        actor['lane_index']=lane;actor['corridor_scope_support']=scope_support
        if lane==-1 and not road.get('reference_line'):
            local=road_coordinates(road,[r['position_W'] for r in actor['samples']])
            actor['samples']=[r for r,q in zip(actor['samples'],local) if lane_at(road,float(q[0]),float(q[1]))==-1]
        for label,rs in [('start',actor['samples'][:3]),('end',actor['samples'][-3:])]:
            lateral=float(np.median(road_coordinates(road,[r['unrefined_position_W'] for r in rs])[:,1]))
            station=float(np.median(road_coordinates(road,[r['unrefined_position_W'] for r in rs])[:,0]))
            candidate=lane_at(road,station,lateral) if road.get('lane_sections') else lane_from_boundaries(lateral,bounds)
            actor['association_lane_'+label]=candidate if candidate is not None else actor['lane_index']
            actor['association_section_'+label]=(section_at(road,station) or {}).get('id')
        scoped.append(actor)
    lane_count=max([len(centres)]+[s['lane_count'] for s in road.get('lane_sections',[])])
    road.update(lane_count=lane_count,lane_count_support='camera_yellow_and_white_boundaries_with_outer_width_prior')
    if unresolved_lanes:road['lane_count_support']='unknown; nominal_display_band_is_not_a_lane_count'
    stopped=np.flatnonzero(np.linalg.norm(np.gradient(ego,times,axis=0),axis=1)<.2)
    stopped_anchor=[frozen[i]['fitted_actor_position_W'][0] for i in stopped if frozen[i]['fitted_actor_position_W'] is not None]
    if not road.get('reference_line') and len(stopped_anchor)>25 and road['marking_support_frames']>=3:
        road['marking_station_x']=float(np.median(stopped_anchor)+lengths[lead_model]/2+1)
        road['marking_support']='stopped_queue_front_plus_clearance_prior; image marking candidates kept separately'
    actors=[]
    for actor in scoped:
        if actor['lane_index']>=lane_count:excluded.append({'id':actor['id'],'reason':'unsupported_outer_lane'});continue
        rows=actor['samples']
        if not actor['frozen_lead'] and actor['lane_index']==-2:
            rows=actor['samples'];positions=np.array([r['position_W'] for r in rows])
            positions=smooth_positions([r['time_s'] for r in rows],positions,strength=.5)
            headings,heading_support=motion_headings([r['time_s'] for r in rows],positions,float(np.interp(rows[0]['time_s'],times,eyaw)))
            for row,p,heading,observed in zip(rows,positions,headings,heading_support):
                row.update(position_W=p.tolist(),yaw_rad=float(heading),lane_prior='disabled_junction_crossing',lane_index=None,heading_support='two_sided_motion' if observed else 'temporal_heading_or_unresolved_prior')
            actor['lane_motion_evidence']=dict(events=[],support='junction_motion_no_parallel_lane_prior')
        elif not actor['frozen_lead']:
            if actor['lane_index']==-1 and not road.get('reference_line'):
                sample_lanes=np.full(len(rows),-1,dtype=int)
                local=road_coordinates(road,[r['position_W'] for r in rows])
                local_centres=np.array([sum(lane_bounds(road,float(q[0]),-1))/2 for q in local])
                lane_review=dict(events=[],support='paired_local_pocket_geometry',observer='fixed_world_road_frame; ego_lane_not_used')
            else:sample_lanes,local_centres,lane_review=actor_lane_priors(rows,bounds,road_slope,actor['lane_index'],return_review=True,road=road)
            actor['lane_motion_evidence']=lane_review
            w=np.array([r['quality'] if lane_observation_usable(r) else .3 for r in rows])
            positions,prior_applied,measured_lanes=refine_lane_geometry(rows,road,sample_lanes,local_centres,w)
            velocity=np.gradient(positions,np.array([r['time_s'] for r in rows]),axis=0)
            headings=road_coordinates(road,positions)[:,2]
            # A stopped first observation has no velocity-derived heading.
            # Its parallel-corridor prior is the LOCAL road tangent, not the
            # first clip's world X direction before the road turned.
            last=float(headings[0])
            for i,(row,v) in enumerate(zip(rows,velocity)):
                if inside_junction(road,positions[i],3.):
                    if np.linalg.norm(v[:2])>1.5:last=math.atan2(v[1],v[0])
                elif road.get('reference_line'):
                    tangent=headings[i];rv=rotation(-tangent)@v
                    last=tangent+corridor_heading(rv,0.,last-tangent)
                else:last=corridor_heading(v,road_slope,last)
                row.update(position_W=positions[i].tolist(),yaw_rad=last,
                           lane_prior='observed_membership_gated_corridor_prior' if prior_applied[i] else 'disabled_outside_unknown_or_conflicting_membership',
                           lane_index=int(sample_lanes[i]) if prior_applied[i] else measured_lanes[i],
                           lane_prior_applied=bool(prior_applied[i]),lane_measurement_usable=lane_observation_usable(row))
        source_rows=groups[actor['id']]
        usable=[r for r in source_rows if identity_observation_usable(r,by_frame[r['frame_id']]['width'],by_frame[r['frame_id']]['height'])]
        wide=[r for r in source_rows if actor['camera_id'] in ('left_repeater','right_repeater') and r['support']=='detector_observation'
              and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=by_frame[r['frame_id']]['width']*.5
              and r['bbox_xyxy'][3]-r['bbox_xyxy'][1]>=by_frame[r['frame_id']]['height']*.35
              and (r['bbox_xyxy'][2]-r['bbox_xyxy'][0])/max(1,r['bbox_xyxy'][3]-r['bbox_xyxy'][1])>=1.05]
        pool=usable if len(usable)>=3 else source_rows;hist=[]
        for index in np.unique(np.linspace(0,len(pool)-1,3).astype(int)):
            r=pool[index];f=by_frame[r['frame_id']];hist.append(appearance(np.asarray(Image.open(package/f['image']).convert('RGB')),r['bbox_xyxy']))
        clear_views=[]
        for index in np.unique(np.linspace(0,len(usable)-1,min(12,len(usable))).astype(int)) if usable else []:
            r=usable[index];f=by_frame[r['frame_id']]
            clear_views.append(dict(frame_id=r['frame_id'],time_s=r['camera_time_s'],histogram=appearance(np.asarray(Image.open(package/f['image']).convert('RGB')),r['bbox_xyxy']).tolist()))
        actor.update(start_s=rows[0]['time_s'],end_s=rows[-1]['time_s'],appearance=np.mean(hist,axis=0).tolist(),clear_appearance_views=clear_views,
            identity_evidence={'usable_observations':len({r['frame_id'] for r in usable}),
                'wide_side_body_observations':len(wide),'wide_side_body_span_s':wide[-1]['camera_time_s']-wide[0]['camera_time_s'] if wide else 0,
                'observed_samples':len({r['frame_id'] for r in source_rows if r['support']=='detector_observation'}),
                'observed_span_s':source_rows[-1]['camera_time_s']-source_rows[0]['camera_time_s'],
                'required_future_usable_observations':3,'criterion':'clear_future_view_and_clear_or_long_past_observation'})
        actors.append(actor)
    deferred_scope_candidates=[dict(id=a['id'],camera_id=a['camera_id'],reason=a['pending_scope_reason']) for a in actors if a.get('scope_pending')]
    actors,fragment_edges=consolidate_camera_fragments(actors)
    association_view=[dict(a,samples=[dict(r,position_W=r.get('unrefined_position_W',r['position_W'])) for r in a['samples']]) for a in actors]
    edges=association_edges(association_view,cfg.get('identity_policy'));pair_review=[]
    actor_by_id={a['id']:a for a in actors}
    for edge in edges:
        if edge['selected'] and edge.get('future_clear_motion'):
            reconstruct_handoff(actor_by_id[edge['from']],actor_by_id[edge['to']],edge['future_clear_motion'])
    root={a['id']:a['id'] for a in actors}
    def find(key):
        while root[key]!=key:key=root[key]
        return key
    for edge in edges:
        if edge['selected']:root[find(edge['b'])]=find(edge['a'])
    combined=defaultdict(list)
    for actor in actors:combined[find(actor['id'])].append(actor)
    global_actors=[]
    for gid,members in combined.items():
        if all(a.get('scope_pending') for a in members):
            for member in members:
                excluded.append(dict(id=member['id'],reason='outside_same_direction_corridor',
                                     scope_support=member['pending_scope_reason'],identity_candidate_preserved=True))
            continue
        anchor=next((a for a in members if a['frozen_lead']),None)
        if anchor:
            for member in members:
                if member is anchor or member['camera_id'] not in ('left_repeater','right_repeater') or member['lane_index']<0:continue
                rs=member['samples'];raw=[dict(r,position_W=r.get('unrefined_position_W',r['position_W'])) for r in rs]
                proposed,targets=confirmed_handoff_targets(raw,road,member['lane_index'])
                fitted,applied,measured=refine_lane_geometry(raw,road,proposed,targets,[r['quality'] for r in rs],confirmed_front_handoff=True)
                for row,point,used,lane in zip(rs,fitted,applied,measured):
                    if not used:continue
                    row.update(position_W=point.tolist(),lane_index=lane,lane_prior_applied=True,
                               lane_prior='exclusive_front_anchor_handoff_with_one_metre_side_contact_uncertainty')
            result=dict(anchor)
            # Keep trusted anchor samples exact, but do not discard later or
            # earlier camera observations when an ID is joined across cameras.
            result['samples']=merge_anchor_samples(anchor,members)
        else:
            result=dict(next(a for a in members if not a.get('scope_pending')));all_rows=sorted([r for a in members for r in a['samples']],key=lambda r:r['time_s']);bins=defaultdict(list)
            for row in all_rows:bins[round(by_frame[row['frame_id']]['relative_request_s'],3)].append(row)
            result['samples']=[dict(max(rs,key=lambda r:r.get('quality',1)),time_s=float(np.mean([r['time_s'] for r in rs]))) for _,rs in sorted(bins.items())]
        result.update(global_id=gid,local_ids=[original for a in members for k in a.get('local_ids',[a['id']]) for original in original_ids.get(k,[k])],start_s=min(a['start_s'] for a in members),end_s=max(a['end_s'] for a in members),
                      scope_pending=False,identity_supported_scope_members=[a['id'] for a in members if a.get('scope_pending')],
                      lane_motion_evidence_by_camera=[dict(local_id=a['id'],camera_id=a['camera_id'],**a['lane_motion_evidence']) for a in members if 'lane_motion_evidence' in a])
        fill_handoff_gaps(result,cfg['gap_budget_s'])
        calibrated_cameras=[a['camera_id'] for a in members if a.get('side_paint_geometry',{}).get('model_type')=='relative_station_shear']
        if calibrated_cameras:
            result['calibrated_tail_camera_ids']=calibrated_cameras
            extend_calibrated_tail(result,road,float(times[0]+(export_frame_count(base_report,times,20)-1)/20))
        global_actors.append(result)
    edges.extend(raw_edges+fragment_edges)
    all_observations=[r for rs in observations.values() for r in rs]
    rear_review=associate_rear(package,frames,all_observations,global_actors)
    membership={key:a['global_id'] for a in global_actors for key in a['local_ids']};presence=defaultdict(list)
    scope_identity_review=[dict(r,global_id=membership.get(r['id']),
                                status='resolved_by_source_identity' if r['id'] in membership else 'unresolved_scope_excluded_from_world',
                                source_observations_preserved=True) for r in deferred_scope_candidates]
    rear_lamp_samples=observed_rear_lamps(package,frames,all_observations,membership)
    rear_lamp_trace=RearAppearanceTrace(rear_lamp_samples)
    for rs in observations.values():
        for row in rs:
            if row['local_track_id'] in membership and row['support'] in ('detector_observation','flow_observation'):
                presence[membership[row['local_track_id']]].append(row['camera_time_s'])
    for gid,reference_times in reference_presence(rear_review,all_observations).items():presence[gid].extend(reference_times)
    presence={key:np.unique(times) for key,times in presence.items()}
    local_sections,local_extent_review=refine_local_road_extents(road,segments,json.loads(raw_geometry_payload),ego,width,cfg['max_lanes'])
    if local_extent_review.get('changes'):
        road['lane_sections']=local_sections;road['lane_count_mode']='section_local_paint_profile'
    right_sections,right_edge_profile_review=refine_right_edge_profile(road,segments,json.loads(raw_geometry_payload),ego,width)
    if right_sections:
        road['lane_sections']=right_sections;road['lane_count_mode']='section_local_right_edge_profile'
    lane_audit=lane_timeline(times,ego,eyaw,road,float(dimensions['ego'][0]),float(dimensions['ego'][1]),centre_x_offset=-1.2,**cfg.get('lane_state_policy',{}))
    maneuver_policy=cfg.get('ego_maneuver_policy',{})
    maneuver_windows=[] if road.get('reference_line') else ego_maneuver_windows(lane_audit['events'],times,ego,eyaw,road,pre_s=maneuver_policy.get('pre_s',2.5),settle_s=maneuver_policy.get('settle_s',1.5))
    maneuver_review=compensate_actors(global_actors,maneuver_windows,road,max_one_sided_s=maneuver_policy.get('max_one_sided_s',5))
    actor_lane_audit={};rear_axis_review={};image_heading_review={};static_pose_review={}
    for a in global_actors:
        rs=a['samples']
        if len(rs)<2:continue
        original_headings=np.array([r['yaw_rad'] for r in rs])
        continuous=original_headings if a.get('frozen_lead') else resolve_axis_flips([r['time_s'] for r in rs],original_headings)
        for r,before,after in zip(rs,original_headings,continuous):
            if abs(np.arctan2(np.sin(after-before),np.cos(after-before)))>1e-7:
                r.update(yaw_before_axis_disambiguation=float(before),yaw_rad=float(after),heading_axis_support='temporal_body_orientation; velocity_sign_not_body_front')
        rear_axis_review[a['global_id']]=align_actor_rear_axis(a,rear_lamp_samples.get(a['global_id'],[]),times,ego)
        if image_heading_manifest:
            image_heading_review[a['global_id']]=refine_image_heading(a,image_heading_predictions,times,eyaw+np.radians(cfg['cameras']['front']['yaw_deg']),max_gap_s=cfg['gap_budget_s'])
            if image_heading_review[a['global_id']].get('changed_samples'):
                static_pose_review[a['global_id']]=refine_static_actor(a,image_heading_review[a['global_id']],by_frame,profiles,cfg['cameras'],times,ego,eyaw,np.unique(model_vertices(assets[a['model']])[:,:3],axis=0),cfg['gap_budget_s'])
        state=lane_timeline([r['time_s'] for r in rs],[r['position_W'] for r in rs],[r['yaw_rad'] for r in rs],road,float(dimensions[a['model']][0]),float(dimensions[a['model']][1]))
        state.update(local_ids=a['local_ids'],scope='estimated surrounding lane state; source review required',evidence=a['lane_motion_evidence_by_camera'])
        actor_lane_audit[a['global_id']]=state
    road['junctions'],branch_review=junction_branch_hints(road,map_records,times,eyaw,observed_end_station=float(road_coordinates(road,ego[-1])[0]))
    road['junctions'],camera_road_review=camera_crossing_roads(road,global_actors)
    surface=None
    if road_masks and cfg.get('observed_surface_policy',{}).get('enabled',True):
        surface=observed_road_surface(road_masks,frames,observations,times,ego,eyaw,profiles['front'],cfg['cameras']['front'])
        road['display_surface']={k:v for k,v in surface.items() if k!='cells'}
    context_surface=None
    if map_manifest and telemetry and cfg.get('context_surface_policy',{}).get('enabled',True):
        map_report_path=Path(osm_context)/'report.json' if osm_context else None
        map_osm_path=json.loads(map_report_path.read_text(encoding='utf-8')).get('osm_path') if map_report_path and map_report_path.is_file() else None
        context_surface=context_road_surface(map_manifest,telemetry,times,ego,map_records,
                                             [Path.cwd()/'map-cache',Path(__file__).parents[2]/'map-cache'],road=road,osm_path=map_osm_path)
        road['context_surface']=context_surface
    same_dir_lanes=max(1,len(road.get('boundaries_y',[0,0]))-1)
    osm_total=(context_surface or {}).get('main_way_total_lanes')
    two_way_display=(osm_total!=1) and ((road.get('left_boundary_role')!='inferred_extent_not_confirmed_centreline') or bool((context_surface or {}).get('main_way_two_way')))
    if road.get('lane_geometry_status')!='unknown_route_display_only' and (two_way_display or osm_total==1):
        road['opposing_display']=dict(enabled=bool(two_way_display),lanes=max(1,min(3,same_dir_lanes)) if two_way_display else 0,
                                      lane_width_m=float(road.get('lane_width_m_prior',3.5)),
                                      total_lanes=osm_total,
                                      max_same_dir_lanes=(context_surface or {}).get('main_way_lanes_per_dir'),
                                      role='visual_opposing_carriageway_only; no_vehicle_or_lane_scope_change')
    blindspot_policy=dict(cfg.get('blindspot_policy',{}),observation_half_step_s=min(.15,float(np.median(np.diff([f['camera_time_s'] for f in front])))/2)+1e-6)
    blindspot_coasts,blindspot_review=build_blindspot_coasts(global_actors,road,times,ego,eyaw,blindspot_policy)
    renderer=SurroundRenderer(assets,road,cfg.get('render_style'));work=destination.parent/(destination.name+'-work');work.mkdir(parents=True,exist_ok=False)
    (work/'world-checkpoint.json').write_bytes(canonical({'actors':global_actors,'road':road,'edges':edges,'paired_fov_review':pair_review,'excluded':excluded}))
    video=work/'surrounding-world.mp4';font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18)
    sequences={c:sorted([f for f in frames if f['camera_id']==c],key=lambda f:f['camera_time_s']) for c in ['front','back','left_repeater','right_repeater']};cache={};snapshots=[];fps=20
    count=export_frame_count(base_report,times,fps)
    if preview_times is not None:
        preview_times=np.asarray(preview_times,float)
        if not len(preview_times) or len(preview_times)>12 or not np.isfinite(preview_times).all() or np.any(np.diff(preview_times)<=0) or preview_times[0]<times[0] or preview_times[-1]>times[0]+count/fps:raise ValueError('Ordered preview times inside source required (maximum 12)')
        count=len(preview_times)
    sample_half_step=min(.15,float(np.median(np.diff([f['camera_time_s'] for f in front])))/2)+1e-6
    render_counts=[];render_audit=[]
    scene_plan=[];local_scope_display_review=[]
    for j in range(count):
        t=float(preview_times[j]) if preview_times is not None else times[0]+j/fps
        ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)]);yaw=float(np.interp(t,times,eyaw));active=[];visible=visible_ids_at(presence,t)
        maneuver_active=any(w['start_s']<=t<=w['end_s'] for w in maneuver_windows)
        for actor in global_actors:
            # Each 5 Hz observation covers its nearest-time interval. Do
            # not make every actor vanish in the final output frames just
            # because the final source PTS falls after the last grid tick.
            if not actor['start_s']-sample_half_step<=t<=actor['end_s']+sample_half_step:continue
            samples=actor['samples'];ts=np.array([r['time_s'] for r in samples]);nearest=int(np.argmin(abs(ts-t)));before=np.searchsorted(ts,t)-1;after=before+1
            if before>=0 and after<len(ts) and ts[after]-ts[before]>cfg['gap_budget_s']:continue
            pos=trajectory_at(actor,t,restore_maneuver=not maneuver_active);heading=float(np.interp(t,ts,np.unwrap([r['yaw_rad'] if maneuver_active else r.get('yaw_before_ego_maneuver',r['yaw_rad']) for r in samples])))
            source_visible=actor['global_id'] in visible;ghost=not source_visible
            position_support='observed_geometry' if abs(ts[nearest]-t)<=.12 and samples[nearest]['support'] not in ('offline_two_endpoint_gap','offline_future_clear_geometry','offline_bidirectional_clear_geometry','offline_occluded_contact_reconstruction','offline_camera_tail_prediction','offline_camera_edge_prediction') and not samples[nearest].get('static_mesh_weight') and not (maneuver_active and samples[nearest].get('ego_maneuver_correction')) else 'temporal_geometry_reconstruction'
            if unresolved_side_height_position(road,actor,samples[nearest],pos):
                local_scope_display_review.append(dict(time_s=t,global_id=actor['global_id'],reason='side_height_prior_conflicts_with_local_camera_road; source_visible_but_position_unlocalized',observation_modified=False))
                continue
            side_cam=samples[nearest].get('camera_id') in ('left_repeater','right_repeater')
            side_dist=float(np.linalg.norm(pos[:2]-ep[:2]))
            side_cap=float(min(1.5,max(.75,0.075*side_dist))) if not actor.get('frozen_lead') and side_cam else 0.
            active.append({'id':actor['global_id'],'position_W':pos,'yaw_rad':heading,'model':actor['model'],'ghost':ghost,'source_visible':source_visible,'position_support':position_support,
                           'rear_appearance':rear_lamp_trace.at(actor['global_id'],t,ghost),
                           'collision_cap_m':side_cap})
            row=samples[nearest]
            front_uncertain_obs=(row.get('camera_id')=='front' and row.get('projection_support') in ('estimated_ground_contact','far_ground_contact_low_confidence','observed_height_prior_after_unresolved_contact') and row.get('support') in ('detector_observation','flow_observation','offline_two_endpoint_gap') and not lane_observation_usable(row))
            front_reconstructed_gap=(actor.get('camera_id')=='front' and row.get('camera_id')=='reconstructed' and row.get('support')=='offline_two_endpoint_gap')
            front_usable_obs=(row.get('camera_id')=='front' and row.get('projection_support')=='estimated_ground_contact' and row.get('support')=='detector_observation' and lane_observation_usable(row))
            if not actor.get('frozen_lead') and (front_uncertain_obs or front_reconstructed_gap):
                ray=pos[:2]-ep[:2];distance=np.linalg.norm(ray)
                if distance>8:
                    active[-1].update(collision_cap_m=float(min(2.,0.1*distance)),collision_axis_W=(ray/distance).tolist())
            elif not actor.get('frozen_lead') and front_usable_obs:
                ray=pos[:2]-ep[:2];distance=np.linalg.norm(ray)
                if distance>15:
                    active[-1].update(collision_cap_m=float(min(1.,0.04*distance)),collision_axis_W=(ray/distance).tolist())
        for prediction in blindspot_coasts_at(blindspot_coasts,t):
            old=next((a for a in active if a['id']==prediction['id']),None)
            override=prediction.pop('override_unobserved',False)
            allow_visible_tail=prediction.pop('allow_visible_tail',False)
            if old is not None:
                if not override:continue
                if old.get('source_visible') or not old.get('ghost'):
                    # These rows are the old clipped-edge extrapolation, not
                    # new ground measurements. Future evidence may revise the
                    # estimate while retaining actual source visibility.
                    if not allow_visible_tail or old.get('position_support')!='temporal_geometry_reconstruction':continue
                    prediction.update(source_visible=old['source_visible'],ghost=old['ghost'],
                                      position_support='future_supported_camera_edge_reconstruction')
                active.remove(old)
            prediction['rear_appearance']=rear_lamp_trace.at(prediction['id'],t,prediction['ghost'])
            active.append(prediction)
        scene_plan.append(dict(time_s=t,actors=active,ego_position_W=ep,ego_yaw_rad=yaw,visible_ids=visible))
    scene_plan,prediction_display_review=omit_conflicting_predictions(scene_plan,dimensions)
    scene_plan,collision_review=constrain_collisions(scene_plan,road,dimensions,max_iterations=cfg.get('collision_policy',{}).get('max_iterations',16))
    with av.open(str(video),'w',options={'movflags':'+faststart'}) as output:
        stream=output.add_stream('libx264',rate=fps);stream.width=1280;stream.height=1000;stream.pix_fmt='yuv420p';stream.options={'crf':'20','preset':'veryfast'}
        for j,planned in enumerate(scene_plan):
            t=planned['time_s'];ep=np.asarray(planned['ego_position_W']);yaw=planned['ego_yaw_rad'];active=planned['actors'];visible=planned['visible_ids']
            unlocalized=sorted(visible-{a['id'] for a in active})
            vp,vy,vw=camera_view(t,ep,yaw,maneuver_windows,road)
            lamp_state=lamp_trace.at(t) if lamp_trace else None
            scene=renderer.frame_many(active,ep,yaw,vp,vy,lamp_state) if vw>0 else renderer.frame_many(active,ep,yaw,lamps=lamp_state)
            canvas=Image.new('RGB',(1280,1000),(22,27,31));canvas.paste(scene,(0,0));d=ImageDraw.Draw(canvas)
            station=float(road_coordinates(road,ep)[0]);current_lanes=len(boundaries_at(road,station))-1+int(pocket_width(road.get('left_pockets',[]),station)>=1.8)
            lane_description='lane geometry unknown / route display only' if unresolved_lanes else f'estimated {current_lanes}-lane corridor'
            d.text((18,12),f'SURROUNDING WORLD PROTOTYPE | {t:05.2f}s | {len(active)} vehicles | {lane_description}',font=font,fill=renderer.text_color);d.text((18,40),'Frozen ego/lead + front/repeater observations + offline trajectories',font=font,fill=renderer.secondary_text_color)
            ls=lane_audit['records'][int(np.argmin(abs(times-t)))];lane_label='unknown' if ls['lane_id'] is None else 'P-L' if ls['lane_id']==-1 else 'C'+str(ls['lane_id'])
            d.text((18,65),f'Ego corridor {lane_label} | {ls["phase"].replace("_"," ")} | visible, position unresolved: {len(unlocalized)}',font=font,fill=renderer.secondary_text_color)
            if map_records:
                context=min(map_records,key=lambda r:abs(r['time_s']-t))
                candidates=context['junction_candidates'] if abs(context['time_s']-t)<1.5 else []
                if candidates:
                    nearest=min(candidates,key=lambda r:r['distance_m'])
                    d.text((18,91),f'OSM junction/merge candidate ~{nearest["distance_m"]:.0f}m | map reference only',font=font,fill=(190,185,130))
            for k,(camera,seq) in enumerate(sequences.items()):
                availability=source_strip_status(source_coverage,camera,t)
                if availability in ('recording_ended','recording_not_started'):
                    source_strip_missing[camera]+=1
                    tile=Image.new('RGB',(320,240),(15,20,24));td=ImageDraw.Draw(tile)
                    td.text((8,12),camera,fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',16))
                    td.text((18,100),'Camera recording ended' if availability=='recording_ended' else 'Camera not started',fill=(215,220,225),font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',18))
                    canvas.paste(tile,(k*320,720));continue
                f=min(seq,key=lambda f:abs(f['camera_time_s']-t))
                if f['frame_id'] not in cache:cache[f['frame_id']]=Image.open(package/f['image']).convert('RGB').resize((320,240))
                tile=cache[f['frame_id']].copy();td=ImageDraw.Draw(tile);td.rectangle((0,0,320,24),fill=(15,20,24));td.text((4,2),f'{camera} {f["camera_time_s"]:.3f}s',fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',14));canvas.paste(tile,(k*320,720))
            footer='Estimated road outline / camera lane evidence. © OpenStreetMap contributors' if map_manifest else 'Estimated road surface from camera evidence.'
            d.text((10,970),footer,font=font,fill=(200,207,212))
            frame=av.VideoFrame.from_image(canvas);frame.pts=j;frame.time_base=Fraction(1,fps)
            for packet in stream.encode(frame):output.mux(packet)
            render_counts.append(len(active));render_audit.append({'time_s':t,'ego_position_W':ep.tolist(),'ego_yaw_rad':yaw,'ego_lamps':lamp_state,'camera_view':{'mode':'lane_change_road_reference' if vw>0 else 'ego_follow','weight':vw,'position_W':vp.tolist(),'yaw_rad':vy},'visible_unlocalized_ids':unlocalized,'actors':[{'id':a['id'],'position_W':a['position_W'].tolist(),'yaw_rad':a['yaw_rad'],'ghost':a['ghost'],
                'source_visible':a['source_visible'],'position_support':a['position_support'],'rear_appearance':a['rear_appearance'],'model':a['model'],
                **({k:a[k] for k in ('position_before_collision_W','collision_correction_W') if k in a})} for a in active]})
            if (preview_times is not None and j<6) or j in [0,80,160,320,480,640]:snapshots.append(canvas.resize((640,500)))
            if j%200==0:print('Surrounding render',j,'/',count,'actors',len(active),flush=True)
        for packet in stream.encode():output.mux(packet)
    with av.open(str(video)) as check:decoded=sum(1 for _ in check.decode(video=0));duration=check.duration/av.time_base
    if decoded!=count:raise ValueError('Incomplete video export')
    sheet=Image.new('RGB',(1920,1000),'#eee')
    for j,image in enumerate(snapshots):sheet.paste(image,((j%3)*640,(j//3)*500))
    buf=io.BytesIO();sheet.save(buf,format='JPEG',quality=95)
    report={'scope':'Estimated same-direction surrounding world prototype; incomplete junction topology and no measured rig claim',
        'scope_identity_review':scope_identity_review,
        'render_style':{'ego':'original_asset_material','surrounding':'matte_clay','ghost':'lightened_screen_door_clay'},
        'global_actors':len(global_actors),'local_admitted':len(actors),'cross_camera_edges_selected':sum(e['selected'] for e in edges if e['kind']!='same_camera_box_continuity'),
        'rear_geometry':'DISABLED','road':road,'frames':count,'duration_s':float(duration),'all_frames_decoded':True,
        'rear_identity_reference':{'supported_intervals':len(rear_review['intervals']),'world_id_merge':'DISABLED','scope':'matched_intervals_only'},
        'right_projection':'apparent_vehicle_height_prior_not_bbox_bottom_ground_contact','coobserved_offset_fusion':'DISABLED_rejected_identity_ambiguity',
        'left_projection':'apparent_vehicle_height_prior_not_bbox_bottom_ground_contact',
        'render_actor_count_range':[min(render_counts),max(render_counts)],'ego_and_lead_source':'frozen base ego and observed anchor arrays unchanged',
        'source_duration_s':base_report.get('source_duration_s'),'export_full_source':'source_duration_s' in base_report,
        'clock_status':'assumed_relative_container_alignment_not_sensor_sync','excluded_tracks':excluded,'metric_accuracy':'NOT_EVALUATED'}
    report['learned_contact_adapter']={'enabled':contact_stats['enabled'],'model_dir':contact_stats['model_dir'],
        'applied_front':contact_stats['applied_front'],'applied_side':contact_stats['applied_side'],
        'mean_abs_du':float(np.mean(contact_stats['du_abs'])) if contact_stats['du_abs'] else 0.,
        'mean_abs_dv':float(np.mean(contact_stats['dv_abs'])) if contact_stats['dv_abs'] else 0.,'cuda':False}
    report['collision_constraints']={k:v for k,v in collision_review.items() if k!='changed_samples'}
    report['prediction_display']={k:v for k,v in prediction_display_review.items() if k!='omitted_samples'}
    report['vehicle_dimensions_m']={k:v.tolist() for k,v in dimensions.items()}
    report['ego_lane_events']=lane_audit['events'];report['ego_lane_pose_modified']=False
    report['image_body_heading']={'enabled':bool(image_heading_manifest),'XYZ_or_identity_modified':False,'role':'bounded_offline_image_heading_for_free_car_proxies_only',
                                  'changed_actors':[k for k,v in image_heading_review.items() if v.get('changed_samples')]}
    report['conditional_static_mesh_pose']={'accepted_actors':[k for k,v in static_pose_review.items() if v.get('accepted')],
                                           'ego_rig_road_or_identity_modified':False,'role':'conditional_parked_actor_pose_from_multicamera_source_reprojection; lower_body_may_be_occluded'}
    report['render_is_preview']=preview_times is not None
    if preview_times is not None:report['export_full_source']=False;report['preview_times_s']=preview_times.tolist()
    report['osm_context']={'enabled':bool(map_manifest),'gps_pose_registration':False,'map_lane_count_or_width_used':False,
                           'role':'topology_confirmation_and_camera_anchored_orientation_hints',
                           'branch_display_arms':sum(sum(a.get('source','').startswith('osm_') for a in j.get('branch_display_hints',[])) for j in road['junctions']),
                           'camera_crossing_display_arms':len(camera_road_review),
                           'branch_hints_modify_actor_or_ego_geometry':False}
    report['ego_lamps']={'telemetry_connected':bool(lamp_trace),'visual_asset':str(renderer.ego_visual),'mesh_groups':sorted(renderer.ego_lamps),'flash_phase':'illustrative_not_measured','brake_source':'SEI brake_applied control proxy; not optically verified bulb','other_vehicles':'observed rear red appearance only; brake and indicators unknown; no emissive or invented phase'}
    coverage=observation_coverage(all_observations,global_actors,render_audit,excluded,original_ids,rear_review)
    report['observation_world_coverage']=coverage['camera_summary']
    payloads={'surrounding-world.mp4':video.read_bytes(),'report.json':canonical(report),'actors.json':canonical(global_actors),
        'association-review.json':canonical(edges),'paired-fov-review.json':canonical(pair_review),'render-states.jsonl':b''.join(canonical(r)+b'\n' for r in render_audit),'proof/contact.jpg':buf.getvalue()}
    payloads['collision-review.json']=canonical(collision_review)
    payloads['local-road-extent-review.json']=canonical(local_extent_review)
    payloads['right-edge-profile-review.json']=canonical(right_edge_profile_review)
    payloads['local-solid-corridor-review.json']=canonical(solid_corridor_review)
    payloads['local-scope-display-review.json']=canonical(dict(unlocalized=local_scope_display_review,source_observations_preserved=True,poses_snapped_to_lane=False))
    payloads['camera-strip-coverage.json']=canonical(dict(coverage=source_coverage,unavailable_render_frames=dict(source_strip_missing),recording_coverage_known=bool(source_coverage),stale_tail_presented_as_current=False if source_coverage else None))
    payloads['blindspot-inference-review.json']=canonical(dict(predictions=blindspot_review,observations_modified=False,identity_merged=False,scope='bounded_pillar_region_display_only'))
    payloads['prediction-display-review.json']=canonical(prediction_display_review)
    payloads['side-paint-geometry.json']=canonical(dict(models=side_paint,actors=side_paint_review))
    payloads['ego-lane-state.json']=canonical(lane_audit)
    payloads['actor-lane-state.json']=canonical(actor_lane_audit)
    payloads['observation-world-audit.json']=canonical(coverage)
    payloads['rear-identity-reference.json']=canonical(rear_review)
    payloads['left-pocket-evidence.json']=canonical(pocket_evidence)
    payloads['curved-road-review.json']=canonical(curve_review)
    payloads['branch-hints-review.json']=canonical(branch_review)
    payloads['camera-crossing-road-review.json']=canonical(dict(changes=camera_road_review,ego_actor_or_lane_modified=False))
    payloads['image-median-review.json']=canonical(pixel_median_review)
    payloads['image-median-lines.json']=canonical(pixel_median_lines)
    payloads['surrounding-rear-appearance.json']=canonical(rear_lamp_samples)
    payloads['rear-axis-review.json']=canonical(rear_axis_review)
    payloads['image-body-heading-review.json']=canonical(image_heading_review)
    payloads['static-mesh-pose-review.json']=canonical(static_pose_review)
    payloads['left-median-membership.json']=canonical(left_median_membership)
    payloads['road-paint-segments.json']=canonical(segments)
    payloads['side-ground-candidates.json']=canonical(dict(status='diagnostic_only_unaccepted_side_rig; not_fused_into_road',segments=side_candidates))
    if surface is not None:payloads['observed-road-surface.json']=canonical(surface)
    if context_surface is not None:payloads['context-road-surface.json']=canonical(context_surface)
    if road.get('reference_line') and road_masks:payloads['pre-road-geometry.json']=raw_geometry_payload
    if map_manifest:
        payloads['osm-context.json']=canonical(map_records)
        payloads['osm-attribution.txt']='© OpenStreetMap contributors. ODbL. https://www.openstreetmap.org/copyright\n'.encode('utf-8')
    payloads['ego-maneuver-compensation.json']=canonical(dict(windows=maneuver_windows,actors=maneuver_review,ego_trajectory_modified=False,scope='bounded ego lane change only; no normal driving correction'))
    if revision()!=started_revision:raise ValueError('Render code changed during execution; preserve work output and rerun with frozen code')
    return publish(destination,stage='surrounding_world_prototype',payloads=payloads,
        inputs=[{'role':role,'artifact_id':m['artifact_id']} for role,m in zip(['source','tracks','frozen_ego_lead'],[pm,tm,bm])]+([{'role':'camera_road_masks','artifact_id':road_mask_manifest['artifact_id']}] if road_mask_manifest else [])+([{'role':'osm_topology_reference','artifact_id':map_manifest['artifact_id']}] if map_manifest else [])+([{'role':'ego_lamp_telemetry','artifact_id':lamp_manifest['artifact_id']}] if lamp_manifest else [])+([{'role':'front_image_body_heading','artifact_id':image_heading_manifest['artifact_id']}] if image_heading_manifest else []),
        config={'prototype':cfg,'optics':profiles,'road':road,'assets':{k:str(v) for k,v in assets.items()},'ego_visual_asset':str(renderer.ego_visual)},model_hashes=[file_hash(Path(p)) for p in assets.values()]+[file_hash(renderer.ego_visual)],
        code_revision=started_revision,runtime={'av':av.__version__,'render_device':renderer.ctx.info['GL_RENDERER'],'cuda':False})
