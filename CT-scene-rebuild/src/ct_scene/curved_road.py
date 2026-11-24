"""Offline camera-paint reference line, with independent support per station.

The ego route only indexes a narrow search strip. The normal offset must be
measured from repeated paint observations; no actor or OSM lane count is used.
"""
from collections import defaultdict
from pathlib import Path
import cv2
import numpy as np
from .affine_probe import camera_matrix
from .road_frame import RoadFrame
from .prototype_front import rotation


def route_index(ego, spacing=5., extension=60.,yaws=None):
    p=np.asarray(ego,float)[:,:2];chosen=[p[0]]
    for q in p[1:]:
        if np.linalg.norm(q-chosen[-1])>=spacing:chosen.append(q)
    if len(chosen)<3:raise ValueError('Insufficient moving route for curve indexing')
    p=np.asarray(chosen);before=(p[1]-p[0]);before/=np.linalg.norm(before)
    after=p[-1]-p[-2];after/=np.linalg.norm(after)
    if yaws is not None:
        before=np.array([np.cos(yaws[0]),np.sin(yaws[0])]);after=np.array([np.cos(yaws[-1]),np.sin(yaws[-1])])
    return RoadFrame(np.vstack([p[0]-extension*before,p,p[-1]+extension*after]),-extension)


def paint_segments(package,frames,observations,times,ego,yaws,profile,mount,road_masks=None,*,yellow_road_mask=False,lane_road_intersection=True):
    K,D=camera_matrix(profile);result=[]
    masks_by_frame={}
    if road_masks:
        import json
        masks_by_frame={r['frame_id']:r for r in json.loads((Path(road_masks)/'frames.json').read_text())}
    for f in sorted((f for f in frames if f['camera_id']=='front'),key=lambda f:f['camera_time_s']):
        image=cv2.imread(str(Path(package)/f['image']));h,w=image.shape[:2]
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);allowed=np.zeros((h,w),np.uint8)
        allowed[int(h*.57):int(h*.94),int(w*.04):int(w*.96)]=255
        road_allowed=allowed.copy()
        if road_masks:
            r=masks_by_frame.get(f['frame_id'])
            if r is None:continue
            semantic=cv2.imread(str(Path(road_masks)/r['lane']),cv2.IMREAD_GRAYSCALE)
            if semantic is None or semantic.shape!=(h,w):raise ValueError('Raw-space lane-mask shape mismatch')
            allowed=cv2.bitwise_and(allowed,cv2.dilate(semantic,np.ones((9,9),np.uint8)))
            rm=cv2.imread(str(Path(road_masks)/r['road']),cv2.IMREAD_GRAYSCALE)
            road_allowed=cv2.bitwise_and(road_allowed,cv2.dilate(rm,np.ones((15,15),np.uint8)))
            if lane_road_intersection:allowed=cv2.bitwise_and(allowed,road_allowed)
        for row in observations.get(f['frame_id'],[]):
            x,y,X,Y=np.rint(row['bbox_xyxy']).astype(int)
            allowed[max(0,y-8):min(h,Y+8),max(0,x-8):min(w,X+8)]=0
            road_allowed[max(0,y-8):min(h,Y+8),max(0,x-8):min(w,X+8)]=0
        contrast=cv2.subtract(hsv[:,:,2],cv2.GaussianBlur(hsv[:,:,2],(0,0),5))
        masks={'white':(hsv[:,:,1]<65)&(hsv[:,:,2]>110)&(contrast>12),
               'yellow':(hsv[:,:,0]>=18)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>75)&(hsv[:,:,2]>70)}
        masks['crossbar']=masks['white']
        white_support=cv2.dilate(masks['white'].astype('uint8'),np.ones((3,3),np.uint8))
        t=f['camera_time_s'];yaw=np.interp(t,times,yaws)
        ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)])
        for kind,mask in masks.items():
            region=road_allowed if kind=='crossbar' or (kind=='yellow' and yellow_road_mask) else allowed
            binary=(mask&(region>0)).astype('uint8')*255
            if kind=='crossbar':
                # Zebra crossings consist of disconnected longitudinal bars,
                # not one continuous transverse Hough line. Join nearby bars
                # into a transverse band, retaining the drivable/actor mask.
                kernel=np.ones((3,max(15,int(w*.05)//2*2+1)),np.uint8)
                binary=cv2.bitwise_and(cv2.morphologyEx(binary,cv2.MORPH_CLOSE,kernel),region)
            lines=cv2.HoughLinesP(binary,1,np.pi/180,35,minLineLength=50,maxLineGap=12)
            if lines is None:continue
            for x,y,X,Y in lines.reshape(-1,4):
                pixels=np.array([[[x,y]],[[X,Y]]],float)
                rays=cv2.undistortPoints(pixels,K,D).reshape(2,2)
                if np.any(rays[:,1]<=.04):continue
                depth=mount['height_m']/rays[:,1]
                if np.any(depth<3) or np.any(depth>30):continue
                local=np.c_[depth,-depth*rays[:,0],np.zeros(2)]
                delta=local[1]-local[0];length=np.linalg.norm(delta[:2])
                if not 1.2<length<22:continue
                if kind=='crossbar':
                    if length<3 or abs(delta[0])>abs(delta[1])*.3:continue
                elif abs(delta[0])<abs(delta[1])*.8:continue
                world=local@rotation(yaw).T+ep
                sx=np.clip(np.rint(np.linspace(x,X,60)).astype(int),0,w-1);sy=np.clip(np.rint(np.linspace(y,Y,60)).astype(int),0,h-1)
                continuity=float(np.mean(white_support[sy,sx]>0)) if kind=='white' else None
                result.append(dict(frame_id=f['frame_id'],time_s=t,kind=kind,points_W_xy=world[:,:2].tolist(),pixels=[int(x),int(y),int(X),int(Y)],
                                   ground_length_m=float(length),color_continuity=continuity))
    return result


def side_yellow_segments(package,frames,observations,times,ego,yaws,profiles,mounts):
    """Side ground paint supplies curb/divider bounds, never rear geometry."""
    result=[]
    for camera in ('left_repeater','right_repeater'):
        K,D=camera_matrix(profiles[camera]);mount=mounts[camera];rig=rotation(np.radians(mount['yaw_deg']))
        for f in (f for f in frames if f['camera_id']==camera):
            im=cv2.imread(str(Path(package)/f['image']));h,w=im.shape[:2];hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
            mask=((hsv[:,:,0]>=12)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>75)&(hsv[:,:,2]>65)).astype('uint8')*255
            mask[:int(h*.5)]=0;mask[int(h*.97):]=0
            for row in observations.get(f['frame_id'],[]):
                x,y,X,Y=np.rint(row['bbox_xyxy']).astype(int);mask[max(0,y-6):min(h,Y+6),max(0,x-6):min(w,X+6)]=0
            lines=cv2.HoughLinesP(mask,1,np.pi/180,30,minLineLength=60,maxLineGap=18)
            if lines is None:continue
            t=f['camera_time_s'];yaw=np.interp(t,times,yaws);ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)])
            for x,y,X,Y in lines.reshape(-1,4):
                rays=cv2.undistortPoints(np.array([[[x,y]],[[X,Y]]],float),K,D).reshape(2,2)
                if np.any(rays[:,1]<=.04):continue
                depth=mount['height_m']/rays[:,1]
                if np.any(depth<.8) or np.any(depth>25):continue
                local=np.c_[depth,-depth*rays[:,0],np.zeros(2)]@rig.T+mount['offset_W_ego_m']
                delta=local[1]-local[0];length=np.linalg.norm(delta[:2])
                if not 1.2<length<25 or abs(delta[0])<abs(delta[1])*.6:continue
                world=local@rotation(yaw).T+ep
                result.append(dict(frame_id=f['frame_id'],time_s=t,kind='yellow',camera_id=camera,points_W_xy=world[:,:2].tolist(),
                                   pixels=[int(x),int(y),int(X),int(Y)],ground_length_m=float(length),support='estimated_side_camera_ground_paint'))
    return result


def fit_reference(segments,ego,boundaries,width=3.5,step=10.,max_gap=40.,yaws=None):
    helper=route_index(ego,yaws=yaws);bins=defaultdict(list)
    # A camera-visible repeated white stripe supplies phase; yellow boundaries
    # are reserved for semantics, rather than confusing a divider with a lane.
    positive=[b for b in boundaries[1:-1] if b>0]
    anchor=min(positive) if positive else float(boundaries[0])
    for row in segments:
        if row['kind']!='white':continue
        p=np.array(row['points_W_xy']);delta=p[1]-p[0];length=np.linalg.norm(delta)
        for u in np.linspace(0,1,5):
            point=p[0]*(1-u)+p[1]*u;s,d,h=helper.project(point)
            angle=np.arctan2(delta[1],delta[0])-h
            if abs(np.sin(angle))>.4 or abs(d)>width*4:continue
            bins[round(s/step)*step].append((float(d),row['frame_id'],min(length,5.)))
    stations=[]
    for s,rows in sorted(bins.items()):
        if len({r[1] for r in rows})<3:continue
        # Equal total vote per source frame prevents a bright fragmented line
        # in one image from outweighing independent observations.
        by_frame=defaultdict(list)
        for d,f,w in rows:by_frame[f].append(d)
        phases=[]
        for ds in by_frame.values():
            z=np.mean(np.exp(2j*np.pi*(np.array(ds)-anchor)/width))
            if abs(z)>.65:phases.append(z/abs(z))
        if len(phases)<3:continue
        z=np.mean(phases)
        if abs(z)<.82:continue
        stations.append(dict(s=float(s),phase=float(np.angle(z)),concentration=float(abs(z)),frames=len(phases)))
    if len(stations)<6:return None,dict(status='insufficient_repeated_paint',stations=stations)
    s=np.array([r['s'] for r in stations]);offset=np.unwrap([r['phase'] for r in stations])*width/(2*np.pi)
    # Fix only the integer gauge from the first camera-supported station.
    offset-=round(offset[0]/width)*width
    if s[-1]-s[0]<50:return None,dict(status='insufficient_station_span',stations=stations)
    # Reject phase aliases instead of inventing a lane change in the road.
    if np.any(abs(np.diff(offset)/np.diff(s))>.18):return None,dict(status='ambiguous_paint_phase',stations=stations)
    # Preserve the indexed route's bends between paint stations. Joining two
    # stations by a chord could cut across a whole turn and displace the road.
    dense_s=np.unique(np.r_[min(0.,s[0]),s,np.arange(min(0.,s[0]),s[-1],3.),helper.stations[(helper.stations>min(0.,s[0]))&(helper.stations<s[-1])]])
    dense_offsets=np.interp(dense_s,s,offset);xy=helper.world(dense_s,dense_offsets)
    reference_frame=RoadFrame(xy,float(dense_s[0]));rs=reference_frame.stations
    gaps=[[float(np.interp(s[i],dense_s,rs)),float(np.interp(s[i+1],dense_s,rs))] for i in range(len(s)-1) if s[i+1]-s[i]>max_gap]
    coverage=1-sum(b-a for a,b in gaps)/(rs[-1]-rs[0])
    partial=coverage<.65
    line=dict(points_W_xy=xy.tolist(),station_origin_m=float(dense_s[0]),support='repeated_camera_ground_white_paint',
              estimated_rig=True,station_support=stations,normal_offsets_m=offset.tolist(),max_supported_gap_m=max_gap,
              unsupported_intervals_m=gaps,supported_extent_m=[float(rs[0]-40),float(rs[-1]+40)],endpoint_extrapolation_budget_m=40.,
              camera_coverage='partial' if partial else 'supported',gap_geometry='own_route_index_with_interpolated_camera_offset; not_observed_lane')
    return line,dict(status='partial_camera_reference_unknown_gaps_retained' if partial else 'estimated_camera_reference',stations=stations,span_m=float(s[-1]-s[0]),
                     offset_range_m=float(np.ptp(offset)),supported_fraction=float(coverage),unsupported_intervals_m=gaps,
                     ego_modified=False,actors_used=False,osm_used=False)


def observed_paint_phase(projected,width):
    """Measure stripe lattice translation, never assume reference is lane-centred.

    Width stays an explicit prior. Cap votes by spatial bin so a long stop on
    one painted word cannot outvote repeated stripes along the moving route.
    """
    white=[(s,d,r['frame_id']) for s,d,r in projected if r['kind']=='white']
    if not white:return width/2,dict(status='unobserved_nominal_phase_prior')
    candidates=[]
    for phase in np.arange(0,width,width/140):
        bins=defaultdict(set);support=[]
        for s,d,fid in white:
            residual=(d-phase+width/2)%width-width/2
            if abs(residual)<=.35:bins[int(np.floor(s/20))].add(fid);support.append((residual,fid))
        score=sum(min(3,len(v)) for v in bins.values())
        candidates.append((score,len({f for _,f in support}),phase,bins,support))
    _,count,phase,bins,support=max(candidates,key=lambda p:(p[0],p[1]))
    if count<5 or sum(len(v)>=2 for v in bins.values())<2:
        return width/2,dict(status='insufficient_spatially_repeated_phase; nominal_prior',source_frames=count)
    # One median per raw frame avoids repeated Hough fragments weighting a
    # frame more strongly than another frame of the same road marking.
    residuals=defaultdict(list)
    for d,fid in support:residuals[fid].append(d)
    phase=float((phase+np.median([np.median(v) for v in residuals.values()]))%width)
    return phase,dict(status='camera_paint_phase',phase_m=phase,source_frames=count,
                      spatial_bins_with_repeated_frames=sum(len(v)>=2 for v in bins.values()),
                      width_m_prior=width,ego_used=False,actors_used=False)


def supported_boundaries(segments,reference,width,actors,max_lanes=4,ego_positions=None,*,inherited_divider=None):
    """Paint bounds first; persistent clear cars may support one outer lane.

    Return evidence explicitly: observed paint and vehicle-supported extent are
    different. Actor-derived extent is never an independent accuracy test.
    """
    frame=RoadFrame(reference['points_W_xy'],reference.get('station_origin_m',0.))
    bins=defaultdict(list);yellow_bins=defaultdict(list);solid_bins=defaultdict(list);all_frames={r['frame_id'] for r in segments}
    projected=[]
    for r in segments:
        if r['kind'] not in ('white','yellow'):continue
        s,d,h=frame.project(np.mean(r['points_W_xy'],axis=0))
        delta=np.diff(r['points_W_xy'],axis=0)[0]
        if abs(np.sin(np.arctan2(delta[1],delta[0])-h))>.4:continue
        projected.append((float(s),float(d),r))
    phase,phase_review=observed_paint_phase(projected,width)
    # A stripe-derived reference has an arbitrary lateral gauge; zero is not
    # necessarily the observer's lane. Classify yellow paint against the fixed
    # own route at the same station, never against another vehicle's position.
    own_local=frame.project(ego_positions) if ego_positions is not None else None
    if own_local is not None:
        own_local=own_local[np.argsort(own_local[:,0])]
    yellow_sides=defaultdict(lambda:defaultdict(list))
    for s,d,r in projected:
        k=round((d-phase)/width)
        if abs(d-(phase+k*width))<.7:
            (yellow_bins if r['kind']=='yellow' else bins)[k].append((float(d),r['frame_id']))
            if r['kind']=='yellow':
                own_d=float(np.interp(s,own_local[:,0],own_local[:,1])) if own_local is not None else 0.
                yellow_sides[k][r['frame_id']].append(d-own_d)
            if r['kind']=='white' and r.get('ground_length_m',0)>=7 and (r.get('color_continuity') or 0)>=.8:
                solid_bins[k].append((float(d),r['frame_id']))
    # A long stop behind a bus cannot raise the required support for a stripe
    # seen clearly earlier on the moving approach. Count repeated source
    # observations, not a percentage of unrelated/occluded clip frames.
    minimum=5
    yellow=[]
    for k,rows in yellow_bins.items():
        frames={f for _,f in rows};d=float(np.median([d for d,_ in rows]))
        relative=[np.median(v) for v in yellow_sides[k].values()]
        if len(frames)>=minimum and np.mean(np.asarray(relative)>.4)>=.85:yellow.append(dict(index=k,y=d,frames=len(frames),kind='yellow_divider'))
    # Repeated yellow ground paint can bound a one-lane carriageway. Requiring
    # TWO white lines silently forced a narrow yellow/white road back to the
    # old three-lane template. Keep the nearest supported left divider; paint
    # on the opposite carriageway never adds our lanes.
    divider=min(yellow,key=lambda p:p['y']) if yellow else None
    if divider is None and inherited_divider is not None:
        continuation=[]
        for k,rows in bins.items():
            frames={fid for _,fid in rows};d=float(np.median([d for d,_ in rows]))
            if len(frames)>=minimum and abs(d-inherited_divider)<.8:
                continuation.append(dict(index=k,y=d,frames=len(frames),kind='inherited_yellow_divider_with_local_separator',
                                         inherited_lateral_m=float(inherited_divider)))
        if continuation:divider=min(continuation,key=lambda p:abs(p['y']-inherited_divider))
    peaks=[]
    for k,rows in sorted(bins.items(),reverse=True):
        frames={f for _,f in rows}
        if divider and k>=divider['index']:continue
        if len(frames)>=minimum:peaks.append(dict(index=k,y=float(np.median([d for d,_ in rows])),frames=len(frames),kind='white'))
    if divider:peaks.insert(0,divider)
    right_yellow=[]
    for k,rows in yellow_bins.items():
        frames={f for _,f in rows};d=float(np.median([d for d,_ in rows]))
        relative=[np.median(v) for v in yellow_sides[k].values()]
        if len(frames)>=minimum and np.mean(np.asarray(relative)<-.4)>=.85:right_yellow.append(dict(index=k,y=d,frames=len(frames),kind='yellow_curb'))
    if right_yellow:
        curb=max(right_yellow,key=lambda p:p['y'])
        peaks=[p for p in peaks if p['index']>curb['index']]+[curb]
    curb=None
    if ego_positions is not None:
        edge_bounds,edge_review=continuous_edge_bounds(solid_bins,frame,width,actors,ego_positions,max_lanes,paint_peaks=peaks)
        if edge_bounds is not None:return edge_bounds,dict(edge_review,peaks=peaks,paint_phase=phase_review)
        # A solid inner separator on the left may have a supported adjacent
        # lane beyond it. That does not invalidate an independently observed
        # right curb: do not count sidewalk/shoulder stripes past that curb.
        curb=edge_review.get('right_edge_candidate')
        if curb:
            peaks=[p for p in peaks if p['index']>curb['index']]+[dict(curb,kind='continuous_right_edge_hypothesis')]
    if len(peaks)<2:return None,dict(status='insufficient_semantic_paint_bounds',peaks=peaks,paint_phase=phase_review)
    if any(a['index']-b['index']!=1 for a,b in zip(peaks,peaks[1:])):return None,dict(status='non_contiguous_paint_bounds',peaks=peaks,paint_phase=phase_review)
    left=peaks[0]['y'];right=peaks[-1]['y'];count=len(peaks)-1;extra=[];side_candidates=[]
    for a in actors:
        side=a.get('camera_id')=='right_repeater'
        rows=a['samples'];clear=[r for r in rows if r.get('camera_id') in ('front','right_repeater') and r.get('support')=='detector_observation'
                                and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>80 and min(r['bbox_xyxy'][0],r['image_width']-r['bbox_xyxy'][2])>6]
        if len(clear)<(10 if side else 15) or clear[-1]['time_s']-clear[0]['time_s']<(2 if side else 3):continue
        local=frame.project([r.get('unrefined_position_W',r['position_W']) for r in clear]);d=float(np.median(local[:,1]))
        if side and np.quantile(abs(local[:,1]-d),.8)>1.2:continue
        progress=(local[-1,0]-local[0,0])/(clear[-1]['time_s']-clear[0]['time_s'])
        if not (.5 if side else 2)<progress<55:continue
        if not right_yellow and curb is None and right-width+.3<d<right-.5 and count<max_lanes:
            proof=dict(local_id=a['id'],median_lateral_m=d,clear_frames=len(clear),side='right',role='outer_vehicle_corridor_not_independent_road_truth')
            if side:side_candidates.append(dict(proof,frames={r['frame_id']:r['time_s'] for r in clear}))
            else:extra.append(proof)
        if not side and divider is None and left+.5<d<left+width-.3 and count<max_lanes:
            extra.append(dict(local_id=a['id'],median_lateral_m=d,clear_frames=len(clear),side='left',role='outer_vehicle_corridor_not_independent_road_truth'))
    # A lone side-camera depth prior can put a correctly travelling car beyond
    # an edge. Require two distinct, simultaneously visible stable tracks before
    # using side geometry to add a band; keep front-camera rules unchanged.
    for i,a in enumerate(side_candidates):
        for b in side_candidates[i+1:]:
            common=set(a['frames'])&set(b['frames'])
            if len(common)<3 or np.ptp([a['frames'][f] for f in common])<.4:continue
            for candidate in (a,b):
                if not any(p['local_id']==candidate['local_id'] for p in extra):extra.append({k:v for k,v in candidate.items() if k!='frames'})
    bounds=[p['y'] for p in peaks]
    if any(p['side']=='right' for p in extra):bounds.append(right-width)
    if any(p['side']=='left' for p in extra):bounds.insert(0,left+width)
    if len(bounds)-1>max_lanes:return None,dict(status='exceeds_configured_scope',peaks=peaks)
    if ego_positions is not None:
        local=frame.project(ego_positions)
        fraction=float(np.mean((local[:,1]>=bounds[-1]-.25)&(local[:,1]<=bounds[0]+.25)))
        if fraction<.85:
            return None,dict(status='rejected_bounds_excluding_own_observed_route',peaks=peaks,ego_inside_fraction=fraction)
    return bounds,dict(status='semantic_paint_with_optional_outer_vehicle_extent',peaks=peaks,vehicle_extent=extra,paint_phase=phase_review)


def continuous_edge_bounds(solid_bins,frame,width,actors,ego_positions,max_lanes,paint_peaks=()):
    """Nearest repeated solid edges bound a carriageway; shoulders add no lanes."""
    e=frame.project(ego_positions)[:,1];lo,hi=np.quantile(e,[.05,.95]);edges=[]
    for k,rows in solid_bins.items():
        frames={f for _,f in rows}
        if len(frames)>=5:edges.append(dict(index=k,y=float(np.median([d for d,_ in rows])),solid_frames=len(frames)))
    left=sorted([p for p in edges if p['y']>hi+.4],key=lambda p:p['y'])
    right=sorted([p for p in edges if p['y']<lo-.4],key=lambda p:-p['y'])
    if not right:return None,dict(status='insufficient_repeated_continuous_edges')
    if not left:return None,dict(status='left_unresolved_right_edge_retained',right_edge_candidate=right[0])
    a,b=left[0],right[0];span=a['y']-b['y'];count=int(round(span/width))
    if not 1<=count<=max_lanes or not .75*width<span/count<1.25*width:return None,dict(status='continuous_edge_spacing_unresolved',right_edge_candidate=b)
    # A dash occasionally projects as a long segment. It must not erase a
    # much more repeatedly observed adjacent stripe on an empty road merely
    # because no car is available to demonstrate that second lane. Yellow
    # divider scope has already removed opposite-carriageway paint peaks.
    outer=[p for p in paint_peaks if p['kind']=='white' and .75*width<p['y']-a['y']<1.25*width
           and p['frames']>=max(15,a['solid_frames']*2)]
    if outer:return None,dict(status='inner_continuous_candidate_has_stronger_adjacent_separator',
                              rejected_left_edge=a,stronger_outer_paint=outer,right_edge_candidate=b,
                              scope='camera_paint_ambiguity; not_vehicle_or_map_lane_count')
    blocked_left=[];blocked_right=[]
    for actor in actors:
        if actor['camera_id']!='front':continue
        rows=[r for r in actor['samples'] if r.get('support')=='detector_observation' and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=80
              and min(r['bbox_xyxy'][0],r['image_width']-r['bbox_xyxy'][2])>6]
        if len(rows)<15 or rows[-1]['time_s']-rows[0]['time_s']<3:continue
        local=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows]);t=np.array([r['time_s'] for r in rows])
        v=np.linalg.lstsq(np.c_[t-t[0],np.ones(len(t))],local[:,:2],rcond=None)[0][0];d=float(np.median(local[:,1]))
        if 2<v[0]<50 and abs(v[1])<v[0]*.5:
            if d>a['y']+.5:blocked_left.append(actor['id'])
            if d<b['y']-.5:blocked_right.append(actor['id'])
    if blocked_left:
        return None,dict(status='continuous_line_has_supported_forward_traffic_beyond_it',
                         left_traffic_ids=blocked_left,right_traffic_ids=blocked_right,
                         right_edge_candidate=b,ignored_right_traffic_ids=blocked_right)
    return np.linspace(a['y'],b['y'],count+1).tolist(),dict(status='repeated_continuous_carriageway_edges',edge_support=[a,b],
                internal_stripes='observed_or_lane_width_prior; outer shoulder lines excluded',independent_lane_count_measurement=False)


def continuous_paint_excludes_lane_jump(window,times,ego,yaws,reference,width=3.5):
    """Dense unaliased camera stripe phase can refute an old straight-frame event.

    The route is only a station index. A missing/low-confidence paint interval
    cannot refute a real lane change. No ego or vehicle pose is changed.
    """
    stations=reference.get('station_support',[])
    if len(stations)<6:return False
    try:index=route_index(ego,yaws=yaws)
    except ValueError:return False
    endpoints=np.array([[np.interp(t,times,np.asarray(ego)[:,k]) for k in range(2)] for t in [window['start_s'],window['end_s']]])
    lo,hi=index.project(endpoints)[:,0]
    if hi-lo<50:return False
    rows=[r for r in stations if lo-2<=r['s']<=hi+2]
    if len(rows)<6:return False
    s=np.array([r['s'] for r in rows]);phase=np.unwrap([r['phase'] for r in rows])
    if s[0]-lo>20 or hi-s[-1]>20 or np.max(np.diff(s))>20:return False
    if any(r.get('frames',0)<3 or r.get('concentration',0)<.9 for r in rows):return False
    if np.max(abs(np.diff(phase)))>.75 or np.ptp(phase)*width/(2*np.pi)>width*.4:return False
    return True


def phase_protection_windows(windows,times,yaws,road,max_alignment_deg=2.5):
    """A straight-road event cannot veto curvature when its own heading is wrong.

    A vehicle travelling at a persistent angle to an inaccurate straight road
    crosses its synthetic lane boundaries. That is not independent evidence
    of a lane change. Only aligned entry/exit headings may protect the old
    straight-road gauge against a paint-phase alias.
    """
    tangent=np.arctan(road.get('heading_evidence',{}).get('slope',0.))
    result=[]
    for w in windows:
        angles=np.interp([w['start_s'],w['end_s']],times,np.unwrap(yaws))-tangent
        angles=np.arctan2(np.sin(angles),np.cos(angles))
        if np.max(abs(angles))<=np.radians(max_alignment_deg):result.append(w)
    return result


def vehicle_supported_paint_gaps(review,reference,width,actors,ego_positions,max_lanes=4):
    """Bound observed traffic with paint; label missing internal stripes priors.

    Only a fallback for incomplete paint, using pre-road-refinement trajectories.
    Opposing and transverse motion cannot expand the carriageway. This is an
    estimated used corridor, not independent lane-count measurement.
    """
    peaks=review.get('peaks',[])
    if len(peaks)<2:return None,dict(status='insufficient_outer_paint')
    frame=RoadFrame(reference['points_W_xy'],reference.get('station_origin_m',0.));proof=[]
    for actor in actors:
        if actor['camera_id']!='front':continue
        rows=[r for r in actor['samples'] if r.get('support')=='detector_observation'
              and r['bbox_xyxy'][2]-r['bbox_xyxy'][0]>=80
              and min(r['bbox_xyxy'][0],r['image_width']-r['bbox_xyxy'][2])>6]
        if len(rows)<15 or rows[-1]['time_s']-rows[0]['time_s']<3:continue
        local=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows]);t=np.array([r['time_s'] for r in rows])
        motion=np.linalg.lstsq(np.c_[t-t[0],np.ones(len(t))],local[:,:2],rcond=None)[0][0]
        if not 2<motion[0]<50 or abs(motion[1])>motion[0]*.5:continue
        d=float(np.median(local[:,1]));spread=float(np.quantile(abs(local[:,1]-d),.8))
        curbs=[p['y'] for p in peaks if p.get('kind')=='continuous_right_edge_hypothesis']
        if curbs and d<max(curbs)-.4:continue
        if spread>1.2:continue
        proof.append(dict(local_id=actor['id'],lateral_m=d,clear_frames=len(rows),forward_mps=float(motion[0]),lateral_spread_m=spread))
    if len(proof)<2:return None,dict(status='insufficient_distinct_persistent_forward_tracks',vehicles=proof)
    centres=[r['lateral_m'] for r in proof];e=frame.project(ego_positions)[:,1]
    # Own trajectory only rejects impossible scope; it does not fit its lane.
    upper=max(max(centres),float(np.quantile(e,.9)));lower=min(min(centres),float(np.quantile(e,.1)))
    left=sorted([p for p in peaks if .4<p['y']-upper<width*1.3],key=lambda p:p['y'])
    right=sorted([p for p in peaks if .4<lower-p['y']<width*1.3],key=lambda p:-p['y'])
    if not right:
        inner=sorted([p for p in peaks if .3<p['y']-lower<width-.3],key=lambda p:p['y'])
        if inner:right=[dict(inner[0],index=inner[0]['index']-1,y=inner[0]['y']-width,kind='one_outer_vehicle_corridor_width_prior')]
    if not left:
        inner=sorted([p for p in peaks if .3<upper-p['y']<width-.3],key=lambda p:-p['y'])
        if inner:left=[dict(inner[0],index=inner[0]['index']+1,y=inner[0]['y']+width,kind='one_outer_vehicle_corridor_width_prior')]
    if not left or not right:return None,dict(status='traffic_not_bracketed_by_paint',vehicles=proof)
    a,b=left[0],right[0];count=a['index']-b['index'];span=a['y']-b['y']
    if not 1<=count<=max_lanes or not .75*width<span/count<1.25*width:return None,dict(status='inconsistent_paint_spacing',vehicles=proof)
    bounds=np.linspace(a['y'],b['y'],count+1).tolist()
    if np.mean((e>=bounds[-1]-.25)&(e<=bounds[0]+.25))<.85:return None,dict(status='corridor_excludes_own_route',vehicles=proof)
    observed={p['index'] for p in peaks};missing=[k for k in range(b['index']+1,a['index']) if k not in observed]
    return bounds,dict(status='outer_paint_and_persistent_forward_traffic_with_internal_width_priors',vehicles=proof,
                       outer_paint=[a,b],missing_stripe_indices=missing,independent_lane_count_measurement=False,
                       geometry_basis='unrefined_actor_observations; not road-snapped output')
