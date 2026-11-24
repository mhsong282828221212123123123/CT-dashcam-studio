"""Station-dependent left road extent from repeated camera paint evidence.

Paint-only geometry and explicitly labelled vehicle-assisted inferred corridors.
No ego pose changes or GPS lane assignment. Optical/mount scale is estimated.
"""
from collections import defaultdict
from pathlib import Path
import cv2
import numpy as np
from .affine_probe import camera_matrix
from .road_frame import frame_for


def rotation(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def paint_observations(package,frames,observations,times,ego,yaws,profiles,mounts,slope,road=None,road_masks=None):
    points=[];segments=[];package=Path(package)
    road_frame=frame_for(road) if road else None
    mask_rows={}
    if road_frame is not None:
        if road_masks is None:raise ValueError('Curved pocket projection needs raw-space road masks')
        import json
        mask_rows={r['frame_id']:r for r in json.loads((Path(road_masks)/'frames.json').read_text(encoding='utf-8'))}
    def local(world):
        return road_frame.project(world)[:,:2] if road_frame is not None else np.c_[world[:,0],world[:,1]-slope*world[:,0]]
    for camera in (('front',) if road_frame is not None else ('front','left_repeater')):
        K,D=camera_matrix(profiles[camera]);mount=mounts[camera]
        for f in [f for f in frames if f['camera_id']==camera][::1 if road_frame is not None else 3]:
            image=cv2.imread(str(package/f['image']));h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            allowed=np.zeros((h,w),np.uint8);allowed[int(h*.57):int(h*.96),:]=255
            if road_frame is not None:
                mask_row=mask_rows.get(f['frame_id'])
                if mask_row is None:continue
                ground=cv2.imread(str(Path(road_masks)/mask_row['road']),cv2.IMREAD_GRAYSCALE)
                if ground is None or ground.shape!=(h,w):raise ValueError('Raw road-mask shape mismatch')
                # A raised median's painted top is not a ground-plane contact.
                # Keep only the drivable surface, with a pixel-scale edge margin.
                allowed=cv2.bitwise_and(allowed,cv2.dilate(ground,np.ones((3,3),np.uint8)))
            for row in observations.get(f['frame_id'],[]):
                x,y,X,Y=np.rint(row['bbox_xyxy']).astype(int);allowed[max(0,y-6):min(h,Y+6),max(0,x-6):min(w,X+6)]=0
            orange=((hsv[:,:,0]<20)&(hsv[:,:,1]>65)&(hsv[:,:,2]>65)).astype('uint8')*255
            orange[:int(h*.35)]=0
            closed=cv2.morphologyEx(orange,cv2.MORPH_CLOSE,np.ones((35,3),np.uint8))
            closed=cv2.morphologyEx(closed,cv2.MORPH_OPEN,np.ones((45,3),np.uint8))
            _,_,stats,_=cv2.connectedComponentsWithStats(closed)
            poles=[]
            for x,y,bw,bh,area in stats[1:]:
                if 3<=bw<=w*.06 and 35<=bh<=h*.6 and 2.5<bh/bw<25 and area/(bw*bh)>.25:
                    if y+bh<h*.96 and allowed[min(h-1,y+bh),min(w-1,x+bw//2)]>0:poles.append((x+bw/2,y+bh))
                    allowed[max(0,y-3):min(h,y+bh+3),max(0,x-4):min(w,x+bw+4)]=0
            contrast=cv2.subtract(hsv[:,:,2],cv2.GaussianBlur(hsv[:,:,2],(0,0),5))
            masks={'yellow':(hsv[:,:,0]>=18)&(hsv[:,:,0]<=40)&(hsv[:,:,1]>70)&(hsv[:,:,2]>70),
                   'white':(hsv[:,:,1]<70)&(hsv[:,:,2]>105)&(contrast>10)}
            t=f['camera_time_s'];ep=np.array([np.interp(t,times,ego[:,k]) for k in range(3)]);yaw=np.interp(t,times,yaws)
            R=rotation(yaw);C=rotation(np.radians(mount['yaw_deg']))
            if len(poles)>=3:
                rays=cv2.undistortPoints(np.asarray(poles,float).reshape(-1,1,2),K,D).reshape(-1,2)
                good=rays[:,1]>.025;rays=rays[good];pixels=np.asarray(poles)[good]
                depth=mount['height_m']/rays[:,1]
                relative=np.c_[depth,-depth*rays[:,0],np.zeros(len(depth))]@C.T+mount['offset_W_ego_m']
                world=relative@R.T+ep;ground=local(world);lateral=ground[:,1]
                valid=(np.linalg.norm(relative[:,:2],axis=1)<35)&(lateral>0)&(lateral<15)
                world=world[valid];ground=ground[valid];lateral=lateral[valid];pixels=pixels[valid]
                if len(world)>=3 and np.ptp(ground[:,0])>=3:
                    design=np.c_[ground[:,0]-np.mean(ground[:,0]),np.ones(len(world))]
                    fit=np.linalg.lstsq(design,lateral,rcond=None)[0]
                    if abs(fit[0])<.45 and np.quantile(abs(design@fit-lateral),.8)<.45:
                        for p,q,pixel in zip(ground,lateral,pixels):
                            sid=len(segments);segments.append(dict(frame_id=f['frame_id'],camera_id=camera,kind='bollard_base',pixels=[*pixel.tolist(),*pixel.tolist()]))
                            points.append(dict(x=float(p[0]),y=float(q),frame_id=f['frame_id'],camera_id=camera,kind='yellow',origin='bollard_base',segment_id=sid))
            for kind,mask in masks.items():
                binary=((mask&(allowed>0))*255).astype('uint8')
                lines=cv2.HoughLinesP(binary,1,np.pi/180,threshold=35,minLineLength=45,maxLineGap=12)
                if lines is None:continue
                for line in lines.reshape(-1,4):
                    x,y,X,Y=line
                    pixels=np.array([[[x+u*(X-x),y+u*(Y-y)]] for u in np.linspace(0,1,7)],float)
                    rays=cv2.undistortPoints(pixels,K,D).reshape(-1,2)
                    if np.any(rays[:,1]<=.025):continue
                    depth=mount['height_m']/rays[:,1]
                    if np.any(depth<.8) or np.any(depth>40):continue
                    relative=np.c_[depth,-depth*rays[:,0],np.zeros(len(depth))]@C.T+mount['offset_W_ego_m']
                    if np.any(np.linalg.norm(relative[:,:2],axis=1)>40):continue
                    world=relative@R.T+ep;ground=local(world);delta=ground[-1]-ground[0]
                    lateral=ground[:,1]
                    crossbar=kind=='white' and abs(delta[1])>2 and abs(delta[0])<abs(delta[1])*.3
                    if not crossbar and (abs(delta[0])<1 or abs(delta[1])>abs(delta[0])*.45):continue
                    if not np.all((lateral>(-8 if crossbar else 0))&(lateral<15)):continue
                    label='crossbar' if crossbar else kind
                    sid=len(segments);segments.append(dict(frame_id=f['frame_id'],camera_id=camera,kind=label,pixels=line.tolist()))
                    points.extend(dict(x=float(p[0]),y=float(q),frame_id=f['frame_id'],camera_id=camera,kind=label,origin='ground_paint',segment_id=sid) for p,q in zip(ground,lateral))
    return dict(points=points,segments=segments,geometry='estimated_ground_paint; not measured rig',
                coordinate_system='road_station_lateral' if road_frame is not None else 'world_x_and_straight_road_lateral',
                ground_support='road_mask_ground_region' if road_frame is not None else 'legacy_unverified_ground_region',
                cameras_used=['front'] if road_frame is not None else ['front','left_repeater'])


def stations(points,step=10):
    bins=defaultdict(list)
    for p in points:bins[round(p['x']/step)*step,p['kind']].append(p)
    result=[]
    for (x,kind),rows in sorted(bins.items()):
        modes=defaultdict(list)
        for r in rows:modes[round(r['y']/.25)*.25].append(r)
        peaks=[]
        for y,pool in sorted(modes.items(),key=lambda kv:len({r['frame_id'] for r in kv[1]}),reverse=True):
            if any(abs(y-p['y'])<.6 for p in peaks):continue
            near=[r for r in rows if abs(r['y']-y)<=.35];fids={r['frame_id'] for r in near}
            if len(fids)<3:continue
            peaks.append(dict(y=float(np.median([r['y'] for r in near])),frames=len(fids),cameras=sorted({r['camera_id'] for r in near}),frame_ids=sorted(fids),origins=sorted({r.get('origin','ground_paint') for r in near})))
        if peaks:result.append(dict(x=x,kind=kind,peaks=peaks[:8]))
    return result


def fit_pockets(station_rows,road):
    """Repeated outer divergence + continuing inner separator + return.

    White paint may bridge a ground-edge occlusion only after divergence is
    independently supported by yellow paint / bollard ground contacts.
    """
    left=road['boundaries_y'][0];width=road.get('lane_width_m_prior',3.5)
    yellow=[s for s in station_rows if s['kind']=='yellow'];white=[s for s in station_rows if s['kind']=='white']
    normal=[s['x'] for s in yellow if any(abs(p['y']-left)<.8 for p in s['peaks'])]
    candidates=[];pockets=[]
    def inner_at(x):
        return [(s,p) for s in white if abs(s['x']-x)<=15 for p in s['peaks'] if abs(p['y']-left)<.8]
    for s in yellow:
        for p in s['peaks']:
            extra=p['y']-left
            if not 1.7<=extra<=width*1.5:continue
            row=dict(peak_x=s['x'],outer_y=p['y'],frame_ids=p['frame_ids'],origins=p.get('origins',[]),status='rejected')
            candidates.append(row);inner=inner_at(s['x'])
            before=[x for x in normal if 5<=s['x']-x<=40];after=[x for x in normal if 25<=x-s['x']<=160]
            if not before or not after or len({a['x'] for a,_ in inner})<2:
                row['reason']='needs_divergence_bracket_and_persistent_inner_separator';continue
            start=max(before);end=min(after)
            # A parallel through-lane separator must remain, or an overall
            # curve/road shift could be mistaken for an extra lane.
            through=[a for a in white if s['x']<=a['x']<=s['x']+30 and any(abs(q['y']-(left-width))<.8 for q in a['peaks'])]
            if not through:row['reason']='through_corridor_not_independently_supported';continue
            inner_y=float(np.median([q['y'] for _,q in inner]));observed_width=p['y']-inner_y
            continuation=[]
            for a in white:
                if not s['x']<a['x']<end or not inner_at(a['x']):continue
                outer=[q for q in a['peaks'] if abs(q['y']-p['y'])<=.8]
                if outer:continuation.append(a['x'])
            support_x=[s['x']]+continuation
            missing_supported=True
            for a,b in zip(sorted(support_x),sorted(support_x)[1:]):
                if b-a<=40:continue
                middle=np.arange(a+10,b,10)
                if b-a>80 or not all(inner_at(x) for x in middle) or any(a<x<b for x in normal):missing_supported=False
            if len(support_x)<3 or max(support_x)-s['x']<20 or not missing_supported:
                row['reason']='insufficient_outer_continuation_across_occlusion';continue
            last=max(support_x)
            if end-last>30:row['reason']='return_too_far_from_last_width_support';continue
            if any(start<q['end_x'] and end>q['start_x'] for q in pockets):row['reason']='overlapping_supported_candidate';continue
            # Existing rig is estimated; gently regularize a narrow ground
            # projection only after a pocket is independently established.
            # This is not a new lane generated by a width prior.
            measured=float(np.clip(.75*observed_width+.25*width,1.8,width*1.5))
            pocket=dict(id='left-pocket-'+str(len(pockets)),lane_id=-1,start_x=float(start),end_x=float(end),
                        width_knots=[[float(start),0.],[float(s['x']),measured],[float(last),measured],[float(end),0.]],
                        observed_width_m=observed_width,inner_reference_y=left,
                        width_support='estimated contacts 75 percent; nominal lane width prior 25 percent',
                        evidence_peak_x=s['x'],evidence_frame_ids=p['frame_ids'],continuation_stations=continuation,
                        support='paired_separator_and_outer_ground_edge_with_observed_return',
                        hidden_interval='white_edge_continuation; width estimated',turn_role='left_auxiliary_pocket; turn_direction_not_classified')
            pockets.append(pocket);row.update(status='accepted',reason='paired_station_geometry',pocket_id=pocket['id'])
    return dict(pockets=pockets,candidates=candidates,lane_ids='main IDs unchanged; left pocket -1',occupancy_used=False)


def fit_curved_pockets(evidence,road):
    """Front-only paired paint; junction-bound pockets need no fictitious return.

    A camera-supported junction can terminate an observed pocket. It cannot
    establish its width: repeated outer paint, inner and through separators
    must all be present. Main lanes and world poses remain unchanged.
    """
    frame=frame_for(road)
    if frame is None or evidence.get('coordinate_system')!='road_station_lateral':
        raise ValueError('Curved pocket evidence requires road-station coordinates')
    if evidence.get('ground_support')!='road_mask_ground_region':raise ValueError('Raised surfaces must not establish a ground-plane pocket')
    if any(p['camera_id']!='front' for p in evidence['points']):raise ValueError('Unaccepted side metric paint is not used')
    white=stations([p for p in evidence['points'] if p['kind']=='white']);width=road.get('lane_width_m_prior',3.5)
    extent=road['reference_line'].get('supported_extent_m',[float(frame.stations[0]),float(frame.stations[-1])])
    sections=road.get('lane_sections') or [dict(station_range_m=extent,boundaries_y=road['boundaries_y'])]
    pockets=[];reviews=[]
    for section in sections:
        lo,hi=section['station_range_m'];left=section['boundaries_y'][0]
        localrows=stations([p for p in evidence['points'] if lo<=p['x']<hi])
        closed=fit_pockets(localrows,dict(road,boundaries_y=section['boundaries_y']))
        for p in closed['pockets']:p['coordinate_system']='road_station_lateral';pockets.append(p)
        if not any(abs(j['road_station_range_m'][0]-hi)<1 for j in road.get('junctions',[])):continue
        ys=[p for p in evidence['points'] if lo<=p['x']<hi and p['kind']=='yellow' and 1.7<=p['y']-left<=width*1.5]
        seeds=sorted({(round(p['x']/20)*20,round(p['y']/.25)*.25) for p in ys})
        for station,y in seeds:
            anchor=[p for p in ys if abs(p['x']-station)<=30 and abs(p['y']-y)<.6]
            fids={p['frame_id'] for p in anchor}
            if len(fids)<3 or np.ptp([p['x'] for p in anchor])<10:continue
            outer=float(np.median([p['y'] for p in anchor]));paired=[]
            for r in white:
                if not lo<=r['x']<hi:continue
                inner=[p for p in r['peaks'] if abs(p['y']-left)<.8]
                outside=[p for p in r['peaks'] if abs(p['y']-outer)<.8]
                if inner and outside:paired.append((r['x'],min(inner,key=lambda p:abs(p['y']-left)),min(outside,key=lambda p:abs(p['y']-outer))))
            # Follow a contiguous observed pair to the junction, rather than
            # filling an arbitrary road span from one yellow fragment.
            groups=[]
            for row in sorted(paired):
                if not groups or row[0]-groups[-1][-1][0]>30:groups.append([row])
                else:groups[-1].append(row)
            groups=[g for g in groups if len(g)>=3 and g[-1][0]-g[0][0]>=20 and g[0][0]-30<=station<=g[-1][0]+30 and 0<=hi-g[-1][0]<=30]
            if not groups:continue
            group=max(groups,key=len);through=[r for r in white if group[0][0]<=r['x']<=group[-1][0] and any(abs(p['y']-(left-width))<.8 for p in r['peaks'])]
            if len(through)<3:continue
            start=max(lo,group[0][0]-10);end=float(hi)
            if any(start<p['end_x'] and end>p['start_x'] for p in pockets):continue
            measured=float(np.clip(.75*np.median([out['y']-inside['y'] for _,inside,out in group])+.25*width,1.8,width*1.5))
            ids=fids|{fid for _,inside,out in group for peak in [inside,out] for fid in peak['frame_ids']}
            p=dict(id='left-pocket-'+str(len(pockets)),lane_id=-1,start_x=float(start),end_x=end,
                   width_knots=[[float(start),0.],[float(group[0][0]),measured],[end,measured]],
                   observed_width_m=float(outer-left),inner_reference_y=float(left),coordinate_system='road_station_lateral',
                   support='front_outer_yellow_and_paired_white_edges_ending_at_supported_junction',
                   entry_support='10m_taper_inferred; preceding_return_not_observed',exit_support='supported_junction_opening; no_closed_return_claim',
                   width_support='paired_camera_width_75percent_nominal_prior25percent',evidence_frame_ids=sorted(ids),
                   yellow_anchor_frame_ids=sorted(fids),continuation_stations=[x for x,_,_ in group],
                   turn_role='left_auxiliary_pocket; turn_direction_not_classified')
            pockets.append(p);reviews.append(dict(status='accepted_junction_terminated_pocket',station_range_m=[start,end],yellow_frames=len(fids),paired_stations=len(group)))
    for i,p in enumerate(pockets):p['id']='left-pocket-'+str(i)
    return dict(pockets=pockets,candidates=reviews,lane_ids='main IDs unchanged; left pocket -1',occupancy_used=False,
                coordinate_system='road_station_lateral',side_metric_geometry_used=False,ego_modified=False)


def vehicle_supported_curved_pockets(evidence,road,actors,cues,geometry_fit):
    """A rear-view car queue plus continuing ground separators supports an
    inferred auxiliary corridor when its outer paint is hidden by that queue.
    Car presence alone and velocity alone cannot establish this corridor.
    """
    frame=frame_for(road);result=dict(geometry_fit,pockets=list(geometry_fit['pockets']),candidates=list(geometry_fit['candidates']))
    white=stations([p for p in evidence['points'] if p['kind']=='white']);width=road.get('lane_width_m_prior',3.5)
    for section in road.get('lane_sections') or [dict(station_range_m=road['reference_line']['supported_extent_m'],boundaries_y=road['boundaries_y'])]:
        lo,hi=section['station_range_m'];left=section['boundaries_y'][0]
        if not any(abs(j['road_station_range_m'][0]-hi)<1 for j in road.get('junctions',[])):continue
        for actor in actors:
            if actor['camera_id']!='front':continue
            cue=cues.get(actor['id'],{}).get('samples',[])
            if len({c['frame_id'] for c in cue})<3 or np.ptp([c['time_s'] for c in cue])<.4:continue
            rows=[r for r in actor['samples'] if r.get('quality',0)>=.5 and r.get('support')=='detector_observation']
            if len(rows)<8:continue
            local=frame.project([r.get('unrefined_position_W',r['position_W']) for r in rows])
            chosen=[(r,q) for r,q in zip(rows,local) if max(lo,hi-80)<q[0]<hi and left+.5<q[1]<left+width-.25]
            if len(chosen)<8:continue
            t=np.array([r['time_s'] for r,q in chosen]);q=np.array([q for r,q in chosen]);duration=np.ptp(t)
            if duration<1.2 or hi-np.quantile(q[:,0],.9)>50:continue
            cue=[c for c in cue if min(t)-1<=c['time_s']<=max(t)+1]
            if len({c['frame_id'] for c in cue})<3 or np.ptp([c['time_s'] for c in cue])<.4:continue
            motion=np.median([(q[j,:2]-q[i,:2])/(t[j]-t[i]) for i in range(len(t)) for j in range(i+1,len(t)) if t[j]-t[i]>=.8],axis=0)
            # A stopped queue's contact-depth noise may have a negative fitted
            # velocity. Rear appearance supplies polarity here; velocity is
            # only an outlier/crossing check, never the direction ground truth.
            if abs(motion[0])>=40 or abs(motion[1])>1.5:continue
            first=float(np.quantile(q[:,0],.1));last=float(np.quantile(q[:,0],.9))
            inner=[r for r in white if max(lo,first-30)<=r['x']<hi and any(abs(p['y']-left)<.8 for p in r['peaks'])]
            through=[r for r in white if max(lo,first-30)<=r['x']<hi and any(abs(p['y']-(left-width))<.8 for p in r['peaks'])]
            if len(inner)<3 or len(through)<3 or np.ptp([r['x'] for r in inner])<20:continue
            start=max(lo,min(first,min(r['x'] for r in inner))-10)
            if any(start<p['end_x'] and hi>p['start_x'] for p in result['pockets']):continue
            outer=[(r['x'],p['y']-left) for r in white if first-30<=r['x']<hi for p in r['peaks'] if abs(p['y']-(left+width))<.8]
            outer_seen=len({x for x,d in outer})>=3
            estimated=float(.75*np.median([d for x,d in outer])+.25*width) if outer_seen else float(width)
            p=dict(id='left-pocket-'+str(len(result['pockets'])),lane_id=-1,start_x=float(start),end_x=float(hi),
                   width_knots=[[float(start),0.],[float(start+10),estimated],[float(hi),estimated]],inner_reference_y=float(left),
                   coordinate_system='road_station_lateral',support='rear_appearance_vehicle_and_parallel_ground_separators_inferred_auxiliary_lane',
                   width_support='partial_outer_white_paint_with_width_prior' if outer_seen else 'nominal_width_prior; outer_edge_unobserved',
                   geometry_status='inferred_from_camera_vehicle_and_paint; not_measured_outer_edge',
                   direction_support='repeated_rear_appearance; metric_velocity_sign_not_used',
                   entry_support='10m_taper_inferred',exit_support='supported_junction_opening',
                   vehicle_local_id=actor['id'],vehicle_frame_ids=[r['frame_id'] for r,q in chosen],
                   rear_cue_frame_ids=sorted({c['frame_id'] for c in cue}),separator_stations=[r['x'] for r in inner],
                   turn_role='left_auxiliary_corridor; turn_direction_not_classified')
            result['pockets'].append(p);result['candidates'].append(dict(status='inferred_vehicle_supported_auxiliary_lane',vehicle_local_id=actor['id'],station_range_m=[start,hi]))
            result['occupancy_used']=True
    return result
