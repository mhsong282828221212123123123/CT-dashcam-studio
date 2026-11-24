"""A whole-track, smooth nonholonomic fit with withheld image samples.

Representative dimensions are estimated within bounded priors. Image agreement
does not establish true metric distance. No host pose or lane is optimized.
"""
import math
import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import least_squares
from scipy.sparse import csr_matrix


def fit_motion_bundle(observations,kind='car',max_speed=22.,max_distance=100.,ground_contact=True):
    from core.scene_fusion import corners,dimensions
    from core.scene_analysis import iou
    if len(observations)<8:return None
    times=np.array([o['time'] for o in observations]);times-=times[0]
    if times[-1]<.7 or np.any(np.diff(times)<=0):return None
    interior=np.arange(1.,times[-1]-.2,1.)
    active_times=times[[not o.get('stationary') for o in observations]]
    if len(active_times):interior=np.array([value for value in interior if min(abs(active_times-value))<1.2])
    else:interior=np.array([])
    knots=np.r_[np.zeros(4),interior,np.full(4,times[-1])];n=len(knots)-4
    basis=BSpline(knots,np.eye(n),3)
    B=basis(times);D=basis.derivative()(times);A=basis.derivative(2)(times)
    size=np.array(dimensions(kind));seed=[]
    for o in observations:
        camera=o['camera'];x,y,w,h=o['bbox'];cx=camera.width/2 if camera.principal_x is None else camera.principal_x
        ray=camera.rotation@np.array([-(x+w/2-cx)/camera.focal,-(y+h-camera.height/2)/camera.focal,1.])
        point=None
        if ray[1]<-.005 and not o.get('blocked'):
            point=(camera.origin-ray*camera.camera_height/ray[1])[[0,2]]
            direction=point-camera.origin[[0,2]];direction/=max(.01,np.linalg.norm(direction))
            point+=direction*size[1]/3
        if point is None or np.linalg.norm(point)>max_distance:
            depth=np.clip(camera.focal*size[2]/max(h,1),2,max_distance)
            local=np.array([-(x+w/2-cx)/camera.focal*depth,0,depth])
            point=(camera.origin+camera.rotation@local)[[0,2]]
        seed.append(o['ego_origin']+o['ego_rotation']@point)
    seed=np.asarray(seed)
    # Robust image-based seeds, rather than inheriting a filtered ID teleport.
    for k in range(2):seed[:,k]=[np.median(seed[max(0,j-2):j+3,k]) for j in range(len(seed))]
    coeff=np.linalg.lstsq(B,seed,rcond=None)[0]
    velocity=D@coeff;moving=np.linalg.norm(velocity,axis=1)>1.
    yaw=np.zeros(len(times))
    if moving.any():
        ids=np.flatnonzero(moving)
        for j in range(len(times)):
            k=min(ids,key=lambda k:abs(times[k]-times[j]));yaw[j]=math.atan2(velocity[k,0],velocity[k,1])
    elif observations[0].get('initial_yaw') is not None:yaw[:]=observations[0]['initial_yaw']
    for j,o in enumerate(observations):
        if o.get('reviewed_world_yaw') is not None:yaw[j]=float(o['reviewed_world_yaw'])
    # A stationary rectangle does not observe body yaw. Preserve the input
    # orientation over a confirmed stationary run instead of rotating the body
    # to explain generic-shape/box mismatch. This is a held prior, not a new
    # measurement; genuine moving turns retain the existing motion fit.
    held_yaw={};runs=[]
    for j,o in enumerate(observations):
        if not o.get('stationary'):continue
        if not runs or j!=runs[-1][-1]+1 or times[j]-times[runs[-1][-1]]>.3:runs.append([])
        runs[-1].append(j)
    for run in runs:
        if times[run[-1]]-times[run[0]]<1.:continue
        o=observations[run[0]];target=o.get('reviewed_world_yaw')
        if target is None and o.get('initial_yaw') is not None:
            axis=o['ego_rotation']@np.array([math.sin(o['initial_yaw']),math.cos(o['initial_yaw'])])
            target=math.atan2(axis[0],axis[1])
        if target is None:continue
        for j in run:
            held_yaw[j]=float(target)
            yaw[j]+=math.atan2(math.sin(target-yaw[j]),math.cos(target-yaw[j]))
    yaw=np.unwrap(yaw);yaw_coeff=np.linalg.lstsq(B,yaw,rcond=None)[0]
    initial=np.r_[coeff[:,0],coeff[:,1],yaw_coeff,[1.,1.,1.]]
    train=[];last_static=-np.inf
    for j in np.flatnonzero(np.arange(len(times))%3!=1):
        if observations[j].get('stationary'):
            if times[j]-last_static<.4:continue
            last_static=times[j]
        train.append(j)
    train=np.array(train,dtype=int);test=np.flatnonzero(np.arange(len(times))%3==1)
    if not len(test):return None

    def states(p):
        positions=np.column_stack([B@p[:n],B@p[n:2*n]])
        velocities=np.column_stack([D@p[:n],D@p[n:2*n]])
        return positions,velocities,B@p[2*n:3*n],size*p[-3:]

    def residual(p):
        positions,velocities,yaws,shape=states(p);errors=[]
        for j in train:
            o=observations[j];point=o['ego_rotation'].T@(positions[j]-o['ego_origin'])
            axis=o['ego_rotation'].T@np.array([math.sin(yaws[j]),math.cos(yaws[j])]);angle=math.atan2(axis[0],axis[1])
            bounds=o['camera'].visible_bounds(corners(*point,shape,yaw=angle,kind=kind))
            if bounds is None:errors.extend([10.]*4);continue
            x,y,w,h=o['bbox'];r=bounds-np.array([x,y,x+w,y+h])
            if o.get('blocked'):
                r[:2]=np.maximum(r[:2],0);r[2:]=np.minimum(r[2:],0)
            else:
                if x<3:r[0]=max(0,r[0])
                if x+w>o['camera'].width-3:r[2]=min(0,r[2])
                if y<3:r[1]=max(0,r[1])
                if y+h>o['camera'].height-3:r[3]=min(0,r[3])
                else:r[3]*=2
            minimum=20 if o.get('measurement_reliable',True) else 60
            if (not ground_contact or not o.get('measurement_reliable',True)) and not o.get('blocked'):
                # Tiny distant tyre rows cannot constrain a flat road height.
                # Retain bearing and apparent size without inventing a rapid
                # depth change to chase a one-pixel ground-contact fluctuation.
                r=np.array([(bounds[0]+bounds[2]-2*x-w)/2,
                            bounds[2]-bounds[0]-w,bounds[3]-bounds[1]-h,0.])
            errors.extend(r/np.array([max(minimum,w),max(minimum,h)]*2))
        lateral=velocities[:,0]*np.cos(yaws)-velocities[:,1]*np.sin(yaws)
        forward=velocities[:,0]*np.sin(yaws)+velocities[:,1]*np.cos(yaws)
        acceleration=np.column_stack([A@p[:n],A@p[n:2*n]])
        errors.extend(lateral*.35);errors.extend(np.minimum(forward,0)*.5)
        errors.extend(np.maximum(np.linalg.norm(velocities,axis=1)-max_speed,0)*.5)
        errors.extend(acceleration.ravel()*.025)
        errors.extend(np.maximum(np.linalg.norm(acceleration,axis=1)-8,0)*.15)
        errors.extend((A@p[2*n:3*n])*.06)
        errors.extend(np.maximum(abs(D@p[2*n:3*n])-math.radians(90),0)*.4)
        radius=1.2 if kind in ('motorcycle','bicycle','motorcycle_body','bicycle_body') else 2.5
        turn_limit=np.linalg.norm(velocities,axis=1)/radius+math.radians(1.)
        errors.extend(np.maximum(abs(D@p[2*n:3*n])-turn_limit,0)*2.)
        errors.extend((p[-3:]-1)*.5)
        # Occlusion-only observations need a weak depth anchor, not an invented
        # metric measurement. Visible-frame box constraints remain dominant.
        for j,o in enumerate(observations):
            if o.get('blocked'):errors.extend((positions[j]-seed[j])*.005)
            if o.get('stationary'):
                errors.extend(velocities[j]*.8);errors.append(float(D[j]@p[2*n:3*n])*.5)
            if j in held_yaw:
                target=held_yaw[j]
                errors.extend([3.*(math.sin(yaws[j])-math.sin(target)),
                               3.*(math.cos(yaws[j])-math.cos(target))])
            if o.get('reviewed_world_yaw') is not None:
                target=o['reviewed_world_yaw']
                errors.extend([.5*(math.sin(yaws[j])-math.sin(target)),.5*(math.cos(yaws[j])-math.cos(target))])
        return np.asarray(errors)

    # World origin is arbitrary and highway travel can exceed 1 km in a clip.
    # Bounds follow the observed host trajectory; final distances are still
    # checked camera-relatively. An unstable spline seed is not a valid reason
    # to abort every other track in the export.
    ego_positions=np.asarray([o['ego_origin'] for o in observations],float)
    lo=ego_positions.min(0)-max_distance;hi=ego_positions.max(0)+max_distance
    lower=np.r_[np.full(n,lo[0]),np.full(n,lo[1]),np.full(n,-20*np.pi),[.8,.8,.8]]
    upper=np.r_[np.full(n,hi[0]),np.full(n,hi[1]),np.full(n,20*np.pi),[1.2,1.2,1.2]]
    if not np.isfinite(initial).all():return None
    initial=np.clip(initial,lower+1e-6,upper-1e-6)
    # Clipped image bounds are float32. Explicit finite differences avoid steps
    # disappearing numerically; sparse spline support keeps long tracks cheap.
    patterns=[]
    supports=[np.flatnonzero((abs(B[j])+abs(D[j])+abs(A[j]))>1e-10) for j in range(len(times))]
    def columns(j,parts=(0,1,2),body=False):
        return {int(k+part*n) for part in parts for k in supports[j]}| (set(range(3*n,3*n+3)) if body else set())
    for j in train:patterns.extend([columns(j,body=True)]*4)
    for _ in range(2):patterns.extend(columns(j) for j in range(len(times)))
    patterns.extend(columns(j,(0,1)) for j in range(len(times)))
    for j in range(len(times)):
        patterns.extend([columns(j,(0,)),columns(j,(1,))])
    patterns.extend(columns(j,(0,1)) for j in range(len(times)))
    for _ in range(2):patterns.extend(columns(j,(2,)) for j in range(len(times)))
    patterns.extend(columns(j) for j in range(len(times)))
    patterns.extend([{3*n+k} for k in range(3)])
    for j,o in enumerate(observations):
        if o.get('blocked'):patterns.extend([columns(j,(0,)),columns(j,(1,))])
        if o.get('stationary'):patterns.extend([columns(j,(0,)),columns(j,(1,)),columns(j,(2,))])
        if j in held_yaw:patterns.extend([columns(j,(2,))]*2)
        if o.get('reviewed_world_yaw') is not None:patterns.extend([columns(j,(2,))]*2)
    column_rows=[{j for j,row in enumerate(patterns) if k in row} for k in range(len(initial))]
    groups=[];occupied=[]
    for k in sorted(range(len(initial)),key=lambda k:-len(column_rows[k])):
        for group,used in zip(groups,occupied):
            if not used.intersection(column_rows[k]):group.append(k);used.update(column_rows[k]);break
        else:groups.append([k]);occupied.append(set(column_rows[k]))
    def jacobian(p):
        step=1e-4;base=residual(p);J=np.zeros((len(base),len(p)))
        for group in groups:
            shifted=p.copy();shifted[group]+=step;change=(residual(shifted)-base)/step
            for k in group:
                indices=list(column_rows[k]);J[indices,k]=change[indices]
        return csr_matrix(J)
    fit=least_squares(residual,initial,bounds=(lower,upper),jac=jacobian,
                      loss='soft_l1',f_scale=.12,max_nfev=70)
    positions,velocities,yaws,shape=states(fit.x)
    scores=[];contacts=[];result=[];valid=[];coverages=[]
    for j,o in enumerate(observations):
        point=o['ego_rotation'].T@(positions[j]-o['ego_origin'])
        axis=o['ego_rotation'].T@np.array([math.sin(yaws[j]),math.cos(yaws[j])]);angle=math.atan2(axis[0],axis[1])
        bounds=o['camera'].visible_bounds(corners(*point,shape,yaw=angle,kind=kind))
        if bounds is None or np.linalg.norm(point)>max_distance:
            if o.get('measurement_reliable',True):return None
            result.append(dict(index=o['index'],position=point.tolist(),world_position=positions[j].tolist(),
                               yaw=angle,image_iou=0.,blocked=bool(o.get('blocked')),outside_supported_range=True))
            valid.append(False);continue
        x,y,w,h=o['bbox']
        actual_contact=abs(bounds[3]-y-h)/max(20,h)
        if not ground_contact:
            # A separate bearing/size hypothesis, not a calibrated ground fit.
            # Hills and different road levels cannot share a tyre plane.
            bounds=bounds.copy();bounds[[1,3]]+=(2*y+h-bounds[1]-bounds[3])/2
        score=iou(np.r_[bounds[:2],bounds[2:]-bounds[:2]],o['bbox'])
        area=max(0,min(x+w,bounds[2])-max(x,bounds[0]))*max(0,min(y+h,bounds[3])-max(y,bounds[1]))
        if o.get('blocked') and o.get('measurement_reliable',True) and j in test:coverages.append(area/max(1,w*h))
        if not o.get('blocked') and o.get('measurement_reliable',True) and j in test:
            scores.append(score)
            if y+h<o['camera'].height-3:contacts.append(actual_contact)
        valid.append(bool(score>=.5 and not o.get('blocked')))
        result.append(dict(index=o['index'],position=point.tolist(),world_position=positions[j].tolist(),
                           yaw=angle,image_iou=score,blocked=bool(o.get('blocked'))))
    speed=np.linalg.norm(velocities,axis=1)
    lateral=abs(velocities[:,0]*np.cos(yaws)-velocities[:,1]*np.sin(yaws))
    projection_hypothesis=bool(not ground_contact and len(scores)>=3 and np.median(scores)>=.65
        and np.mean(np.array(scores)>=.5)>=.9 and speed.max()<=max_speed+1
        and np.percentile(lateral,95)<1.)
    accepted=bool(ground_contact and scores and np.median(scores)>=.65 and np.mean(np.array(scores)>=.5)>=.9
        and (not contacts or np.percentile(contacts,95)<=.15)
        and speed.max()<=max_speed+1 and np.percentile(lateral,95)<1.)
    hypothesis=bool(ground_contact and len(scores)<3 and len(coverages)>=3 and np.median(coverages)>=.9
        and np.mean(np.array(coverages)>=.8)>=.9 and speed.max()<=max_speed+1
        and np.percentile(lateral,95)<1.)
    return dict(accepted=accepted,positions=result,dimensions=shape.tolist(),kind=kind,
        occlusion_hypothesis=hypothesis,
        projection_hypothesis=projection_hypothesis,
        validation_alignment='original_image' if ground_contact else 'vertical_translation_free; ground height unverified',
        knots=knots.tolist(),coefficients=fit.x[:3*n].reshape(3,n).T.tolist(),
        reference_time_s=observations[0]['time'],
        validation_iou_median=float(np.median(scores)) if scores else None,
        validation_fraction_above_half=float(np.mean(np.array(scores)>=.5)) if scores else 0,
        contact_error_p95=float(np.percentile(contacts,95)) if contacts else None,
        max_speed_mps=float(speed.max()),lateral_speed_p95_mps=float(np.percentile(lateral,95)),
        speed_limit_mps=float(max_speed),world_bounds_source='host_trajectory_plus_camera_relative_range',
        max_body_turn_deg_s=float(np.degrees(abs(D@fit.x[2*n:3*n])).max()),
        stationary_heading_prior_frames=len(held_yaw),
        heading_accuracy='held input orientation at standstill; box/motion fit while moving; not independently measured',
        withheld_samples=len(scores),valid_image_frames=sum(valid),
        metric_accuracy='not surveyed; bounded representative body dimensions')
