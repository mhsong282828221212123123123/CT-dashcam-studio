"""Experimental offline box/trajectory fitting with withheld time samples.

Returns proposals only. Image reprojection is not independent metric truth.
An object is not inserted or rotated by this module without downstream review.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from core.scene_fusion import corners,dimensions
from core.scene_analysis import iou


def fit_trajectory(observations,seed_world,kind='car',turn_rate=False,heading_coupled=False,max_distance=65.):
    if len(observations)<8:return None
    times=np.array([o['time'] for o in observations],float)
    if np.ptp(times)<.7 or np.ptp(times)>4:return None
    reference=float(np.mean(times));dt=times-reference
    seed=np.asarray(seed_world,float)
    valid=np.isfinite(seed).all(axis=1)
    if valid.sum()<3:return None
    velocity,origin=np.linalg.lstsq(np.column_stack([dt[valid],np.ones(valid.sum())]),seed[valid],rcond=None)[0]
    if not np.isfinite(np.r_[velocity,origin]).all():return None
    size=dimensions(kind)
    train=list(range(0,len(times),2));test=list(range(1,len(times),2))

    def predict(parameters,index):
        o=observations[index]
        angle=parameters[5]*dt[index] if turn_rate else 0.
        if turn_rate and abs(parameters[5])>1e-6:
            s=math.sin(angle)/parameters[5];c=(1-math.cos(angle))/parameters[5]
            vx,vz=parameters[2:4]
            world=parameters[:2]+np.array([vx*s+vz*c,-vx*c+vz*s])
        else:world=parameters[:2]+parameters[2:4]*dt[index]
        local=o['ego_rotation'].T@(world-o['ego_origin'])
        heading=math.atan2(parameters[2],parameters[3]) if heading_coupled else parameters[4]
        axis=o['ego_rotation'].T@np.array([math.sin(heading+angle),math.cos(heading+angle)])
        yaw=math.atan2(axis[0],axis[1])
        bounds=o['camera'].visible_bounds(corners(*local,size,yaw=yaw,kind=kind))
        return local,yaw,bounds

    def residual(parameters,indices):
        errors=[]
        for index in indices:
            o=observations[index];_,_,bounds=predict(parameters,index)
            x,y,w,h=o['bbox']
            if bounds is None:errors.extend([10.]*4);continue
            r=(bounds-np.array([x,y,x+w,y+h]))/np.array([max(20,w),max(20,h)]*2)
            # Clipped edges do not constrain the hidden vehicle extent.
            if x<3:r[0]=max(0,r[0])
            if x+w>o['camera'].width-3:r[2]=min(0,r[2])
            if y<3:r[1]=max(0,r[1])
            if y+h>o['camera'].height-3:r[3]=min(0,r[3])
            else:r[3]*=2
            errors.extend(r)
        return np.asarray(errors)

    candidates=[]
    def numerical_jacobian(function,parameters):
        # OpenCV image clipping uses float32. Relative sqrt(eps) steps can
        # disappear there, especially for zero velocity/yaw-rate parameters.
        step=1e-4
        eye=np.eye(len(parameters))*step
        return np.column_stack([(function(parameters+d)-function(parameters-d))/(2*step)
                                for d in eye])

    heading=math.atan2(velocity[0],velocity[1]) if np.linalg.norm(velocity)>1 else 0.
    # The envelope is front/back symmetric: heading +/- pi/2 are the same
    # initial axis, not independent starts. Cover the entire axis interval;
    # a seed derived from a zero-yaw placement can itself have biased velocity.
    starts=[heading] if heading_coupled else [heading]+list(np.linspace(-math.pi/2,math.pi/2,8,endpoint=False))
    for yaw in starts:
        suffix=[yaw,0.] if turn_rate else [yaw]
        initial=np.r_[origin,np.clip(velocity,-50,50),suffix]
        fixed_residual=lambda p:residual(np.r_[p,suffix],train)
        full_residual=lambda p:residual(p,train)
        fixed=least_squares(fixed_residual,initial[:4],
                            jac=lambda p:numerical_jacobian(fixed_residual,p),
                            bounds=(np.r_[origin-20,[-55,-55]],np.r_[origin+20,[55,55]]),
                            loss='soft_l1',f_scale=.1,max_nfev=100)
        initial[:4]=fixed.x
        fit=least_squares(full_residual,initial,loss='soft_l1',f_scale=.1,
                          jac=lambda p:numerical_jacobian(full_residual,p),
                          bounds=(np.r_[origin-20,[-55,-55],[yaw-math.pi/2,-.8] if turn_rate else [yaw-math.pi/2]],
                                  np.r_[origin+20,[55,55],[yaw+math.pi/2,.8] if turn_rate else [yaw+math.pi/2]]),max_nfev=100)
        candidates.append((float(np.mean(residual(fit.x,train)**2)),fit.x))
    _,parameters=min(candidates,key=lambda c:c[0])
    if np.linalg.norm(parameters[2:4])>55:return None
    scores=[];contact=[];positions=[]
    for index,o in enumerate(observations):
        local,yaw,bounds=predict(parameters,index)
        if bounds is None or np.linalg.norm(local)>max_distance:return None
        x,y,w,h=o['bbox']
        if index in test:
            scores.append(iou(np.r_[bounds[:2],bounds[2:]-bounds[:2]],o['bbox']))
            if y+h<o['camera'].height-3:contact.append(abs(bounds[3]-y-h)/max(20,h))
        positions.append(dict(time=o['time'],position=local.tolist(),axis_yaw=float(yaw)))
    accepted=(np.median(scores)>=.65 and np.mean(np.array(scores)>=.5)>=.9
              and (not contact or np.percentile(contact,95)<=.15))
    return dict(accepted=bool(accepted),world_velocity=parameters[2:4].tolist(),
                world_origin=parameters[:2].tolist(),reference_time_s=reference,
                axis_world_yaw_rad=float(math.atan2(parameters[2],parameters[3]) if heading_coupled else parameters[4]),
                motion_model='constant_turn_rate' if turn_rate else 'constant_velocity',
                yaw_rate_rad_s=float(parameters[5]) if turn_rate else 0.,
                validation_iou_median=float(np.median(scores)),
                validation_iou_min=float(min(scores)),
                validation_contact_error_p95=float(np.percentile(contact,95)) if contact else None,
                training_samples=len(train),withheld_samples=len(test),positions=positions,
                heading_semantics='forward travel assumption; body axis coupled to trajectory' if heading_coupled else 'vehicle axis; front/back direction unresolved',
                metric_accuracy='not evaluated')
