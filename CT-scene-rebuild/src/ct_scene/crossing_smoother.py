"""Robust fixed-interval world-XY smoothing for explicit cross-street tracks.

Solves every position jointly under camera-ray measurement covariance and a
continuous-velocity (acceleration-noise) prior. No road projection, two-point
velocity initialization, or prediction-triggered rejection of later evidence.
"""
import math
import numpy as np


def smooth_crossing(actor,samples,step_obs,sigma_accel=3.,coordinate_frame='world_xy_crossing'):
    n=len(samples);ts=np.array([r['time_s'] for r in samples],float)
    if n<3 or np.any(np.diff(ts)<=0):return dict(applied=False,reason='insufficient_ordered_crossing_samples')
    observations=[]
    for i,obs in enumerate(step_obs):
        for xy,var_x,var_y,camera,cov in obs:
            R=np.diag([var_x,var_y]) if cov is None else np.asarray(cov,float)
            R=.5*(R+R.T)+np.eye(2)*1e-6
            observations.append((i,np.asarray(xy,float),np.linalg.inv(R)))
    if len(observations)<3:return dict(applied=False,reason='too_few_crossing_observations')
    # Integrated acceleration residual: velocity difference over each adjacent
    # interval. Isotropic world acceleration keeps the fit rotation invariant.
    D=np.zeros((n-2,n))
    for i in range(1,n-1):
        left=ts[i]-ts[i-1];right=ts[i+1]-ts[i]
        scale=1./(sigma_accel*math.sqrt((left+right)/2))
        D[i-1,i-1]=scale/left;D[i-1,i]=-scale*(1/left+1/right);D[i-1,i+1]=scale/right
    prior=np.kron(D.T@D,np.eye(2))+np.eye(2*n)*1e-9
    weights=np.ones(len(observations))
    for iteration in range(5):
        normal=prior.copy();rhs=np.zeros(2*n)
        for weight,(i,xy,precision) in zip(weights,observations):
            sl=slice(2*i,2*i+2);W=weight*precision;normal[sl,sl]+=W;rhs[sl]+=W@xy
        xy_fit=np.linalg.solve(normal,rhs).reshape(n,2)
        residuals=[float((xy_fit[i]-xy)@precision@(xy_fit[i]-xy)) for i,xy,precision in observations]
        # Huber on the complete offline fit, never on one forward prediction.
        weights=np.minimum(1.,np.sqrt(9.21/np.maximum(residuals,1e-9)))
    posterior=np.linalg.inv(normal);velocity=np.gradient(xy_fit,ts,axis=0)
    for i,row in enumerate(samples):
        row['position_before_crossing_world_W']=list(row['position_W'])
        row['position_W']=[*map(float,xy_fit[i]),float(row['position_W'][2])]
        cov=posterior[2*i:2*i+2,2*i:2*i+2]
        row['position_covariance_W_xy']=cov.tolist();row['kalman_coordinate_frame']=coordinate_frame
        row['kalman_sigma_s_m']=float(math.sqrt(max(cov[0,0],.02)))
        row['kalman_sigma_d_m']=float(math.sqrt(max(cov[1,1],.02)))
        row['kalman_sigma_m']=float(math.sqrt(max(np.trace(cov),.04)))
        row['position_smoothing_support']='offline_all_observations_camera_ray_covariance'
        if np.linalg.norm(velocity[i])>1.8:row['yaw_rad']=float(math.atan2(velocity[i,1],velocity[i,0]))
    return dict(applied=True,coordinate_frame=coordinate_frame,method='robust_fixed_interval_acceleration_prior',samples=n,
                observations=len(observations),downweighted_observations=int(sum(weights<.999)),
                multi_camera_fused_steps=sum(len({o[3] for o in obs})>=2 for obs in step_obs),
                median_posterior_sigma_m=float(np.median([r['kalman_sigma_m'] for r in samples])),
                uncertainty_role='conditional_on_estimated_camera_and_class_priors; not_metric_ground_truth')


def bearing_conflict(xy_fit,step_obs,times):
    """Repeated transverse residuals; depth disagreement alone cannot trigger."""
    bad=[];total=0
    for i,obs in enumerate(step_obs):
        for xy,var_r,var_t,camera,cov in obs:
            if camera!='front' or cov is None:continue
            values,vectors=np.linalg.eigh(cov)
            # The smaller eigenvalue is the tightly observed angular axis.
            tangent=vectors[:,0]
            total+=1
            standardized=abs((np.asarray(xy_fit[i])-xy)@tangent)/math.sqrt(max(values[0],.02))
            if standardized>3.:bad.append(float(times[i]))
    trigger=len(bad)>=5 and len(bad)>=.2*max(total,1) and max(bad,default=0)-min(bad,default=0)>=.6
    return dict(trigger=bool(trigger),bad_direct_front_samples=len(bad),direct_front_samples=total,
                bad_time_range_s=[min(bad),max(bad)] if bad else None)
