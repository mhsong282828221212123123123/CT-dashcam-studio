"""Ground-feature/odometry calibration; no vehicle boxes or body dimensions.

Camera height is an explicit scale prior. Forward motion constrains focal*height
much more strongly than either separately; this is not surveyed calibration.
"""
import numpy as np
from scipy.optimize import least_squares

def rotation(pitch):
    c,s=np.cos(pitch),np.sin(pitch)
    return np.array([[1.,0.,0.],[0.,c,-s],[0.,s,c]])

def transfer(points,origin_a,R_a,origin_b,R_b,focal,horizon_a,horizon_b,height=1.28,width=1280,image_height=960):
    points=np.asarray(points,float);cx=width/2;cy=image_height/2
    C_a=rotation(np.arctan((cy-horizon_a)/focal));C_b=rotation(np.arctan((cy-horizon_b)/focal))
    rays=np.c_[-(points[:,0]-cx)/focal,-(points[:,1]-cy)/focal,np.ones(len(points))]@C_a.T
    scale=-height/np.minimum(rays[:,1],-1e-5)
    ground=np.array([0.,height,1.2])+rays*scale[:,None]
    world=ground[:,[0,2]]@R_a.T+origin_a
    local=(world-origin_b)@R_b
    xyz=np.c_[local[:,0],np.zeros(len(local)),local[:,1]]-np.array([0.,height,1.2])
    camera=xyz@C_b;depth=np.maximum(.1,camera[:,2])
    return np.c_[cx-focal*camera[:,0]/depth,cy-focal*camera[:,1]/depth]

def fit_ground_camera(pairs,knots,height=1.28,fixed_focal=None,horizon_reference=None):
    if len(pairs)<5:raise ValueError('Insufficient independent motion pairs')
    knots=np.asarray(knots,float);n=len(knots)
    reference=np.asarray(horizon_reference,float) if horizon_reference is not None else None
    def unpack(p):
        if reference is not None:return (float(p[0]) if fixed_focal is None else fixed_focal,reference)
        return (float(p[0]),p[1:]) if fixed_focal is None else (fixed_focal,p)
    def residual(p,heldout=False):
        focal,h=unpack(p);errors=[]
        for item in pairs:
            indices=np.arange(len(item['a']))%3==1 if heldout else np.arange(len(item['a']))%3!=1
            a,b=item['a'][indices],item['b'][indices]
            prediction=transfer(a,item['origin_a'],item['R_a'],item['origin_b'],item['R_b'],focal,
                np.interp(item['time_a'],knots,h),np.interp(item['time_b'],knots,h),height=height)
            errors.extend((prediction-b).ravel())
        if not heldout:errors.extend(np.diff(h,n=2)*.15)
        return np.asarray(errors)
    h0=np.full(n,490.)
    initial=np.r_[1300.,h0] if fixed_focal is None else h0
    low=np.r_[600.,np.full(n,400.)] if fixed_focal is None else np.full(n,400.)
    high=np.r_[2400.,np.full(n,550.)] if fixed_focal is None else np.full(n,550.)
    if reference is not None:
        initial=np.array([1300. if fixed_focal is None else fixed_focal]);low=np.array([600.]);high=np.array([2400.])
    fit=least_squares(residual,initial,bounds=(low,high),loss='soft_l1',f_scale=1.,diff_step=1e-4,max_nfev=150)
    if reference is not None and fixed_focal is not None:fit.x[0]=fixed_focal
    f,h=unpack(fit.x);errors=np.linalg.norm(residual(fit.x,True).reshape(-1,2),axis=1)
    return dict(focal_px=f,height_prior_m=height,focal_height_product_px_m=f*height,
        horizon_trace=[dict(time_s=float(t),horizon_px=float(v)) for t,v in zip(knots,h)],
        withheld_points=len(errors),withheld_median_error_px=float(np.median(errors)),
        withheld_p95_error_px=float(np.percentile(errors,95)),
        status='ground_motion_estimate_conditional_on_height_and_SEI_odometry',surveyed=False,
        height_scale_warning='Height is held fixed, not measured by this estimator.',
        solver_success=bool(fit.success),jacobian_condition=float(np.linalg.cond(fit.jac)) if np.linalg.norm(fit.jac)>0 else None)
