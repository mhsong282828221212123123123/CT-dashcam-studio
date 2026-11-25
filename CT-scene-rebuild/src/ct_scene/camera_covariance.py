"""Camera-range uncertainty is rotated into the road frame, not axis-aligned."""
import numpy as np


def ray_covariance(position,ego,sigma_range,sigma_transverse,road_heading=0.):
    ray=np.asarray(position,float)[:2]-np.asarray(ego,float)[:2]
    norm=np.linalg.norm(ray)
    ray=ray/max(norm,1e-9) if norm>1e-9 else np.array([1.,0.])
    basis=np.column_stack([ray,[-ray[1],ray[0]]])
    world=basis@np.diag([sigma_range**2,sigma_transverse**2])@basis.T
    c,s=np.cos(road_heading),np.sin(road_heading);J=np.array([[c,s],[-s,c]])
    return world,J@world@J.T


def update_position(x,P,z,R):
    H=np.c_[np.eye(2),np.zeros((2,2))]
    y=np.asarray(z)-H@x;S=H@P@H.T+R
    K=np.linalg.solve(S,H@P).T
    A=np.eye(4)-K@H
    cov=A@P@A.T+K@R@K.T
    return x+K@y,(cov+cov.T)/2


def initialize_offline(samples,step_obs,first_idx,speed_cap):
    t0=float(samples[first_idx]['time_s']);rows=[]
    for i in range(first_idx,len(samples)):
        dt=float(samples[i]['time_s'])-t0
        if dt>1.2:break
        for obs in step_obs[i]:rows.append((dt,np.asarray(obs[0],float),float(obs[1]),float(obs[2])))
    if not rows:raise ValueError('No initial observations')
    t=np.array([r[0] for r in rows]);z=np.array([r[1] for r in rows]);sig=np.sqrt(np.array([[r[2],r[3]] for r in rows]))
    if len(set(t))<3 or np.ptp(t)<.25:
        z0=np.mean(z,axis=0)
        return z0,np.zeros(2),np.maximum(np.median(sig,axis=0)**2,1.)
    A=np.c_[np.ones(len(t)),t];coef=[];vars=[]
    for j in range(2):
        w=1/np.maximum(sig[:,j]**2,.04)
        # Robust full-window fit prevents two startup detections from deciding
        # an entire offline trajectory's initial velocity.
        slopes=[(z[b,j]-z[a,j])/(t[b]-t[a]) for a in range(len(t)) for b in range(a+1,len(t)) if t[b]-t[a]>=.19]
        slope=float(np.median(slopes)) if slopes else 0.
        beta=np.array([np.median(z[:,j]-slope*t),slope])
        residual=z[:,j]-A@beta
        center=np.median(residual);scale=max(.25,1.4826*np.median(abs(residual-center)))
        for _ in range(5):
            residual=z[:,j]-A@beta
            huber=np.minimum(1.,2.5*scale/np.maximum(abs(residual),1e-9));W=w*huber
            beta=np.linalg.lstsq(A*np.sqrt(W[:,None]),z[:,j]*np.sqrt(W),rcond=None)[0]
        coef.append(beta);vars.append(max(1.,float(np.median(sig[:,j]**2))))
    coef=np.asarray(coef)
    return coef[:,0],np.clip(coef[:,1],[-speed_cap,-6.],[speed_cap,6.]),np.asarray(vars)
