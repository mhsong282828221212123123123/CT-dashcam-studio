"""Held-out cross-camera projection with per-image presentation timestamps.

SEI sequence IDs identify metadata, not guaranteed simultaneous exposures.
This experiment fits a constant rig using a continuous left-camera pose trace.
The held-out left image anchors the query pose; right query pixels never fit it.
SfM scale and the vehicle-frame origin remain unmeasured.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, RotationSpline

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scene-runtime-calibration'))
import pycolmap as pc


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root',type=Path)
    ap.add_argument('--time',default='022.0.jpg')
    ap.add_argument('--ignore-pts',action='store_true')
    ap.add_argument('--fit-offset',action='store_true')
    ap.add_argument('--refine-lens',action='store_true')
    args=ap.parse_args()
    out=args.root/('timed-heldout-'+args.time.replace('.jpg','')+('-nominal' if args.ignore_pts else '')+('-offset' if args.fit_offset else '')+('-lens' if args.refine_lens else ''))
    out.mkdir(exist_ok=True)
    index={row['name']:row for row in json.loads((args.root/'input-index.json').read_text())}
    def timestamp(name):
        row=index[name]
        return row['time_s'] if args.ignore_pts else row['pts_s']
    rec=pc.Reconstruction(args.root/'sparse'/'3')
    saved={}
    for im in list(rec.images.values()):
        if im.name.endswith('/'+args.time):
            saved[im.name.split('/')[0]]=dict(name=im.name,camera=im.camera_id,
                points=[(np.array(p.xy),p.point3D_id) for p in im.points2D if p.has_point3D()])
            rec.deregister_frame(im.frame_id)
    if set(saved)!={'left_repeater','right_repeater'}:
        raise ValueError('Both held-out views required')
    opt=pc.BundleAdjustmentOptions();opt.print_summary=False
    opt.ceres.solver_options.max_num_iterations=100
    pc.bundle_adjustment(rec,opt)
    left=sorted([im for im in rec.images.values() if im.has_pose and im.name.startswith('left_repeater/')],key=lambda im:timestamp(im.name))
    times=np.array([timestamp(im.name) for im in left])
    centres=np.array([im.projection_center() for im in left])
    rotations=Rotation.from_matrix(np.array([im.cam_from_world().inverse().rotation.matrix() for im in left]))
    rs=RotationSpline(times,rotations);cs=CubicSpline(times,centres,axis=0)
    def pose_at(t):
        # world-from-camera rotation and optical centre -> camera-from-world.
        R=rs(t).as_matrix().T
        return R, -R@cs(t)
    def project(R,t,points,camera):
        return camera.img_from_cam(points@R.T+t)
    right=[im for im in rec.images.values() if im.has_pose and im.name.startswith('right_repeater/')]
    right_camera=rec.cameras[saved['right_repeater']['camera']]
    train_local=[];train_xy=[];relative=[];groups=[]
    for im in right:
        t=timestamp(im.name)
        if not times[0]<=t<=times[-1]:continue
        R,translation=pose_at(t)
        current=im.cam_from_world()
        Rrel=current.rotation.matrix()@R.T
        relative.append((Rrel,current.translation-Rrel@translation))
        group=[]
        for p in im.points2D:
            if not p.has_point3D() or p.point3D_id not in rec.points3D:continue
            point=rec.points3D[p.point3D_id]
            if point.track.length()<3:continue
            train_local.append(R@point.xyz+translation);train_xy.append(p.xy)
            group.append(point.xyz)
        groups.append((t,np.array(group)))
    train_local=np.array(train_local);train_xy=np.array(train_xy)
    init=np.r_[Rotation.from_matrix(np.array([v[0] for v in relative])).mean().as_rotvec(),np.median([v[1] for v in relative],axis=0)]
    if args.fit_offset:init=np.r_[init,0.]
    lens_index=len(init)
    original_lens=right_camera.params.copy()
    if args.refine_lens:
        if right_camera.model_name!='RADIAL':raise ValueError('Lens refinement requires RADIAL')
        init=np.r_[init,original_lens[[0,3,4]]]
    def residual(p):
        if args.refine_lens:
            params=original_lens.copy();params[[0,3,4]]=p[lens_index:lens_index+3]
            right_camera.params=params
        local=train_local
        if args.fit_offset:
            parts=[]
            for t,xyz in groups:
                R,translation=pose_at(t+p[6]);parts.append(xyz@R.T+translation)
            local=np.vstack(parts)
        uv=project(Rotation.from_rotvec(p[:3]).as_matrix(),p[3:6],local,right_camera)
        return np.nan_to_num(uv-train_xy,nan=1e4,posinf=1e4,neginf=-1e4).ravel()
    bounds=(np.r_[np.full(6,-np.inf),-.12],np.r_[np.full(6,np.inf),.12]) if args.fit_offset else (-np.inf,np.inf)
    if args.refine_lens:
        lo=np.full(lens_index,-np.inf);hi=np.full(lens_index,np.inf)
        if args.fit_offset:lo[6]=-.12;hi[6]=.12
        bounds=(np.r_[lo,.85*original_lens[0],-1.,-1.],np.r_[hi,1.15*original_lens[0],1.,1.])
    fit=least_squares(residual,init,loss='soft_l1',f_scale=1.,max_nfev=100,bounds=bounds,x_scale='jac')
    residual(fit.x)
    relative_R=Rotation.from_rotvec(fit.x[:3]).as_matrix();relative_t=fit.x[3:6]
    pairs={}
    for key,entry in saved.items():
        rows=[(xy,rec.points3D[pid].xyz) for xy,pid in entry['points']
              if pid in rec.points3D and rec.points3D[pid].track.length()>=3]
        pairs[key]=tuple(map(np.array,zip(*rows)))
    estimation=pc.AbsolutePoseEstimationOptions();estimation.ransac.max_error=3.
    estimation.ransac.random_seed=123
    refinement=pc.AbsolutePoseRefinementOptions();refinement.gradient_tolerance=1e-10
    left_camera=rec.cameras[saved['left_repeater']['camera']]
    pose=pc.estimate_and_refine_absolute_pose(*pairs['left_repeater'],left_camera,estimation,refinement)
    if pose is None:raise RuntimeError('Left localization failed')
    observed_left=pose['cam_from_world']
    left_xy,left_xyz=pairs['left_repeater']
    left_error=np.linalg.norm(project(observed_left.rotation.matrix(),observed_left.translation,left_xyz,left_camera)-left_xy,axis=1)
    left_time=timestamp(saved['left_repeater']['name']);right_time=timestamp(saved['right_repeater']['name'])+(fit.x[6] if args.fit_offset else 0.)
    a,ta=pose_at(left_time);b,tb=pose_at(right_time)
    delta_R=b@a.T;delta_t=tb-delta_R@ta
    query_R=delta_R@observed_left.rotation.matrix()
    query_t=delta_R@observed_left.translation+delta_t
    predicted_R=relative_R@query_R;predicted_t=relative_R@query_t+relative_t
    observed,xyz=pairs['right_repeater']
    predicted=project(predicted_R,predicted_t,xyz,right_camera)
    error=np.linalg.norm(predicted-observed,axis=1)
    training=np.linalg.norm(residual(fit.x).reshape(-1,2),axis=1)
    report=dict(timestamp=args.time,ignore_pts=args.ignore_pts,
        estimated_clock_offset_ms=float(fit.x[6]*1000) if args.fit_offset else 0.,
        right_camera_params=right_camera.params.tolist(),lens_refined_in_rig_fit=args.refine_lens,
        left_localization_inliers=int(pose['num_inliers']),
        left_localization_median_px=float(np.median(left_error)),
        left_localization_p95_px=float(np.percentile(left_error,95)),
        container_timestamp_delta_ms=(timestamp(saved['right_repeater']['name'])-left_time)*1000,
        applied_timestamp_delta_ms=(right_time-left_time)*1000,
        training_points=len(training),training_median_px=float(np.median(training)),
        training_p95_px=float(np.percentile(training,95)),test_points=len(error),
        test_median_px=float(np.median(error)),test_p95_px=float(np.percentile(error,95)),
        test_max_px=float(error.max()),relative_rotation=relative_R.tolist(),
        relative_translation_arbitrary_units=relative_t.tolist(),
        accepted=bool(fit.success and np.isfinite(error).all()
                      and (not args.fit_offset or abs(fit.x[6])<.119)
                      and np.median(error)<=3 and np.percentile(error,95)<=8),
        metric_calibration_verified=False,optimizer_success=bool(fit.success))
    (out/'validation.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    np.savez_compressed(out/'projections.npz',observed=observed,predicted=predicted)
    image=cv2.imdecode(np.fromfile(args.root/'images'/saved['right_repeater']['name'],np.uint8),cv2.IMREAD_COLOR)
    for obs,pred in zip(observed,predicted):
        if not np.isfinite(pred).all():continue
        a,b=tuple(np.rint(obs).astype(int)),tuple(np.rint(pred).astype(int))
        cv2.circle(image,a,4,(0,220,0),1,cv2.LINE_AA)
        cv2.drawMarker(image,b,(0,100,255),cv2.MARKER_CROSS,7,1)
    cv2.rectangle(image,(0,0),(1280,70),(20,20,20),-1)
    cv2.putText(image,f'HELD-OUT RIGHT | {args.time} | PTS compensation: {not args.ignore_pts}',(15,25),0,.7,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(image,f'median {np.median(error):.2f}px | p95 {np.percentile(error,95):.2f}px | NOT METRIC CALIBRATED',(15,55),0,.6,(255,255,255),1,cv2.LINE_AA)
    cv2.imencode('.jpg',image)[1].tofile(out/'proof.jpg')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
