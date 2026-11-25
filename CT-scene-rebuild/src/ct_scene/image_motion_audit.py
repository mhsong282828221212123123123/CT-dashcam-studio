"""Source-image consistency diagnostics, never a surveyed metric accuracy score.

For a rear/front-facing rigid vehicle, apparent height should vary inversely
with optical depth. Comparing log changes cancels the unknown physical height.
Turning, occlusion and detector jitter can also violate this relationship; events
require original-frame review and must not automatically move or hide a car.
"""
import math
import cv2
import numpy as np
from .affine_probe import camera_matrix


def audit(states,actors,profile,dimensions,mount):
    K,D=camera_matrix(profile);times=np.array([s['time_s'] for s in states]);tracks=[]
    by_time=[{a['id']:a for a in s['actors']} for s in states]
    for actor in actors:
        key=actor.get('global_id',actor.get('id'));rows=[]
        for sample in actor['samples']:
            if sample.get('camera_id',actor.get('camera_id'))!='front' or sample.get('support')!='detector_observation':continue
            b=sample.get('bbox_xyxy')
            if not b or min(b[2]-b[0],b[3]-b[1])<20 or (b[2]-b[0])/(b[3]-b[1])>2.2:continue
            if min(b[0],b[1],sample.get('image_width',1280)-b[2],sample.get('image_height',960)-b[3])<8:continue
            t=float(sample['time_s']);i=int(np.argmin(abs(times-t)))
            if abs(times[i]-t)>.026 or key not in by_time[i]:continue
            a=by_time[i][key];s=states[i]
            if a.get('ghost') or not a.get('source_visible',True):continue
            yaw=float(s['ego_yaw_rad']);c,sn=math.cos(yaw),math.sin(yaw)
            R=np.array([[c,-sn,0],[sn,c,0],[0,0,1]])
            local=R.T@(np.asarray(a['position_W'])-s['ego_position_W'])-np.asarray(mount.get('offset_W_ego_m',[0,0,0]))
            my=math.radians(mount.get('yaw_deg',0));cm,sm=math.cos(my),math.sin(my)
            local=np.array([[cm,sm,0],[-sm,cm,0],[0,0,1]])@local
            depth=local[0]-float(dimensions[a['model']][0])/2
            if depth<10:continue
            cx=(b[0]+b[2])/2;rays=cv2.undistortPoints(np.array([[[cx,b[1]]],[[cx,b[3]]]],float),K,D).reshape(2,2)
            h=float(rays[1,1]-rays[0,1])
            if h<=0:continue
            bearing_error_px=abs(-local[1]/max(local[0],1)-float(np.mean(rays[:,0])))*K[0,0]
            rows.append(dict(time_s=t,render_time_s=float(times[i]),optical_depth_m=float(depth),angular_height=h,bearing_error_px=float(bearing_error_px)))
        rows=sorted({r['time_s']:r for r in rows}.values(),key=lambda r:r['time_s'])
        events=[]
        for i,a in enumerate(rows):
            candidates=[b for b in rows[i+1:] if .8<=b['time_s']-a['time_s']<=1.2]
            if not candidates:continue
            b=min(candidates,key=lambda b:abs(b['time_s']-a['time_s']-1))
            log_depth=math.log(b['optical_depth_m']/a['optical_depth_m']);log_size=math.log(b['angular_height']/a['angular_height'])
            residual=log_depth+log_size
            if abs(residual)>.18:
                events.append(dict(start_s=a['time_s'],end_s=b['time_s'],log_range_size_residual=residual,
                    depth_ratio=b['optical_depth_m']/a['optical_depth_m'],inverse_size_ratio=a['angular_height']/b['angular_height'],
                    reason='range_change_not_explained_by_apparent_body_height; inspect_turn_occlusion_detector'))
        if rows:
            tracks.append(dict(id=key,direct_samples=len(rows),range_size_event_count=len(events),events=events,
                bearing_error_p95_px=float(np.percentile([r['bearing_error_px'] for r in rows],95)),samples=rows))
    return dict(status='DIAGNOSTIC_REQUIRES_SOURCE_REVIEW',metric_accuracy='NOT_EVALUATED',
        role='scale_independent_front_body_size_motion_consistency; not source ground-truth or automatic correction',
        front_tracks=len(tracks),direct_samples=sum(t['direct_samples'] for t in tracks),
        range_size_event_count=sum(t['range_size_event_count'] for t in tracks),tracks=tracks)
