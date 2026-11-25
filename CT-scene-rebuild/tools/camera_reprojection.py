"""Reproject fixed-size 3D footprints into source cameras; diagnostic, not metric GT."""
import argparse,json,math
from pathlib import Path
from collections import defaultdict
import cv2,numpy as np
from ct_scene.affine_probe import camera_matrix
from ct_scene.artifacts import verify,canonical


def cube_bbox(position,yaw,dimensions,ego,ego_yaw,mount,profile):
    l,w,h=dimensions;c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s],[s,c]])
    c,s=math.cos(ego_yaw),math.sin(ego_yaw);RE=np.array([[c,-s],[s,c]])
    origin=np.asarray(ego[:2])+RE@np.asarray(mount['offset_W_ego_m'][:2]);angle=ego_yaw+math.radians(mount['yaw_deg']);c,s=math.cos(angle),math.sin(angle);RC=np.array([[c,-s],[s,c]])
    pitch=mount.get('pitch_correction_rad',0.);cp,sp=math.cos(pitch),math.sin(pitch);points=[]
    for z in (0.,h):
        for x,y in ((-l/2,-w/2),(-l/2,w/2),(l/2,w/2),(l/2,-w/2)):
            q=RC.T@(np.asarray(position[:2])+R@np.array([x,y])-origin)
            X,Y,Z=-q[1],mount['height_m']-z,q[0]
            points.append([X,cp*Y-sp*Z,sp*Y+cp*Z])
    points=np.array(points)
    if np.min(points[:,2])<=.2:return None
    K,D=camera_matrix(profile);uv=cv2.projectPoints(points,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
    if not np.isfinite(uv).all():return None
    return [float(uv[:,0].min()),float(uv[:,1].min()),float(uv[:,0].max()),float(uv[:,1].max())]


def bbox_metrics(pred,observed):
    a,b=np.array(pred),np.array(observed);pa=np.maximum(0,a[2:]-a[:2]);pb=np.maximum(0,b[2:]-b[:2]);intersection=np.prod(np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])))
    return dict(height_ratio=float(pa[1]/pb[1]),width_ratio=float(pa[0]/pb[0]),bbox_iou=float(intersection/max(np.prod(pa)+np.prod(pb)-intersection,1e-9)),centre_x_error_px=float(abs((a[0]+a[2]-b[0]-b[2])/2)))


def run(scene,profiles,mounts):
    verify(scene);report=json.loads((scene/'report.json').read_text());actors=json.loads((scene/'actors.json').read_text());states=[json.loads(x) for x in (scene/'render-states.jsonl').read_text().splitlines()];ts=np.array([s['time_s'] for s in states]);body_maps=[{a['id']:a for a in s['actors']} for s in states];records=[];counts=defaultdict(lambda:defaultdict(int))
    for actor in actors:
        key=actor.get('global_id',actor['id'])
        for row in actor['samples']:
            cam=row.get('camera_id',actor.get('camera_id'));b=row.get('bbox_xyxy')
            if cam not in profiles or cam not in mounts or row.get('support')!='detector_observation' or b is None:continue
            if min(b[2]-b[0],b[3]-b[1])<30 or min(b[0],b[1],row.get('image_width',1280)-b[2],row.get('image_height',960)-b[3])<15:continue
            counts[cam]['source_clear_observations']+=1;t=float(row['time_s']);i=int(np.argmin(abs(ts-t)))
            if abs(ts[i]-t)>.026 or key not in body_maps[i]:counts[cam]['missing_world_at_observation']+=1;continue
            body=body_maps[i][key];state=states[i];pos=np.array(body['position_W']);yaw=body['yaw_rad'];ego=np.array(state['ego_position_W']);eyaw=state['ego_yaw_rad']
            # Evaluate at the actual observation PTS rather than adding a 25ms
            # mismatch between camera image and fast-moving 20Hz render state.
            j=int(np.searchsorted(ts,t)-1);k=j+1
            if 0<=j<k<len(ts) and key in body_maps[j] and key in body_maps[k]:
                u=(t-ts[j])/(ts[k]-ts[j]);a,bod=body_maps[j][key],body_maps[k][key]
                pos=(1-u)*np.array(a['position_W'])+u*np.array(bod['position_W']);yaw=a['yaw_rad']+u*math.atan2(math.sin(bod['yaw_rad']-a['yaw_rad']),math.cos(bod['yaw_rad']-a['yaw_rad']))
                ego=(1-u)*np.array(states[j]['ego_position_W'])+u*np.array(states[k]['ego_position_W']);eyaw=states[j]['ego_yaw_rad']+u*math.atan2(math.sin(states[k]['ego_yaw_rad']-states[j]['ego_yaw_rad']),math.cos(states[k]['ego_yaw_rad']-states[j]['ego_yaw_rad']))
            mount=dict(mounts[cam]);yaw_fit=report.get('side_yaw_compensation',{}).get(cam,{})
            mount['yaw_deg']=yaw_fit.get('used_yaw_deg',mount['yaw_deg']);pitch=report.get('side_pitch_compensation',{}).get(cam,{})
            mount['pitch_correction_rad']=pitch.get('delta_pitch_rad',0.) if pitch.get('accepted') else 0.
            projected=cube_bbox(pos,yaw,report['vehicle_dimensions_m'][body['model']],ego,eyaw,mount,profiles[cam])
            if projected is None:counts[cam]['projected_body_crosses_camera_plane']+=1;continue
            counts[cam]['reprojected']+=1
            records.append(dict(id=key,source_local_track_id=row.get('local_track_id',key),camera_id=cam,time_s=t,frame_id=row['frame_id'],source_bbox=row['bbox_xyxy'],projected_bbox=projected,**bbox_metrics(projected,row['bbox_xyxy'])))
    summary={}
    for cam,counter in counts.items():
        rs=[r for r in records if r['camera_id']==cam];summary[cam]=dict(counter)
        if rs:
            summary[cam].update(median_height_ratio=float(np.median([r['height_ratio'] for r in rs])),p90_abs_log_height_ratio=float(np.percentile([abs(math.log(max(r['height_ratio'],1e-9))) for r in rs],90)),median_bbox_iou=float(np.median([r['bbox_iou'] for r in rs])),p90_centre_x_error_px=float(np.percentile([r['centre_x_error_px'] for r in rs],90)))
    return dict(status='SOURCE_REPROJECTION_DIAGNOSTIC',metric_accuracy='NOT_EVALUATED',scope='estimated-rig fixed cuboid vs original box; occlusion/class/shape errors possible; never an independent 3D accuracy score',summary=summary,records=records)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    result=run(a.scene,json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras'],json.loads(Path('configs/surrounding-prototype.json').read_text())['cameras']);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_bytes(canonical(result));print(result['summary'])
