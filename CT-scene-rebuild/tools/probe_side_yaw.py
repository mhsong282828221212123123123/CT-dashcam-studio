"""Side-view axis consistency probe; camera yaw from straight relative tracks."""
import math,json
from pathlib import Path
import cv2,numpy as np
from ct_scene.affine_probe import camera_matrix


def track_axis_candidates(rows,profile,ego_times,ego_yaws,seed_deg,vehicle_height=1.5):
    K,D=camera_matrix(profile);rs=[]
    yaw_rate=np.gradient(np.unwrap(ego_yaws),ego_times)
    for r in rows:
        b=r.get('bbox_xyxy');t=float(r.get('time_s',r.get('camera_time_s')))
        if r.get('support')!='detector_observation' or r.get('score',1.)<.55 or 'truncated' in r.get('quality_flags',[]) or b is None or min(b[2]-b[0],b[3]-b[1])<35 or min(b[0],b[1],1280-b[2],960-b[3])<15:continue
        if abs(np.interp(t,ego_times,yaw_rate))>math.radians(1.5):continue
        x=(b[0]+b[2])/2;rays=cv2.undistortPoints(np.array([[[x,b[1]]],[[x,b[3]]]],float),K,D).reshape(2,2);dh=rays[1,1]-rays[0,1]
        if dh<=.02:continue
        z=vehicle_height/dh
        if not 4<z<65:continue
        rs.append((t,z,-z*float(np.mean(rays[:,0]))))
    if len(rs)<10:return []
    data=np.array(rs);results=[]
    for start in np.arange(data[0,0],data[-1,0]-2.4,2.):
        win=data[(data[:,0]>=start)&(data[:,0]<start+4)]
        if len(win)<10 or np.ptp(win[:,0])<2 or np.max(np.diff(win[:,0]))>.8:continue
        ys=np.interp(win[:,0],ego_times,np.unwrap(ego_yaws))
        if np.ptp(ys)>math.radians(3):continue
        q=win[:,1:];_,sv,vh=np.linalg.svd(q-np.mean(q,axis=0),full_matrices=False);axis=vh[0];along=(q-q.mean(0))@axis;cross=(q-q.mean(0))@vh[1]
        span=float(np.ptp(along));noise=float(np.percentile(abs(cross),90))
        if span<4 or noise>max(.45,.08*span):continue
        phi=math.atan2(axis[1],axis[0]);yaw=-phi
        seed=math.radians(seed_deg);yaw=seed+(yaw-seed+math.pi/2)%math.pi-math.pi/2
        if abs(yaw-seed)>math.radians(30):continue
        results.append(dict(start_s=float(start),yaw_deg=math.degrees(yaw),span_m=span,noise_p90_m=noise))
    return results


if __name__=='__main__':
    profiles=json.loads(Path('configs/multicam-optical-candidates.json').read_text())['cameras'];cfg=json.loads(Path('configs/surrounding-prototype.json').read_text())['cameras'];all_out={}
    for case,folder in [('aug28','midnight-aug28-012'),('highway','midnight-highway-011'),('junction','midnight-junction-011'),('oct22','midnight-oct22-011')]:
        p=Path('runs')/folder;actors=json.loads((p/'actors.json').read_text());dims=json.loads((p/'report.json').read_text())['vehicle_dimensions_m'];ss=[json.loads(x) for x in (p/'render-states.jsonl').read_text().splitlines()];ts=np.array([s['time_s'] for s in ss]);ys=np.array([s['ego_yaw_rad'] for s in ss]);all_out[case]={}
        for cam in ['left_repeater','right_repeater']:
            out=[]
            for a in actors:
                rows=[r for r in a['samples'] if r.get('camera_id')==cam]
                windows=track_axis_candidates(rows,profiles[cam],ts,ys,cfg[cam]['yaw_deg'],dims[a['model']][2])
                if windows:out.append(dict(track_id=a['global_id'],median_yaw_deg=float(np.median([r['yaw_deg'] for r in windows])),windows=windows))
            all_out[case][cam]=out;print(case,cam,[(r['track_id'],round(r['median_yaw_deg'],1),len(r['windows'])) for r in out])
            if out:print('BALANCED YAW',float(np.median([r['median_yaw_deg'] for r in out])))
    Path('runs/session-20261126-two-hours/side-yaw-probe.json').write_text(json.dumps(all_out,indent=2))
