"""Known-size plate-plane reference; not a surveyed vehicle-body origin.

No OCR. The user clip stays local. Corners are explicit manual references,
tracked from the reference image with an independently checked image transform.
The 520x110 mm plate-type assumption must be visually checked before reuse.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import CameraReader


def main():
    root=Path(__file__).resolve().parents[2]/'scene-results'/'camera-zero-review'
    out=root/'plate-anchor';out.mkdir(exist_ok=True)
    original=root/'easy-queue-front-12s.jpg'
    reference=cv2.imdecode(np.fromfile(original,np.uint8),cv2.IMREAD_COLOR)
    gray=cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY)
    corners=np.array([[582.5,568.8],[708.,569.],[708.1,594.6],[582.1,594.3]],np.float64)
    obj=np.array([[-.26,-.055,0],[.26,-.055,0],[.26,.055,0],[-.26,.055,0]],np.float64)
    fitted=json.loads((root/'rig-sfm'/'front-heldout-validation.json').read_text())[0]['params']
    K=np.array([[fitted[0],0,640],[0,fitted[0],480],[0,0,1]],np.float64)
    D=np.array([fitted[3],0,0,0,0],np.float64)
    legacyK=K.copy();legacyK[0,0]=legacyK[1,1]=850.
    def solve(xy,K=K,D=D):
        ok,rv,tv=cv2.solvePnP(obj,xy,K,D,flags=cv2.SOLVEPNP_IPPE)
        if not ok or tv[2,0]<=0:raise ValueError('Invalid plate pose')
        reproj=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        return rv,tv.ravel(),np.linalg.norm(reproj-xy,axis=1)
    rv,tv,reprojection=solve(corners)
    legacy=solve(corners,legacyK,np.zeros(5))[1]
    mask=np.zeros_like(gray);mask[563:602,576:715]=255
    points=cv2.goodFeaturesToTrack(gray,200,.01,3,mask=mask)
    if points is None or len(points)<24:raise RuntimeError('Not enough plate texture')
    source=Path('E:/tesla/a4/TeslaCam/SentryClips/2026-10-15_17-39-27/2026-10-15_17-33-13-front.mp4')
    reader=CameraReader(source);rows=[];frames=[]
    try:
        for t in np.arange(12.,25.,1.):
            idx,image=reader.read_at(float(t))
            if image is None:raise RuntimeError('Missing original frame')
            current=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            moved,ok,_=cv2.calcOpticalFlowPyrLK(gray,current,points,None,winSize=(21,21),maxLevel=3)
            back,ok2,_=cv2.calcOpticalFlowPyrLK(current,gray,moved,None,winSize=(21,21),maxLevel=3)
            valid=(ok.ravel()!=0)&(ok2.ravel()!=0)&(np.linalg.norm(back-points,axis=2).ravel()<.5)
            a,b=points.reshape(-1,2)[valid],moved.reshape(-1,2)[valid]
            train=np.arange(len(a))%3!=1
            H,inliers=cv2.findHomography(a[train],b[train],cv2.RANSAC,.75)
            if H is None or (~train).sum()<8:raise RuntimeError('Unverified plate tracking')
            pred=cv2.perspectiveTransform(a[~train][None],H)[0]
            tracking_error=np.linalg.norm(pred-b[~train],axis=1)
            tracked=cv2.perspectiveTransform(corners[None],H)[0]
            rvec,translation,error=solve(tracked)
            reliable=bool(np.percentile(tracking_error,95)<=1.5)
            rows.append(dict(time_s=float(t),frame_index=idx,plate_camera_xyz_m=translation.tolist(),
                corners=tracked.tolist(),tracking_test_points=int((~train).sum()),
                tracking_p95_px=float(np.percentile(tracking_error,95)),
                pose_fit_error_px=float(np.mean(error)),tracking_accepted=reliable))
            if t in (12.,18.,24.):
                rendered=image.copy()
                cv2.polylines(rendered,[np.rint(tracked).astype(np.int32)],True,(0,230,0),2,cv2.LINE_AA)
                cv2.drawFrameAxes(rendered,K,D,rvec,translation,.3,2)
                cv2.rectangle(rendered,(0,0),(1280,75),(20,20,20),-1)
                cv2.putText(rendered,f'PLATE REFERENCE | {t:.1f}s | camera-relative depth {translation[2]:.2f} m',(15,28),0,.75,(255,255,255),1,cv2.LINE_AA)
                cv2.putText(rendered,'520x110 mm type assumed | inferred lens | NOT surveyed body/road coordinates',(15,59),0,.6,(255,255,255),1,cv2.LINE_AA)
                frames.append(cv2.resize(rendered,(768,576)))
    finally:reader.close()
    values=np.array([r['plate_camera_xyz_m'] for r in rows if r['tracking_accepted']])
    rng=np.random.default_rng(2026);depths=[]
    for _ in range(400):
        # Sensitivity only: these distributions are assumed, not measured noise.
        jitter=corners+rng.normal(0,.5,(4,2))
        sampleK=K.copy();sampleK[0,0]*=1+rng.normal(0,.02);sampleK[1,1]=sampleK[0,0]
        depths.append(solve(jitter,sampleK,D)[1][2])
    report=dict(source=str(source),reference=str(original),reference_corners=corners.tolist(),
        plate_dimensions_m=[.520,.110],dimension_source='https://www.law.go.kr/admRulLsInfoP.do?admRulSeq=2100000256300',
        dimension_status='standard plate type visually assumed; not personally measured',
        K=K.tolist(),D=D.tolist(),camera_xyz_m=tv.tolist(),legacy_850_camera_xyz_m=legacy.tolist(),
        query_count=len(rows),tracking_accepted=sum(r['tracking_accepted'] for r in rows),
        median_depth_m=float(np.median(values[:,2])),depth_span_m=float(np.ptp(values[:,2])),
        assumed_noise_depth_interval_m=np.percentile(depths,[2.5,97.5]).tolist(),
        uncertainty_warning='Sensitivity under assumed corner/focal noise. Not an absolute accuracy guarantee; lens bias, plate boundary/type and mounting are not surveyed.',
        vehicle_body_origin_verified=False,road_plane_verified=False,
        rows=rows)
    (out/'validation.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    cv2.imencode('.jpg',np.vstack(frames))[1].tofile(out/'reference-proof.jpg')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))


if __name__=='__main__':main()
