"""Raw background correspondence candidates, not calibrated poses or static truth."""
from pathlib import Path
import io
import json
import cv2
import numpy as np
from PIL import Image,ImageDraw
from .artifacts import canonical,digest,publish,verify


def feature_roi(mask):
    """Conservative provisional image ROI to suppress ego body/mirror/sky edges."""
    mask=mask.copy();h,w=mask.shape
    roi=np.zeros_like(mask)
    roi[round(h*.25):round(h*.9),round(w*.08):round(w*.92)]=255
    return cv2.bitwise_and(mask,roi)


def optical_feature_roi(mask,camera_id):
    """Image-body exclusions for calibration; do not discard upper facade detail.

    Provisional HW3 image regions, not factory calibration or a road mask.
    Original feature_roi remains unchanged for reproducibility of older runs.
    """
    h,w=mask.shape;roi=np.zeros_like(mask)
    if camera_id=='front':
        roi[round(h*.04):round(h*.9),round(w*.02):round(w*.98)]=255
    elif camera_id in ('left_repeater','right_repeater'):
        roi[round(h*.04):round(h*.88),round(w*.03):round(w*.97)]=255
        if camera_id=='left_repeater':roi[:round(h*.29),:round(w*.43)]=0
        else:roi[:round(h*.29),round(w*.57):]=0
    else:raise ValueError('Optical ROI camera unsupported')
    return cv2.bitwise_and(mask,roi)


def correspondences(a,b,mask_a,mask_b):
    points=cv2.goodFeaturesToTrack(a,800,.01,8,mask=feature_roi(mask_a),blockSize=7)
    if points is None:return np.empty((0,4),np.float32),{'reason':'no_features'}
    q,s,_=cv2.calcOpticalFlowPyrLK(a,b,points,None,winSize=(25,25),maxLevel=4)
    if q is None:return np.empty((0,4),np.float32),{'reason':'flow_failed'}
    back,bs,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,winSize=(25,25),maxLevel=4)
    if back is None:return np.empty((0,4),np.float32),{'reason':'reverse_flow_failed'}
    p,q,back=points.reshape(-1,2),q.reshape(-1,2),back.reshape(-1,2)
    h,w=b.shape
    good=(s.ravel()==1)&(bs.ravel()==1)&np.isfinite(q).all(1)&(np.linalg.norm(back-p,axis=1)<1.5)
    inside=(q[:,0]>=0)&(q[:,0]<w)&(q[:,1]>=0)&(q[:,1]<h)
    good &= inside
    ids=np.where(good)[0];rounded=np.rint(q[ids]).astype(int);rounded[:,0]=np.clip(rounded[:,0],0,w-1);rounded[:,1]=np.clip(rounded[:,1],0,h-1)
    keep=feature_roi(mask_b)[rounded[:,1],rounded[:,0]]>0
    ids=ids[keep];matches=np.column_stack([p[ids],q[ids]]).astype(np.float32)
    displacement=np.linalg.norm(matches[:,2:]-matches[:,:2],axis=1)
    cells={(int(x/w*4),int(y/h*3)) for x,y in matches[:,:2]}
    median=float(np.median(displacement)) if len(displacement) else None
    return matches,{'candidate_matches':len(matches),'occupied_cells_4x3':len(cells),'median_flow_px':median,
                    'low_motion':median is None or median<.25,
                    'reason':'candidates_only_no_epipolar_or_static_validation'}


def run(package,masks,destination):
    package,masks=Path(package),Path(masks)
    pm,mm=verify(package),verify(masks)
    if not any(i['artifact_id']==pm['artifact_id'] for i in mm['inputs']):raise ValueError('Mask/source artifact mismatch')
    fs=json.loads((package/'frames.json').read_text(encoding='utf-8'))
    obs={r['frame_id']:r for r in map(json.loads,(masks/'observations.jsonl').read_text().splitlines())}
    payloads={};pairs=[];arrays=[];offset=0;proofs={};resolutions={};cv2.setNumThreads(2)
    for camera in ['front','left_repeater','right_repeater']:
        sequence=sorted((f for f in fs if f['camera_id']==camera),key=lambda f:f['camera_time_s'])
        previous=None
        for i,f in enumerate(sequence):
            image=np.asarray(Image.open(package/f['image']).convert('L'))
            if image.shape!=(f['height'],f['width']):raise ValueError('Image/metadata resolution mismatch')
            resolution=[f['width'],f['height']]
            if camera in resolutions and resolutions[camera]!=resolution:raise ValueError('Camera resolution changed within artifact')
            resolutions[camera]=resolution
            mask=np.asarray(Image.open(masks/obs[f['frame_id']]['static_mask']))
            if previous is not None:
                pf,a,ma=previous
                if np.mean(mask==0)>.8 or np.mean(ma==0)>.8:
                    matches=np.empty((0,4),np.float32);diag={'reason':'excessive_mask_no_geometry'}
                else:matches,diag=correspondences(a,image,ma,mask)
                pairs.append(dict(camera_id=camera,from_frame_id=pf['frame_id'],to_frame_id=f['frame_id'],
                                  from_camera_time_s=pf['camera_time_s'],to_camera_time_s=f['camera_time_s'],
                                  array_offset=offset,array_count=len(matches),space_id='raw',**diag))
                arrays.append(matches);offset+=len(matches)
                if i in [10,75,165]:proofs[(camera,i)]=(f,matches)
            previous=(f,image,mask)
    buffer=io.BytesIO();np.savez_compressed(buffer,matches=np.concatenate(arrays) if arrays else np.empty((0,4)))
    payloads['matches.npz']=buffer.getvalue();payloads['pairs.jsonl']=b''.join(canonical(r)+b'\n' for r in pairs)
    proof=Image.new('RGB',(3*384,3*316),(20,20,20));draw=ImageDraw.Draw(proof)
    for col,camera in enumerate(['front','left_repeater','right_repeater']):
        for row,index in enumerate([10,75,165]):
            if (camera,index) not in proofs:continue
            f,points=proofs[camera,index];im=Image.open(package/f['image']).convert('RGB');d=ImageDraw.Draw(im)
            for x1,y1,x2,y2 in points:d.line([x1,y1,x2,y2],fill='yellow',width=2);d.ellipse([x2-2,y2-2,x2+2,y2+2],fill='lime')
            im.thumbnail((384,288));x,y=col*384,row*316;proof.paste(im,(x,y+25));draw.text((x+3,y+3),f'{camera} t={f["relative_request_s"]:.1f} n={len(points)}',fill='white')
    buf=io.BytesIO();proof.save(buf,format='JPEG',quality=90);payloads['proof.jpg']=buf.getvalue()
    report={'pairs':len(pairs),'candidate_correspondences':offset,'rear_geometry':'disabled_unverified_distortion',
            'resolutions_by_camera':resolutions,
            'intrinsics':'NOT_ESTIMATED','rig':'NOT_ESTIMATED','ego_pose':'NOT_ESTIMATED',
            'warning':'FB-consistent masked feature candidates are not verified static landmarks. Pixel ROI is provisional.',
            'by_camera':{c:{'pairs':sum(r['camera_id']==c for r in pairs),'low_motion_pairs':sum(r['camera_id']==c and r.get('low_motion',False) for r in pairs)} for c in ['front','left_repeater','right_repeater']}}
    payloads['report.json']=canonical(report)
    return publish(destination,stage='static_feature_candidates',payloads=payloads,
                   inputs=[{'role':'raw_package','artifact_id':pm['artifact_id']},{'role':'dynamic_masks','artifact_id':mm['artifact_id']}],
                   config={'roi_fraction':[.08,.25,.92,.9],'fb_error_px':1.5,'max_points_per_pair':800},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'opencv':cv2.__version__})
