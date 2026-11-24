"""Time-bounded rear/side identity evidence. Rear pixels never produce XYZ.

Local rear tracks can switch cars. Consequently a successful match assigns
only its supported interval, not the whole rear ID or a transitive union.
"""
from collections import defaultdict
from functools import lru_cache
from pathlib import Path
import cv2
import numpy as np


POLICY = dict(sample_period_s=.2, clock_tolerance_s=.08, minimum_inliers=6,
              minimum_inlier_ratio=.5, minimum_coverage=.015,
              minimum_body_extent_x=.12, minimum_body_extent_y=.08,
              minimum_support_frames=3, minimum_span_s=.4, maximum_gap_s=.45)


def crop_features(image, box):
    h,w=image.shape[:2]
    x,y,X,Y=np.rint(box).astype(int)
    x,y,X,Y=max(0,x),max(0,y),min(w,X),min(h,Y)
    if X-x<100 or Y-y<70:return None
    crop=image[y:Y,x:X]
    scale=min(1.,640/max(crop.shape))
    if scale<1:crop=cv2.resize(crop,None,fx=scale,fy=scale)
    h,w=crop.shape;mask=np.zeros_like(crop)
    mask[int(h*.1):int(h*.95),int(w*.05):int(w*.95)]=255
    keypoints,descriptors=cv2.SIFT_create(nfeatures=1000,contrastThreshold=.015).detectAndCompute(crop,mask)
    if descriptors is None or len(descriptors)<8:return None
    return np.float32([k.pt for k in keypoints]),descriptors,(h,w)


def feature_agreement(first,second,policy=None):
    policy={**POLICY,**(policy or {})}
    if first is None or second is None:return None
    a,da,shape_a=first;b,db,shape_b=second;matcher=cv2.BFMatcher()
    forward=[m for m,n in matcher.knnMatch(da,db,k=2) if m.distance<.75*n.distance]
    reverse={(m.trainIdx,m.queryIdx) for m,n in matcher.knnMatch(db,da,k=2) if m.distance<.75*n.distance}
    good=[];seen_a=set();seen_b=set()
    for m in sorted(forward,key=lambda m:m.distance):
        ka=tuple(np.rint(a[m.queryIdx]));kb=tuple(np.rint(b[m.trainIdx]))
        if (m.queryIdx,m.trainIdx) not in reverse or ka in seen_a or kb in seen_b:continue
        seen_a.add(ka);seen_b.add(kb);good.append(m)
    if len(good)<policy['minimum_inliers']:return None
    pa=np.float32([a[m.queryIdx] for m in good]);pb=np.float32([b[m.trainIdx] for m in good])
    cv2.setRNGSeed(20261114)
    H,mask=cv2.findHomography(pa,pb,cv2.RANSAC,3.)
    if H is None or mask is None or not np.isfinite(H).all():return None
    inside=mask.ravel().astype(bool);count=int(inside.sum());ratio=count/len(good)
    # RANSAC can return a finite matrix with zero/too few inliers. Reject that
    # hypothesis before asking OpenCV for the convex hull of an empty set.
    if count<policy['minimum_inliers'] or ratio<policy['minimum_inlier_ratio']:return None
    coverage=[float(cv2.contourArea(cv2.convexHull(p[inside]))/np.prod(shape)) for p,shape in [(pa,shape_a),(pb,shape_b)]]
    if min(coverage)<policy['minimum_coverage']:return None
    # Identical wheel designs are repeatable features even on different cars.
    # Require robust spread over the body; one background outlier must not
    # make a wheel-only cluster look like whole-vehicle identity evidence.
    extents=[((np.quantile(p[inside],.8,axis=0)-np.quantile(p[inside],.2,axis=0))/np.array([shape[1],shape[0]])).tolist() for p,shape in [(pa,shape_a),(pb,shape_b)]]
    if any(x<policy['minimum_body_extent_x'] or y<policy['minimum_body_extent_y'] for x,y in extents):return None
    return dict(inliers=count,inlier_ratio=ratio,coverage=coverage,score=count*ratio,
                robust_body_extents=extents,
                matched_points_a=pa[inside].tolist(),matched_points_b=pb[inside].tolist())


def select_reference_intervals(evidence,policy=None):
    """Exclusive matching at each timestamp, followed by repeated support.

    No extrapolation outside matched time windows; an ambiguous observation
    cannot bridge two disjoint evidence windows.
    """
    policy={**POLICY,**(policy or {})};bins=defaultdict(list)
    for e in evidence:bins[round(e['time_s'],3)].append(e)
    chosen=[]
    for rows in bins.values():
        for e in rows:
            rivals=[r for r in rows if r is not e and (r['rear_id']==e['rear_id'] or r['global_id']==e['global_id'])]
            runner=max([r['score'] for r in rivals],default=0)
            e['exclusive']=e['score']>=runner*1.3 and e['score']>runner
            if e['exclusive']:chosen.append(e)
    pairs=defaultdict(list)
    for e in chosen:pairs[e['rear_id'],e['global_id']].append(e)
    intervals=[]
    for (rear,gid),rows in pairs.items():
        rows=sorted(rows,key=lambda r:r['time_s']);runs=[]
        for e in rows:
            conflict=False
            if runs:
                previous=runs[-1][-1]['time_s']
                for t,candidates in bins.items():
                    if not previous<t<e['time_s']:continue
                    related=[r for r in candidates if r['rear_id']==rear or r['global_id']==gid]
                    if related and not any(r.get('exclusive') and r['rear_id']==rear and r['global_id']==gid for r in related):
                        conflict=True;break
            if not runs or conflict or e['time_s']-runs[-1][-1]['time_s']>policy['maximum_gap_s']:runs.append([])
            runs[-1].append(e)
        for run in runs:
            if len(run)<policy['minimum_support_frames'] or run[-1]['time_s']-run[0]['time_s']+1e-6<policy['minimum_span_s']:continue
            intervals.append(dict(rear_id=rear,global_id=gid,start_s=run[0]['time_s'],end_s=run[-1]['time_s'],
                evidence_frames=len(run),evidence=run,geometry='DISABLED',scope='matched_interval_only'))
    return intervals


def associate(package,frames,observations,actors,policy=None):
    policy={**POLICY,**(policy or {})};package=Path(package)
    by_frame={f['frame_id']:f for f in frames};by_id=defaultdict(list);by_camera=defaultdict(list)
    for f in frames:by_camera[f['camera_id']].append(f)
    for rs in by_camera.values():rs.sort(key=lambda f:f['camera_time_s'])
    membership={key:a['global_id'] for a in actors for key in a['local_ids']}
    for r in observations:
        if r['support']=='detector_observation' and r['class_id'] in (2,5,7):by_id[r['frame_id']].append(r)
    @lru_cache(maxsize=32)
    def image(fid):return cv2.imread(str(package/by_frame[fid]['image']),cv2.IMREAD_GRAYSCALE)
    evidence=[];last=-float('inf');compared=0
    for rear_frame in by_camera['back']:
        t=rear_frame['camera_time_s']
        if t-last<policy['sample_period_s']-.03:continue
        last=t;rear_rows=by_id[rear_frame['frame_id']]
        rear_features={r['local_track_id']:crop_features(image(rear_frame['frame_id']),r['bbox_xyxy']) for r in rear_rows}
        for camera in ('left_repeater','right_repeater'):
            if not by_camera[camera]:continue
            f=min(by_camera[camera],key=lambda f:abs(f['camera_time_s']-t))
            if abs(f['camera_time_s']-t)>policy['clock_tolerance_s']:continue
            for side in by_id[f['frame_id']]:
                gid=membership.get(side['local_track_id'])
                if gid is None:continue
                feature=crop_features(image(f['frame_id']),side['bbox_xyxy'])
                for rear in rear_rows:
                    compared+=1;agreement=feature_agreement(rear_features[rear['local_track_id']],feature,policy)
                    if agreement:
                        evidence.append(dict(time_s=float(t),rear_id=rear['local_track_id'],global_id=gid,
                            side_id=side['local_track_id'],rear_frame_id=rear_frame['frame_id'],side_frame_id=f['frame_id'],
                            rear_box=rear['bbox_xyxy'],side_box=side['bbox_xyxy'],**agreement))
    intervals=select_reference_intervals(evidence,policy)
    return dict(policy=policy,comparisons=compared,candidate_frames=evidence,intervals=intervals,
                rear_geometry='DISABLED',world_id_merge='DISABLED',clock_status='assumed_relative_container_alignment',
                scope='repeated local-feature identity reference; not a general vehicle ReID model')


def reference_presence(report,observations):
    """Only times explicitly inside the validated window count as reference."""
    result=defaultdict(list)
    by_id=defaultdict(list)
    for r in observations:
        if r['camera_id']=='back' and r['support']=='detector_observation':by_id[r['local_track_id']].append(r)
    for interval in report['intervals']:
        for r in by_id[interval['rear_id']]:
            if interval['start_s']<=r['camera_time_s']<=interval['end_s']:
                result[interval['global_id']].append(r['camera_time_s'])
    return dict(result)
