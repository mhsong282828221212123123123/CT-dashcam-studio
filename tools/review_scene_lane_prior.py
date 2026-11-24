"""Replay road evidence and containment on an existing source-reviewed timeline."""
import copy
import json
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_vehicle_road import add_vehicle_corridors
from core.scene_lane_prior import apply_lane_containment_prior
from core.scene_ego_turn import apply_ego_turns
from core.driving_scene import DrivingRenderer
from tools.validate_single_clip_reference import motion_outliers


if __name__=='__main__':
    root=Path(sys.argv[1]);out=root/'lane-prior-review';out.mkdir(exist_ok=True)
    with (root/'current.pkl').open('rb') as f:t=pickle.load(f)
    original_poses=copy.deepcopy(t.ego_poses)
    original_display=[copy.deepcopy(getattr(s,'ego_display_override',None)) for s in t.scenes]
    before_counts=[a.get('travel_corridors',{}).get('display_lane_count') for a in t.audit]
    old_outliers=motion_outliers(t)
    for s in t.scenes:s.lanes=copy.deepcopy(getattr(s,'observed_lane_segments',[]))
    add_vehicle_corridors(t.scenes,t.ego_poses,t.audit)
    apply_ego_turns(t.scenes,t.ego_poses,t.audit)
    t.lane_prior_corrections=apply_lane_containment_prior(t.scenes,t.audit,t.rows,t.camera_frames)
    outliers=motion_outliers(t)
    assert all(np.array_equal(a[0],b[0]) and np.array_equal(a[1],b[1]) for a,b in zip(original_poses,t.ego_poses))
    assert all(np.allclose(a,s.ego_display_override,atol=1e-12) for a,s in zip(original_display,t.scenes))
    report=dict(road_counts_before=sorted(set(v for v in before_counts if v is not None)),
        road_counts_after=sorted(set(a['travel_corridors']['display_lane_count'] for a in t.audit)),
        corrected_actor_samples=t.lane_prior_corrections,
        corrected_sources=sorted({s for scene,audit in zip(t.scenes,t.audit) for fix in audit['lane_containment_prior']
                                 for actor in scene.actors if actor.key==fix['id'] for s in actor.sources}),
        max_correction_m=max((abs(r['road_lateral_correction_m']) for a in t.audit for r in a['lane_containment_prior']),default=0.),
        motion_outliers_before=len(old_outliers),motion_outliers_after=len(outliers),
        host_odometry_unchanged=True,metric_accuracy_verified=False)
    report['host_display_pose_unchanged']=True
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    (out/'motion-outliers.json').write_text(json.dumps(outliers,indent=2),encoding='utf8')
    (out/'audit.json').write_text(json.dumps(t.audit,indent=2),encoding='utf8')
    with (out/'timeline.pkl').open('wb') as f:pickle.dump(t,f)
    renderer=DrivingRenderer(720,480);images=[]
    for time in (2.,12.,24.,34.):
        image=renderer.render(t.sample(time));cv2.putText(image,f'SOURCE {time:.1f}s',(15,30),0,.7,(30,50,170),2)
        images.append(image)
    cv2.imencode('.jpg',np.vstack([np.hstack(images[:2]),np.hstack(images[2:])]))[1].tofile(out/'proof.jpg')
    print(json.dumps(report,indent=2),flush=True)
