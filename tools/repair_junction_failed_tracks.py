"""Retry rejected ground fits as explicitly uncalibrated bearing/size ghosts."""
import json,pickle,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_bundle_refinement import refine_bundles,reject_unestablished_outliers
from tools.audit_scene_motion import audit_motion
from tools.validate_single_clip_reference import motion_outliers

if __name__=='__main__':
 root=Path(sys.argv[1]);t=pickle.load((root/'reviewed.pkl').open('rb'))
 failed={r['source'] for r in t.motion_bundle_report if not (r.get('accepted') or r.get('occlusion_hypothesis'))}
 def progress(s):print(s['source'],'bearing ghost',s.get('projection_hypothesis'),flush=True)
 reports=refine_bundles(t.scenes,t.rows,t.ego_poses,t.camera_frames,t.audit,
     source_filter=failed,bearing_only=True,progress=progress)
 t.motion_bundle_report=[r for r in t.motion_bundle_report if r['source'] not in failed]+reports
 t.motion_abstentions+=reject_unestablished_outliers(t.scenes,t.rows,t.ego_poses,t.audit)
 for scene,row,audit in zip(t.scenes,t.rows,t.audit):
  beyond={o['track_id'] for d in row['cameras'].values() for o in d.get('objects',[]) if o.get('road_relation')=='beyond_central_boundary'}
  scene.actors=[a for a in scene.actors if not a.sources or not set(a.sources)<=beyond]
  audit['visible_ids']=[a.key for a in scene.actors]
  audit['world_tracks']=[dict(id=a.key,position=[a.x,a.z],state=a.tracking_state,sources=a.sources,image_observed=a.image_observed) for a in scene.actors]
 pickle.dump(t,(root/'final.pkl').open('wb'))
 for k,v in [('audit',audit_motion(t)),('models',t.motion_bundle_report),('motion-outliers',motion_outliers(t)),('abstentions',t.motion_abstentions)]:
  (root/f'final-{k}.json').write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf8')
 print('DONE',root,flush=True)
