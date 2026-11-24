"""Build a new clip without importing another clip's reviewed vehicle labels."""
import json,pickle,sys,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline
from core.scene_map import load_road_map
from tools.audit_scene_motion import audit_motion
from tools.validate_single_clip_reference import motion_outliers
from tools.review_scene_sources import review
from tools.prepare_scene_osm_api import prepare

if __name__=='__main__':
    root=Path(sys.argv[1]);derived=root/'reviewed-input';tag=sys.argv[2] if len(sys.argv)>2 else 'initial'
    if not derived.exists():shutil.copytree(root/'analysis',derived)
    m=json.loads((derived/'manifest.json').read_text(encoding='utf8'))
    m['motion_bundle_refinement']=True;m.setdefault('review_assisted',False)
    try:
        prepare(derived,root/'roads.json');m['road_map']=load_road_map(root/'roads.json');m['osm']='OSM API outline reference; no lane snapping'
    except Exception as exc:m['osm']='unavailable: '+str(exc);print('MAP',repr(exc),flush=True)
    (derived/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf8')
    rows=[json.loads(x) for x in (derived/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    if not (root/'source-review/identity-index.json').exists():print('SOURCE REVIEW',review(derived,root/'source-review'),flush=True)
    def progress(s):print(s['source'],s.get('accepted'),s.get('occlusion_hypothesis'),flush=True)
    t=FusedSceneTimeline(rows,m,motion_progress=progress)
    pickle.dump(t,(root/(tag+'.pkl')).open('wb'))
    for k,v in [('audit',audit_motion(t)),('models',t.motion_bundle_report),('motion-outliers',motion_outliers(t)),('abstentions',t.motion_abstentions)]:
        (root/f'{tag}-{k}.json').write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf8')
    print('DONE',root,flush=True)
