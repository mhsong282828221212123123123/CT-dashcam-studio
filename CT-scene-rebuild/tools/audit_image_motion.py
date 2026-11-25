import argparse,json
from pathlib import Path
from ct_scene.artifacts import verify,canonical
from ct_scene.image_motion_audit import audit

p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--lens-config',type=Path,default=Path('configs/multicam-optical-candidates.json'));p.add_argument('--prototype-config',type=Path,default=Path('configs/surrounding-prototype.json'))
a=p.parse_args();m=verify(a.scene)
if a.output.exists():raise FileExistsError(a.output)
states=[json.loads(x) for x in (a.scene/'render-states.jsonl').read_text().splitlines()];actors=json.loads((a.scene/'actors.json').read_text());r=json.loads((a.scene/'report.json').read_text())
result=audit(states,actors,json.loads(a.lens_config.read_text())['cameras']['front'],r['vehicle_dimensions_m'],json.loads(a.prototype_config.read_text())['cameras']['front']);result['artifact_id']=m['artifact_id']
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_bytes(canonical(result))
print('Front source-motion consistency:',result['direct_samples'],'samples;',result['range_size_event_count'],'review events')
print([(t['id'],t['range_size_event_count'],round(t['bearing_error_p95_px'],1)) for t in result['tracks']])
