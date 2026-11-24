"""Read actual renderer MVP and distinguish world presence from frustum scope.

Bounding-box plane intersection is conservative. It does not prove rasterized
pixels or absence of occlusion; those still require image review.
"""
import argparse
from collections import defaultdict
from itertools import product
import json
from pathlib import Path
import numpy as np
from ct_scene.artifacts import canonical,verify
from ct_scene.prototype_front import Renderer,model_vertices,rotation

p=argparse.ArgumentParser()
for name in ('scene','config','assets-root','output'):p.add_argument('--'+name,type=Path,required=True)
a=p.parse_args();m=verify(a.scene);cfg=json.loads(a.config.read_text())['render_style']
renderer=Renderer(a.assets_root/'tesla_fsd_sedan.glb',a.assets_root/'tesla_fsd_sedan.glb',
                  cfg.get('camera_eye',[-30.,0.,30.]),cfg.get('camera_target',[0.,0.,0.]),camera_zoom=cfg.get('camera_zoom',1.3))
mvp=np.frombuffer(renderer.program['mvp'].read(),np.float32).reshape(4,4).T
models={};names={'truck':'small_truck'};counts=defaultdict(lambda:defaultdict(int));rows=[]
actor_models={x['global_id']:x['model'] for x in json.loads((a.scene/'actors.json').read_text())}
for line in (a.scene/'render-states.jsonl').read_text().splitlines():
    s=json.loads(line);view=s['camera_view'];eye=np.array(view['position_W']);yaw=view['yaw_rad']
    for actor in s['actors']:
        kind=actor.get('model',actor_models[actor['id']])
        if kind not in models:
            v=model_vertices(a.assets_root/('tesla_fsd_'+names.get(kind,kind)+'.glb'))[:,:3]
            models[kind]=np.array(list(product(*zip(v.min(0),v.max(0)))))
        relative=rotation(-yaw)@(np.array(actor['position_W'])-eye)
        vertices=models[kind]@rotation(actor['yaw_rad']-yaw).T+relative
        clip=np.c_[vertices,np.ones(len(vertices))]@mvp.T
        planes=np.c_[clip[:,3]+clip[:,0],clip[:,3]-clip[:,0],clip[:,3]+clip[:,1],clip[:,3]-clip[:,1],clip[:,3]+clip[:,2],clip[:,3]-clip[:,2]]
        outside=bool(np.any(np.all(planes<0,axis=0)))
        label='outside_frustum' if outside else 'potentially_in_frustum'
        counts[actor['id']]['world_states']+=1;counts[actor['id']][label]+=1
        rows.append(dict(time_s=s['time_s'],global_id=actor['id'],frustum=label,source_visible=actor['source_visible']))
a.output.mkdir(exist_ok=False)
(a.output/'states.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows),encoding='utf-8')
report=dict(scene_artifact=m['artifact_id'],actors={k:dict(v) for k,v in counts.items()},
            method='actual_renderer_MVP_and_asset_bounds_plane_test',occlusion='NOT_EVALUATED',
            rasterized_visibility='NOT_EVALUATED',scope='Conservative frustum candidates; not an actual visible-object count or source recall')
(a.output/'review.json').write_bytes(canonical(report));print(json.dumps(report))
