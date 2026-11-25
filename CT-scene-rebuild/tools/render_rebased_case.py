"""Render a new published base with exactly the cached scene dependencies."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from ct_scene.artifacts import verify
from review_guards import require_same_base

p=argparse.ArgumentParser()
for flag in ('scene','base','engine','output'):p.add_argument('--'+flag,type=Path,required=True)
p.add_argument('--allow-base-change',action='store_true',help='Explicitly replace the scene frozen base; ordinary renderer regression must keep it identical')
p.add_argument('--contact-adapter',type=Path,help='Override opt-in adapter; by default preserve baseline report adapter')
p.add_argument('--cache-root',type=Path,action='append',default=[],help='Additional read-only artifact cache directory')
a=p.parse_args()
old=verify(a.scene);new_base=verify(a.base)
require_same_base(old,new_base,a.allow_base_change)
index={}
for root in [Path('runs'),*a.cache_root]:
    for pattern in ('*/artifact.json','*/*/artifact.json','*/*/*/artifact.json','*/*/*/*/artifact.json'):
        for f in root.glob(pattern):
            m=json.loads(f.read_text())
            if 'artifact_id' in m:index.setdefault(m['artifact_id'],f.parent)
roles={r['role']:r['artifact_id'] for r in old['inputs']}
print('roles',roles,flush=True)
command=[sys.executable,'-m','ct_scene','surrounding-prototype','--base',str(a.base),
         '--lens-config',str(a.engine/'configs/multicam-optical-candidates.json'),
         '--prototype-config',str(a.engine/'configs/surrounding-prototype.json'),
         '--assets-root',str(a.engine/'assets'),'--output',str(a.output)]
flags={'source':'package','tracks':'tracks','telemetry':'telemetry',
       'road_masks':'road-masks','road-masks':'road-masks',
       'camera_road_masks':'road-masks','osm_topology_reference':'osm-context',
       'ego_lamp_telemetry':'telemetry','front_image_body_heading':'image-headings',
       'osm_context':'osm-context','osm-context':'osm-context',
       'image_headings':'image-headings','image-headings':'image-headings'}
for role,artifact_id in roles.items():
    if role=='frozen_ego_lead':continue
    if role not in flags:raise ValueError('Unhandled cached dependency '+role)
    path=index[artifact_id]
    if verify(path)['artifact_id']!=artifact_id:raise ValueError('Changed dependency')
    command+=['--'+flags[role],str(path)]
adapter=a.contact_adapter
contact=json.loads((a.scene/'report.json').read_text()).get('learned_contact_adapter',{})
if adapter is None and contact.get('enabled'):
    adapter=Path(contact['model_dir']).resolve()
if adapter is not None:
    if not adapter.is_dir():raise ValueError('Baseline contact adapter missing; provide --contact-adapter')
    command+=['--contact-adapter',str(adapter)]
env=os.environ.copy();env['PYTHONPATH']=str((a.engine/'src').resolve())
raise SystemExit(subprocess.run(command,env=env).returncode)
