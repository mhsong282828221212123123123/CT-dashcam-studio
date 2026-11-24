"""Preserve the exact current engine only when it matches a published scene."""
import argparse,ast,hashlib,json,shutil
from pathlib import Path
from ct_scene.artifacts import verify

ap=argparse.ArgumentParser();ap.add_argument('--scene',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
manifest=verify(args.scene);source=Path('src/ct_scene/surrounding.py');tree=ast.parse(source.read_text(encoding='utf-8'))
names=next(ast.literal_eval(n.value) for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='revision_names' for t in n.targets))
revision=hashlib.sha256(b''.join(source.with_name(name).read_bytes() for name in names)).hexdigest()
if revision!=manifest['code_revision']:raise ValueError('Current source differs from published scene; do not label this as its engine snapshot')
args.output.mkdir(parents=True,exist_ok=False)
for folder in ['src','configs','tests']:
    shutil.copytree(folder,args.output/folder,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
for p in [*Path('.').glob('requirements*'),Path('pyproject.toml')]:
    if p.is_file():shutil.copy2(p,args.output/p.name)
files={str(p.relative_to(args.output)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.rglob('*') if p.is_file()}
record=dict(scene_artifact_id=manifest['artifact_id'],engine_revision=revision,files=files,scope='engine/config/tests snapshot; models, original video and dependencies remain external verified inputs')
(args.output/'snapshot-manifest.json').write_text(json.dumps(record,indent=2),encoding='utf-8');print(json.dumps(dict(engine_revision=revision,files=len(files),scene_artifact_id=manifest['artifact_id'])))
