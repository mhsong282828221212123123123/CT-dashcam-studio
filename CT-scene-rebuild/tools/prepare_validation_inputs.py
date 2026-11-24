"""Source/telemetry only, before model predictions; reusable immutable inputs."""
import argparse,json,shutil
from pathlib import Path
from ct_scene.annotation import prepare
from ct_scene.telemetry import audit_sei
from ct_scene.artifacts import canonical
ap=argparse.ArgumentParser();ap.add_argument('--selection',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(exist_ok=False)
shutil.copytree('src/ct_scene',a.output/'frozen/src/ct_scene',ignore=shutil.ignore_patterns('__pycache__'));sources=json.loads(a.selection.read_text())['selected'];record=[]
for source in sources:
    sid=source['source_id'];dest=a.output/sid;dest.mkdir();print('SOURCE_ONLY_START '+sid,flush=True)
    pm=prepare(source['source_files']['front'],dest/'source',source['last_s'],5);tm=audit_sei(source['source_files']['front'],dest/'telemetry',source['last_s'])
    record.append(dict(source_id=sid,source_artifact=pm['artifact_id'],telemetry_artifact=tm['artifact_id'],predictions_run=False));(a.output/'inputs.json').write_bytes(canonical(record));print('SOURCE_ONLY_END '+sid,flush=True)
