import argparse,json
from pathlib import Path
from ct_scene.background_rotation import run
ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--optics',default='configs/multicam-optical-candidates.json');a=ap.parse_args()
manifest=run(a.case/'source',a.case/'tracks',a.case/'telemetry',a.optics,a.output);print(json.dumps(dict(artifact_id=manifest['artifact_id'])))
