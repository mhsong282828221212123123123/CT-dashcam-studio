import argparse,json
from pathlib import Path
from ct_scene.image_vehicle_heading import run
ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--model',required=True);ap.add_argument('--optics',default='configs/multicam-optical-candidates.json');ap.add_argument('--backend',choices=['directml','cpu'],default='directml');a=ap.parse_args()
r=run(a.case/'source',a.case/'tracks',a.model,a.optics,a.output,a.backend);print(json.dumps(r))
