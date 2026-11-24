"""Attach a validated cached OSM payload to a new analysis result; no network."""
import argparse,json,shutil,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_map import load_road_map

def attach(result,map_path,output):
    result,output=Path(result),Path(output)
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((result/'manifest.json').read_text(encoding='utf-8'))
    manifest['road_map']=load_road_map(map_path)
    manifest['osm']='cached OSM connected road shape; camera/vehicle inferred width'
    output.mkdir(parents=True)
    shutil.copyfile(result/'observations.jsonl',output/'observations.jsonl')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(output,manifest['road_map']['sha256'])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result',required=True)
    p.add_argument('--map',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();attach(a.result,a.map,a.output)
