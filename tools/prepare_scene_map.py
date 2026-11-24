"""Fetch/cache road vectors for an analyzed clip; sends only a rounded bounding box."""
import argparse,json,math,sys,urllib.request,urllib.parse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_map import gps,load_road_map

# Private.coffee is the officially published successor to the Kumi endpoint.
# Selection stays explicit: do not silently send clip locations to another host.
MAP_ENDPOINTS=('https://overpass-api.de/api/interpreter',
               'https://overpass.kumi.systems/api/interpreter',
               'https://overpass.private.coffee/api/interpreter')
DEFAULT_MAP_ENDPOINT='https://overpass.private.coffee/api/interpreter'

def fetch(result,output,transport='post',endpoint=DEFAULT_MAP_ENDPOINT):
    if Path(output).exists():
        cached=load_road_map(output)
        print('Using cached map',output,cached['sha256'])
        return
    rows=[json.loads(x) for x in (Path(result)/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    positions=[gps((r.get('ego') or {}).get('data')) for r in rows];positions=[p for p in positions if p is not None]
    if not positions:raise ValueError('No valid GPS for map bounds')
    south=math.floor((min(p[0] for p in positions)-.001)/.002)*.002
    north=math.ceil((max(p[0] for p in positions)+.001)/.002)*.002
    west=math.floor((min(p[1] for p in positions)-.001)/.002)*.002
    east=math.ceil((max(p[1] for p in positions)+.001)/.002)*.002
    if (north-south)*(east-west)>.01:raise ValueError('Clip area too large for bounded map request')
    query=f'[out:json][timeout:25];way["highway"]({south:.6f},{west:.6f},{north:.6f},{east:.6f});out body geom;'
    encoded=urllib.parse.urlencode({'data':query})
    if endpoint not in MAP_ENDPOINTS:
        raise ValueError('Unsupported map endpoint')
    url=endpoint
    request=urllib.request.Request(url+'?'+encoded if transport=='get' else url,
        data=None if transport=='get' else encoded.encode(),headers={'User-Agent':'CTDashcamStudio/SceneLab'})
    with urllib.request.urlopen(request,timeout=40) as response:raw=response.read(20_000_000)
    payload=json.loads(raw)
    if payload.get('remark') or not isinstance(payload.get('elements'),list):raise ValueError('Incomplete OSM response')
    path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as stream:stream.write(raw)
    print(path,len(payload['elements']),load_road_map(path)['sha256'])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result',required=True);p.add_argument('--output',required=True)
    p.add_argument('--transport',choices=['post','get'],default='post')
    p.add_argument('--endpoint',default=DEFAULT_MAP_ENDPOINT)
    a=p.parse_args();fetch(a.result,a.output,a.transport,a.endpoint)
