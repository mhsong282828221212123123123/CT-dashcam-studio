"""Small, explicit OSM Editing API download; cache once, never bulk tile.

Use regional extracts for large workloads. No OSM database edits are made.
"""
import argparse,json,math,sys,urllib.request,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_map import gps,ROAD_TYPES,load_road_map


def convert(raw):
    root=ET.fromstring(raw)
    if root.tag!='osm' or root.find('error') is not None:raise ValueError('Invalid OSM response')
    nodes={int(n.attrib['id']):dict(lat=float(n.attrib['lat']),lon=float(n.attrib['lon'])) for n in root.findall('node')}
    ways=[]
    for way in root.findall('way'):
        tags={t.attrib['k']:t.attrib['v'] for t in way.findall('tag')}
        if tags.get('highway') not in ROAD_TYPES:continue
        refs=[int(n.attrib['ref']) for n in way.findall('nd')]
        if len(refs)<2:continue
        if any(n not in nodes for n in refs):raise ValueError('Incomplete OSM way nodes')
        ways.append(dict(type='way',id=int(way.attrib['id']),nodes=refs,tags=tags,geometry=[nodes[n] for n in refs]))
    if not ways:raise ValueError('No supported roads in response')
    return dict(elements=ways,source='https://api.openstreetmap.org/api/0.6/map',
                attribution='© OpenStreetMap contributors')


def prepare(result,output,xml=None):
    output=Path(output)
    if output.exists():load_road_map(output);print('Using cached map',output);return
    if xml:
        raw=Path(xml).read_bytes()
    else:
        rows=[json.loads(line) for line in (Path(result)/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
        positions=[gps((row.get('ego') or {}).get('data')) for row in rows]
        positions=[p for p in positions if p is not None]
        if not positions:raise ValueError('No valid GPS')
        south=math.floor((min(p[0] for p in positions)-.001)/.002)*.002
        north=math.ceil((max(p[0] for p in positions)+.001)/.002)*.002
        west=math.floor((min(p[1] for p in positions)-.001)/.002)*.002
        east=math.ceil((max(p[1] for p in positions)+.001)/.002)*.002
        if (north-south)*(east-west)>.0001:raise ValueError('Use a regional extract for larger areas')
        request=urllib.request.Request(f'https://api.openstreetmap.org/api/0.6/map?bbox={west},{south},{east},{north}',
                                       headers={'User-Agent':'CTDashcamStudio/SceneLab'})
        with urllib.request.urlopen(request,timeout=40) as response:raw=response.read(20_000_001)
        if len(raw)>20_000_000:raise ValueError('Map response too large')
    payload=convert(raw);output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as stream:json.dump(payload,stream,ensure_ascii=False)
    print(output,len(payload['elements']),'roads')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--result');p.add_argument('--output',required=True)
    p.add_argument('--xml',help='Convert an existing download without a network request')
    args=p.parse_args()
    if not args.result and not args.xml:p.error('--result or --xml required')
    prepare(args.result,args.output,args.xml)
