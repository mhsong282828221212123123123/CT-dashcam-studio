"""Read-only OSM topology hints. No ego registration, lane or width inference."""
from collections import defaultdict
from pathlib import Path
import json,math,xml.etree.ElementTree as ET
from urllib.parse import urlencode
from urllib.request import Request,urlopen
import numpy as np
from .artifacts import canonical,publish,digest,verify
from .ingest import file_hash

DRIVABLE={'motorway','motorway_link','trunk','trunk_link','primary','primary_link','secondary','secondary_link','tertiary','tertiary_link','unclassified','residential','living_street','service'}


def load_roads(path):
    path=Path(path)
    if path.suffix.lower() in ('.osm','.xml'):
        root=ET.parse(path).getroot();nodes={int(n.attrib['id']):[float(n.attrib['lat']),float(n.attrib['lon'])] for n in root.findall('node')};ways=[]
        for w in root.findall('way'):
            tags={t.attrib['k']:t.attrib['v'] for t in w.findall('tag')};ids=[int(n.attrib['ref']) for n in w.findall('nd')]
            if tags.get('highway') not in DRIVABLE or any(n not in nodes for n in ids):continue
            ways.append(dict(id=int(w.attrib['id']),nodes=ids,geometry=[nodes[n] for n in ids],tags=tags))
    else:
        data=json.loads(path.read_text(encoding='utf-8'));ways=[]
        nodes={int(n['id']):[n['lat'],n['lon']] for n in data.get('elements',[]) if n.get('type')=='node'}
        for w in data.get('elements',[]):
            if w.get('type')!='way' or w.get('tags',{}).get('highway') not in DRIVABLE:continue
            ids=w.get('nodes',[]);geometry=w.get('geometry')
            points=[[p['lat'],p['lon']] for p in geometry] if geometry else [nodes[n] for n in ids if n in nodes]
            if len(ids)!=len(points) or len(ids)<2:continue
            ways.append(dict(id=w['id'],nodes=ids,geometry=points,tags=w.get('tags',{})))
    return ways


def level(tags):
    return (str(tags.get('layer','0')),tags.get('bridge','no') not in ('no','false','0'),tags.get('tunnel','no') not in ('no','false','0'))


def junctions(ways):
    neighbors=defaultdict(set);members=defaultdict(set);positions={};arms=defaultdict(dict)
    for w in ways:
        grade=level(w['tags'])
        for i,(nid,p) in enumerate(zip(w['nodes'],w['geometry'])):
            key=(nid,grade);positions[key]=p;members[key].add(w['id'])
            if i:neighbors[key].add(w['nodes'][i-1])
            if i+1<len(w['nodes']):neighbors[key].add(w['nodes'][i+1])
            for j in [i-1,i+1]:
                if not 0<=j<len(w['nodes']):continue
                nid2=w['nodes'][j];target=w['geometry'][j];step=1 if j>i else -1;end=j
                while np.linalg.norm(xy(target,p))<12 and 0<=end+step<len(w['nodes']):
                    end+=step;target=w['geometry'][end]
                arm=arms[key].setdefault(nid2,dict(neighbor_node_id=nid2,position=target,way_ids=set()))
                if np.linalg.norm(xy(target,p))>np.linalg.norm(xy(arm['position'],p)):arm['position']=target
                arm['way_ids'].add(w['id'])
    def branches(key):
        result=[]
        for nid,arm in sorted(arms[key].items()):
            # Use the first ~12m of an outgoing polyline for a stable local
            # tangent; adjacent OSM nodes may be centimetres apart.
            origin=positions[key];target=arm['position'];delta=xy(target,origin)
            if np.linalg.norm(delta)<.5:continue
            result.append(dict(neighbor_node_id=nid,way_ids=sorted(arm['way_ids']),bearing_deg=float(math.degrees(math.atan2(delta[0],delta[1]))%360)))
        return result
    return [dict(node_id=k[0],level=list(k[1]),latlon=positions[k],way_ids=sorted(members[k]),degree=len(ns),branches=branches(k),
                 kind='junction_or_merge_candidate',lane_information_used=False) for k,ns in neighbors.items() if len(ns)>=3 and len(members[k])>=2]


def xy(points,origin):
    p=np.asarray(points,float);lat,lon=origin
    return np.stack([(p[...,1]-lon)*111320*math.cos(math.radians(lat)),(p[...,0]-lat)*111320],axis=-1)


def candidates(ways,junction_rows,lat,lon,heading,max_distance=65.):
    scored=[]
    for w in ways:
        p=xy(w['geometry'],[lat,lon]);d=np.diff(p,axis=0);length=np.sum(d*d,axis=1);valid=length>1e-6
        if not np.any(valid):continue
        a=p[:-1][valid];d=d[valid];length=length[valid];u=np.clip(-np.sum(a*d,axis=1)/length,0,1)
        residual=a+u[:,None]*d;distance=np.linalg.norm(residual,axis=1);i=int(np.argmin(distance))
        if distance[i]>max_distance:continue
        bearing=math.atan2(d[i,0],d[i,1]);alignment=abs(math.sin(bearing-math.radians(heading)))
        if alignment>.65:continue
        scored.append((float(distance[i]+alignment*20),w,float(distance[i])))
    scored.sort(key=lambda r:r[0]);best=scored[:3];ids={w['id'] for _,w,_ in best};h=math.radians(heading);near=[]
    for j in junction_rows:
        if not ids.intersection(j['way_ids']):continue
        p=xy(j['latlon'],[lat,lon]);forward=p[0]*math.sin(h)+p[1]*math.cos(h)
        if np.linalg.norm(p)>100 or not -20<forward<90:continue
        branches=[dict(b,relative_bearing_deg=float((b['bearing_deg']-heading+180)%360-180)) for b in j['branches']]
        near.append(dict(j,branches=branches,distance_m=float(np.linalg.norm(p)),forward_m=float(forward),support='map_topology_only; GPS lane unresolved'))
    return dict(way_candidates=[dict(way_id=w['id'],distance_m=d,score=s) for s,w,d in best],
                ambiguous=bool(len(best)>1 and best[1][0]-best[0][0]<8),junction_candidates=near)


def run(telemetry,osm,destination):
    telemetry=Path(telemetry);m=verify(telemetry);ways=load_roads(osm);js=junctions(ways);records=[]
    for i,r in enumerate(map(json.loads,(telemetry/'telemetry.jsonl').read_text().splitlines())):
        if i%30:continue
        f=r['fields'];keys=['latitude_deg','longitude_deg','heading_deg']
        if not all(f[k]['valid'] for k in keys):continue
        lat,lon,heading=[float(f[k]['value']) for k in keys]
        if not (-90<=lat<=90 and -180<=lon<=180) or abs(lat)+abs(lon)<1e-6:continue
        records.append(dict(time_s=r['camera_time_s'],**candidates(ways,js,lat,lon,heading)))
    report=dict(ways=len(ways),junction_nodes=len(js),records=len(records),candidate_records=sum(bool(r['junction_candidates']) for r in records),
                osm_path=str(Path(osm).resolve()),
                role='topology_reference_only',ego_modified=False,lane_count_used=False,width_used=False,metric_registration='NOT_EVALUATED',
                attribution='© OpenStreetMap contributors; ODbL; https://www.openstreetmap.org/copyright')
    return publish(Path(destination),stage='osm_topology_context',payloads={'context.json':canonical(records),'report.json':canonical(report)},
                   inputs=[dict(role='telemetry',artifact_id=m['artifact_id'])],config={'osm_sha256':file_hash(Path(osm))},
                   code_revision=digest(Path(__file__).read_bytes()),runtime={'network_requests':0})


def _fetch_overpass(south,west,north,east,timeout=25):
    query=f'[out:xml][timeout:{timeout}];(way["highway"]({south:.7f},{west:.7f},{north:.7f},{east:.7f}););(._;>;);out body;'
    for endpoint in ('https://overpass-api.de/api/interpreter','https://overpass.kumi.systems/api/interpreter'):
        try:
            req=Request(endpoint,data=urlencode({'data':query}).encode('utf-8'),
                        headers={'User-Agent':'CTSceneRebuild/0.1 (read-only local scene topology)','Accept':'application/xml'})
            with urlopen(req,timeout=timeout) as response:data=response.read(32*1024*1024+1)
            if len(data)<=32*1024*1024 and ET.fromstring(data).tag=='osm':return data,endpoint
        except Exception:continue
    return None,None


def fetch(telemetry,cache,margin_deg=.0025):
    """Explicit user-authorized small bbox request, once; immutable local cache."""
    cache=Path(cache)
    if cache.exists():
        load_roads(cache)
        return dict(cache=str(cache),network_requests=0,status='reused_cache')
    points=[]
    for r in map(json.loads,(Path(telemetry)/'telemetry.jsonl').read_text().splitlines()):
        f=r['fields']
        if all(f[k]['valid'] for k in ('latitude_deg','longitude_deg')):
            lat,lon=f['latitude_deg']['value'],f['longitude_deg']['value']
            if -85<lat<85 and -180<lon<180 and abs(lat)+abs(lon)>1:points.append([lat,lon])
    if not points:raise ValueError('No valid GPS bounds')
    p=np.asarray(points);data=None;source_url=None;requests_made=0;last_error=None
    margins=[margin_deg,.0015,.001] if margin_deg>.001 else [margin_deg]
    for m_deg in margins:
        south,west=p.min(axis=0)-m_deg;north,east=p.max(axis=0)+m_deg
        if (north-south)*(east-west)>.02:continue
        url=f'https://api.openstreetmap.org/api/0.6/map?bbox={west:.7f},{south:.7f},{east:.7f},{north:.7f}'
        request=Request(url,headers={'User-Agent':'CTSceneRebuild/0.1 (read-only local scene topology)','Accept':'application/xml'})
        requests_made+=1
        try:
            with urlopen(request,timeout=25) as response:candidate=response.read(32*1024*1024+1)
            if len(candidate)<=32*1024*1024 and ET.fromstring(candidate).tag=='osm':
                data=candidate;source_url=url;break
        except Exception as exc:
            last_error=exc
    if data is None:
        south,west=p.min(axis=0)-margin_deg;north,east=p.max(axis=0)+margin_deg
        if (north-south)*(east-west)<=.05:
            requests_made+=1
            data,source_url=_fetch_overpass(south,west,north,east)
    if data is None:
        raise ValueError(f'OSM fetch failed ({last_error or "area_limit_or_endpoint_unavailable"})')
    cache.parent.mkdir(parents=True,exist_ok=True)
    with cache.open('xb') as stream:stream.write(data)
    return dict(cache=str(cache),network_requests=requests_made,status='downloaded',source=source_url,ways=len(load_roads(cache)))
