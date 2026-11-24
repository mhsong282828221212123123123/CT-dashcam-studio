"""OSM road prior in local ENU metres. Map geometry is not lane-level truth."""
import hashlib,json,math
from pathlib import Path
import numpy as np

ROAD_TYPES={'motorway','trunk','primary','secondary','tertiary','unclassified','residential','living_street','service',
            'motorway_link','trunk_link','primary_link','secondary_link','tertiary_link'}

def gps(data):
    if not data:return None
    lat,lon=data.get('lat'),data.get('lon')
    if lat is None or lon is None or not np.isfinite([lat,lon]).all() or not -90<lat<90 or not -180<lon<180 or (lat==0 and lon==0):return None
    return np.array([lat,lon],float)

def enu(coordinates,reference):
    a=np.asarray(coordinates,float)-reference
    return np.stack([a[...,1]*111320*math.cos(math.radians(reference[0])),a[...,0]*111320],axis=-1)

def basis(heading):
    h=math.radians(heading)
    return np.array([[-math.cos(h),math.sin(h)],[math.sin(h),math.cos(h)]])

class RoadMap:
    def __init__(self,payload,reference):
        self.reference=np.asarray(reference);self.ways=[];self.segments=[];self.junctions=[]
        memberships={};self.junction_records=[]
        for way in payload.get('elements',[]):
            tags=way.get('tags',{})
            if way.get('type')!='way' or tags.get('highway') not in ROAD_TYPES or len(way.get('geometry',[]))<2:continue
            points=enu([[p['lat'],p['lon']] for p in way['geometry']],self.reference)
            nodes=way.get('nodes',[]);level=(tags.get('layer','0'),tags.get('bridge','no'),tags.get('tunnel','no'))
            item={'id':way['id'],'points':points,'tags':tags,'nodes':nodes,'level':level}
            self.ways.append(item)
            for j,(a,b) in enumerate(zip(points,points[1:])):
                length=np.linalg.norm(b-a)
                if length<.1:continue
                self.segments.append((item,j,a,b,(b-a)/length))
            for j,node in enumerate(nodes):
                degree=1 if j in (0,len(nodes)-1) else 2
                old=memberships.setdefault((node,level),[0,points[min(j,len(points)-1)],set()])
                old[0]+=degree
                old[2].add(way['id'])
        for (node,level),(degree,point,way_ids) in memberships.items():
            if degree>=3 and len(way_ids)>=2:
                self.junction_records.append(dict(node_id=node,level=level,point=point,way_ids=way_ids,degree=degree))
        self.junctions=[record['point'] for record in self.junction_records]

    def candidates(self,position,heading):
        f=basis(heading)[:,1];result=[]
        for way,j,a,b,tangent in self.segments:
            q=float(np.clip(np.dot(position-a,b-a)/np.dot(b-a,b-a),0,1));point=a+q*(b-a)
            distance=float(np.linalg.norm(position-point))
            if distance>30:continue
            directions=[-1] if way['tags'].get('oneway')=='-1' else [1] if way['tags'].get('oneway') in ('yes','1') else [1,-1]
            direction=max(directions,key=lambda sign:np.dot(sign*tangent,f));forward=direction*tangent
            alignment=float(np.dot(forward,f))
            if alignment<.4:continue
            result.append({'way':way,'segment':j,'point':point,'forward':forward,'distance':distance,
                           'score':distance**2/36+8*(1-alignment),'direction':direction})
        return sorted(result,key=lambda c:c['score'])[:6]

    def match(self,scenes):
        """Sequence matching with connected-way transitions; preserves GPS residual."""
        all_candidates=[];costs=[];parents=[];last_position=None
        for scene in scenes:
            ll=gps(scene.telemetry);heading=(scene.telemetry or {}).get('heading')
            position=enu(ll,self.reference) if ll is not None else None
            cand=self.candidates(position,heading) if position is not None and heading is not None and math.isfinite(heading) else []
            score=[];parent=[]
            for c in cand:
                options=[]
                if all_candidates and all_candidates[-1] and last_position is not None:
                    travelled=np.linalg.norm(position-last_position)
                    for k,old in enumerate(all_candidates[-1]):
                        connected=c['way']['id']==old['way']['id'] or bool(set(c['way']['nodes'])&set(old['way']['nodes']))
                        penalty=0 if connected else 20
                        penalty+=abs(np.linalg.norm(c['point']-old['point'])-travelled)*.2
                        options.append((costs[-1][k]+penalty,k))
                best=min(options,default=(0,-1));score.append(best[0]+c['score']);parent.append(best[1])
            if score:
                minimum=min(score);score=[v-minimum for v in score]
            all_candidates.append(cand);costs.append(score);parents.append(parent);last_position=position
        matches=[None]*len(scenes);k=None
        for i in range(len(scenes)-1,-1,-1):
            if not costs[i]:k=None;continue
            if k is None or k<0:k=int(np.argmin(costs[i]))
            matches[i]=all_candidates[i][k];k=parents[i][k]
        return matches

def load_road_map(path):
    data=Path(path).read_bytes();payload=json.loads(data)
    if not isinstance(payload.get('elements'),list) or payload.get('remark'):raise ValueError('Incomplete or invalid OSM response')
    return {'payload':payload,'sha256':hashlib.sha256(data).hexdigest(),'attribution':'© OpenStreetMap contributors',
            'source':'OSM road geometry; not surveyed lane positions'}

def apply_map_roads(scenes,map_data,audits):
    """Attach contextual map hypotheses only; never register scene geometry.

    GPS can select nearby topology, not a lane or an exact intersection boundary.
    Camera road surfaces, markings, actors and host poses remain authoritative.
    """
    reference=next((gps(s.telemetry) for s in scenes if gps(s.telemetry) is not None),None)
    for scene,audit in zip(scenes,audits):
        scene.road_prior=None
        scene.road_context={'source':'osm','status':'unavailable','geometry_authority':False}
        audit['road_map']={'status':'unavailable','role':'shape_only','generated_lanes':0}
        scene.road_status=audit['road_map']
    if reference is None:return False
    roads=RoadMap(map_data['payload'],reference);matches=roads.match(scenes)
    for scene,match,audit in zip(scenes,matches,audits):
        if match is None:
            audit['road_map']['status']='unmatched'
            continue
        position=enu(gps(scene.telemetry),reference);B=basis(scene.telemetry['heading'])
        local=lambda p:((np.asarray(p)-position)@B).tolist()
        paths=[]
        for way in roads.ways:
            for a,b in zip(way['points'],way['points'][1:]):
                tangent=b-a;length=np.linalg.norm(tangent)
                if length<.1:continue
                q=np.clip(np.dot(position-a,tangent)/length**2,0,1)
                if np.linalg.norm(a+q*tangent-position)>85:continue
                # Keep local curve pieces, not arbitrarily long way endpoints.
                for lo in np.arange(max(0,q*length-90),min(length,q*length+90),3.):
                    hi=min(length,lo+3)
                    line=local([a+tangent*(lo/length),a+tangent*(hi/length)])
                    if max(p[1] for p in line)<-20 or min(p[1] for p in line)>75:continue
                    paths.append({'points':line,'way_id':way['id'],'matched_way':way['id']==match['way']['id']})
        # A crossing on an unrelated or grade-separated road is not our junction.
        junction_records=[]
        for record in roads.junction_records:
            if record['level']!=match['way']['level'] or match['way']['id'] not in record['way_ids']:continue
            point=local(record['point']);distance=float(np.linalg.norm(record['point']-position))
            if distance>=85 or not -20<=point[1]<=75:continue
            junction_records.append(dict(node_id=record['node_id'],point=point,distance_m=distance,
                degree=record['degree'],way_ids=sorted(record['way_ids']),
                kind='intersection' if record['degree']>=4 else 'junction_or_merge'))
        junction_records.sort(key=lambda record:record['distance_m'])
        junctions=[record['point'] for record in junction_records]
        scene.road_prior={'paths':paths,'junctions':junctions,'role':'shape_only',
            'junction_candidates':junction_records,
            'alignment':'GPS approximate; not registered to camera',
            'attribution':map_data.get('attribution','© OpenStreetMap contributors')}
        connected=[w for w in roads.ways if w['id']!=match['way']['id']
                   and w['level']==match['way']['level']
                   and set(w['nodes']).intersection(match['way']['nodes'])]
        vectors=np.diff(match['way']['points'],axis=0)
        lengths=np.linalg.norm(vectors,axis=1);vectors=vectors[lengths>.1]/lengths[lengths>.1,None]
        turn_angles=np.arccos(np.clip(np.sum(vectors[:-1]*vectors[1:],axis=1),-1,1)) if len(vectors)>1 else []
        scene.road_context=dict(source='osm',status='available',geometry_authority=False,
            nearby_junction_candidate=bool(junctions),connected_branch_candidate=bool(connected),
            junction_candidates=junction_records,
            curved_way_candidate=bool(len(turn_angles) and max(turn_angles)>.12),
            connected_way_ids=[w['id'] for w in connected],
            position_semantics='approximate GPS neighbourhood; camera confirmation required')
        audit['road_map']={'status':'shape_prior','role':'shape_only','way_id':match['way']['id'],
            'gps_distance_m':match['distance'],'generated_lanes':0,'lane_source':'camera_vehicle_inference',
            'scene_geometry_modified':False,'context':scene.road_context,
            'alignment':'not_camera_registered','sha256':map_data.get('sha256')}
        scene.road_status=audit['road_map']
    return True


def confirm_map_junctions(scenes,audits):
    """Distinguish a map candidate from camera/ego-motion supported passage.

    The map marker remains in its GPS schematic. Road surfaces come exclusively
    from camera evidence and host odometry, never from snapping to the OSM node.
    """
    for scene,audit in zip(scenes,audits):
        context=getattr(scene,'road_context',None)
        if not context or not context.get('nearby_junction_candidate'):continue
        evidence=audit.get('road_junction',{})
        camera=audit.get('camera_junction',{})
        confirmed=bool(evidence.get('osm_supported') or camera.get('active'))
        context['junction_indication']=dict(state=evidence.get('state','visible') if confirmed else 'candidate',
            camera_confirmed=confirmed,source='osm_and_camera' if confirmed else 'osm_only',
            position_semantics='approximate; not a lane position')
        audit['road_map']['junction_indication']=context['junction_indication']
