"""A/B coverage and image consistency; no claim of metric ground truth."""
import argparse,collections,hashlib,json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline,corners,dimensions
from core.scene_analysis import CameraReader

def validate(result,source,output):
    root=Path(result);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[json.loads(x) for x in (root/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'));manifest.pop('road_map',None)
    before=FusedSceneTimeline(rows,manifest,postprocess=False);after=FusedSceneTimeline(rows,manifest)
    events=[dict(time_s=audit['time_s'],**e) for audit in after.audit for e in audit.get('postprocess',[]) if e['status']=='recovered']
    expected=0;coverage=[0,0];overlaps=[]
    for i,row in enumerate(rows):
        sources={o['track_id'] for name,c in row['cameras'].items() if name!='back' and c.get('available')
                 and abs(c.get('source_time_s',row['time_s'])-row['time_s'])<=.08
                 for o in c.get('objects',[]) if o.get('track_status')=='confirmed' and not o.get('association_only')}
        expected+=len(sources)
        for n,timeline in enumerate((before,after)):
            placed={source for a in timeline.scenes[i].actors for source in a.sources}
            coverage[n]+=len(sources&placed)
        actors=after.scenes[i].actors
        assert len({a.key for a in actors})==len(actors)
        for k,a in enumerate(actors):
            for b in actors[k+1:]:
                if abs(a.x-b.x)<(a.width+b.width)/2 and abs(a.z-b.z)<(a.length+b.length)/2:overlaps.append([row['time_s'],a.key,b.key])
    report={'primary_confirmed_object_observations':expected,'supported_position_observations_before':coverage[0],
        'supported_position_observations_after':coverage[1],'coverage_before':coverage[0]/max(1,expected),
        'coverage_after':coverage[1]/max(1,expected),'recovered_observations':len(events),
        'previous_states':dict(collections.Counter(str(e['prior_state']) for e in events)),
        'overlap_frames':overlaps,'reprojection_iou_min':min((e['reprojection_iou'] for e in events),default=None),
        'limitations':'Observation coverage and image-box agreement are not metric accuracy or independent identity ground truth',
        'code_sha256':hashlib.sha256((Path(__file__).resolve().parents[1]/'core/scene_postprocess.py').read_bytes()).hexdigest(),
        'events':events}
    # Every recovered frame keeps its detector box and reprojected 3D envelope
    # for visual audit. Green=image evidence, magenta=world trajectory projection.
    grouped=collections.defaultdict(list)
    for e in events:grouped[e['source'].split(':')[0]].append(e)
    for camera,items in grouped.items():
        reader=CameraReader(Path(source)/manifest['cameras'][camera]['file'])
        try:
            for e in sorted(items,key=lambda e:e['time_s']):
                i=min(range(len(rows)),key=lambda k:abs(rows[k]['time_s']-e['time_s']))
                _,im=reader.read_at(e['time_s']);im=im.copy()
                obj=next(o for o in rows[i]['cameras'][camera]['objects'] if o['track_id']==e['source'])
                actor=next(a for a in after.scenes[i].actors if a.key==e['id'])
                shape=actor.geometry_kind or actor.kind
                bounds=after.camera_frames[i][camera].visible_bounds(corners(actor.x,actor.z,dimensions(shape),kind=shape))
                x,y,w,h=map(round,obj['bbox']);cv2.rectangle(im,(x,y),(x+w,y+h),(0,220,0),2)
                x,y,x2,y2=map(round,bounds);cv2.rectangle(im,(x,y),(x2,y2),(220,0,220),2)
                cv2.putText(im,f"{camera} {e['time_s']:.2f}s IoU={e['reprojection_iou']:.2f}",(15,30),0,.7,(255,255,255),2)
                cv2.imencode('.jpg',im)[1].tofile(str(out/f"{camera}-{i:04d}-{e['source'].split(':')[-1]}.jpg"))
        finally:reader.close()
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print({k:v for k,v in report.items() if k!='events'},flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for arg in ('result','source','output'):p.add_argument('--'+arg,required=True)
    a=p.parse_args();validate(a.result,a.source,a.output)
