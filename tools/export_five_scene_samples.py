"""CPU consumer: reviewed subtype labels, current fusion, portrait MP4 and QA.

Runs alongside the single GPU analysis producer. Original media are read only.
Only matching local OSM caches are consulted; this tool makes no map requests.
"""
import copy
import argparse
import hashlib
import json
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_fusion import FusedSceneTimeline
from core.scene_map import RoadMap,gps,enu
from core.driving_scene import DrivingRenderer
from tools.render_mobile_comparison import render
from tools.audit_scene_motion import audit_motion
from tools.validate_single_clip_reference import motion_outliers


def wait_for(path):
    began=time.monotonic()
    while not path.exists():
        if time.monotonic()-began>1800:raise TimeoutError(str(path))
        time.sleep(1)


def cache_bank(root):
    result=[];seen=set()
    for path in root.parent.rglob('manifest.json'):
        if root in path.parents:continue
        try:
            data=json.loads(path.read_text(encoding='utf8')).get('road_map')
            if not data or not data.get('payload',{}).get('elements'):continue
            key=data.get('sha256') or hashlib.sha256(json.dumps(data).encode()).hexdigest()
            if key in seen:continue
            seen.add(key);result.append((path,data))
        except (ValueError,OSError):continue
    return result


def select_cache(rows,bank):
    samples=[r['ego']['data'] for r in rows[::20] if r.get('ego') and gps(r['ego']['data']) is not None]
    if not samples:return None,0.,None
    reference=gps(samples[0]);best=(0.,None,None)
    for path,data in bank:
        roads=RoadMap(data['payload'],reference);hits=0
        for sample in samples:
            candidates=roads.candidates(enu(gps(sample),reference),sample.get('heading',0.))
            if any(c['distance']<=20 and c['score']<12 for c in candidates):hits+=1
        coverage=hits/len(samples)
        if coverage>best[0]:best=(coverage,data,str(path))
    return (copy.deepcopy(best[1]),best[0],best[2]) if best[0]>=.8 else (None,best[0],None)


if __name__=='__main__':
    cv2.setNumThreads(2)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--cases',nargs='+',type=int)
    parser.add_argument('--revision',default='')
    args=parser.parse_args();root=args.root
    cases=json.loads((root/'selected.json').read_text(encoding='utf8'))
    if args.cases:cases=[c for c in cases if c['number'] in args.cases]
    bank=cache_bank(root);print('LOCAL MAP CACHES',len(bank),flush=True)
    for case in cases:
        source_case=root/case['id'];target=source_case/args.revision if args.revision else source_case
        target.mkdir(exist_ok=True)
        if (target/'export-qa.json').exists():continue
        wait_for(source_case/'analysis/manifest.json');wait_for(source_case/'subtype-review.json')
        review=json.loads((source_case/'subtype-review.json').read_text(encoding='utf8'))
        manifest=json.loads((source_case/'analysis/manifest.json').read_text(encoding='utf8'))
        if review['front_file']!=manifest['cameras']['front']['file']:raise ValueError('Subtype review source mismatch')
        rows=[json.loads(s) for s in (source_case/'analysis/observations.jsonl').read_text(encoding='utf8').splitlines()]
        changes={key:0 for key in review.get('labels',{})}
        for row in rows:
            for data in row['cameras'].values():
                for obj in data.get('objects',[]):
                    kind=review.get('labels',{}).get(obj['track_id'])
                    if kind:
                        obj.update(render_kind=kind,geometry_kind=kind,subtype_review='per-clip visual review; no manual coordinates')
                        changes[obj['track_id']]+=1
        roadmap,coverage,cachepath=select_cache(rows,bank)
        if roadmap:manifest['road_map']=roadmap
        manifest.update(motion_bundle_refinement=True,review_assisted=bool(changes))
        manifest['osm']='matching local cached outline only' if roadmap else 'no covering local cache; no external map request'
        manifest['sample_provenance']=dict(experiment='five_new_source_clips_current_code',
            subtype_review=review,subtype_observations=changes,fresh_analysis=True,
            cached_map_coverage=coverage,cached_map_manifest=cachepath,
            camera_vehicle_origin_verified=False)
        derived=target/'sample-input';derived.mkdir(exist_ok=True)
        (derived/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf8')
        (derived/'observations.jsonl').write_text('\n'.join(json.dumps(r,ensure_ascii=False,allow_nan=False) for r in rows)+'\n',encoding='utf8')
        print('FUSE',case['number'],case['id'],flush=True)
        def progress(r):print('BUNDLE',case['number'],r['source'],r.get('accepted'),flush=True)
        timeline=FusedSceneTimeline(rows,manifest,motion_progress=progress)
        with (target/'timeline.pkl').open('wb') as f:pickle.dump(timeline,f)
        audit=audit_motion(timeline);outliers=motion_outliers(timeline)
        (target/'track-audit.json').write_text(json.dumps(audit,indent=2),encoding='utf8')
        (target/'motion-outliers.json').write_text(json.dumps(outliers,indent=2),encoding='utf8')
        (target/'models.json').write_text(json.dumps(timeline.motion_bundle_report,indent=2),encoding='utf8')
        renderer=DrivingRenderer(720,480);proof=[]
        for t in (2.,12.,24.,34.):
            image=renderer.render(timeline.sample(t));cv2.putText(image,f'SOURCE {t:.1f}s',(15,30),0,.7,(30,40,170),2)
            proof.append(image)
        cv2.imencode('.jpg',np.vstack([np.hstack(proof[:2]),np.hstack(proof[2:])]))[1].tofile(target/'scene-proof.jpg')
        output=target/f'CT_Dashcam_sample_{case["number"]:02d}_{case["id"]}_35s.mp4'
        print('RENDER',case['number'],flush=True)
        render(derived,Path(case['source']).parent,output,duration=35.,fps=15,timeline=timeline,
            title=f'{case["number"]}/5 · {case["title"]} · 35초',
            subtitle='새 원본 · 차로 내 주행 우선 추정 / 카메라 영점 미검증',
            footer_lines=['반투명 = 추정 중 · '+('차종 육안 검토 포함' if changes else '자동 검출 / 추정'),
                '원본 4채널 · '+('OSM: 일치하는 로컬 캐시 참고' if roadmap else 'OSM: 해당 구간 캐시 미확보'),
                '검토용 프로토타입 · 거리 / 차체 기준점 / 도로 형상 미검증'])
        cap=cv2.VideoCapture(str(output));fps=cap.get(cv2.CAP_PROP_FPS);count=0;black=[];tiles=[]
        for_expected=525
        while True:
            ok,image=cap.read()
            if not ok:break
            if image.shape!=(1280,720,3):raise ValueError('Unexpected video dimensions')
            if image[80:560].mean()<2:black.append(count)
            if count in (0,60,120,180,240,300,360,420,517):
                thumb=cv2.resize(image,(240,427));cv2.putText(thumb,f'{count/fps:.1f}s',(168,199),0,.45,(0,255,255),1,cv2.LINE_AA);tiles.append(thumb)
            count+=1
        cap.release()
        if count!=for_expected or fps!=15 or black:raise ValueError(dict(frames=count,fps=fps,black=black))
        cv2.imencode('.jpg',np.vstack([np.hstack(tiles[k:k+3]) for k in range(0,9,3)]))[1].tofile(target/'export-proof.jpg')
        result=dict(number=case['number'],source=case['id'],title=case['title'],path=str(output.resolve()),
            frames=count,fps=fps,duration_s=count/fps,dimensions=[720,1280],bytes=output.stat().st_size,
            all_frames_decoded=True,black_scene_frames=black,metric_accepted=False,
            source_review=review,eligible_observations=audit['observation_frames'],
            unplaced_observations=audit['missing_frames'],motion_outlier_steps=len(outliers),
            lane_prior_corrections=timeline.lane_prior_corrections,map_cache_used=bool(roadmap),map_cache_coverage=coverage,
            inference=manifest.get('inference'))
        (target/'export-qa.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf8')
        print('READY',case['number'],str(output),flush=True)
    print('ALL FIVE EXPORTED',flush=True)
