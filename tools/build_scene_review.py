"""Build a reproducible, read-only sample review package from original camera files."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import SceneDetector
from core.scene_fusion import FusedSceneTimeline
from core.driving_scene import DrivingRenderer
from core.scene_review import compare_targets,check_relations


def save_image(path,image):
    ok,buffer=cv2.imencode('.png',image)
    if not ok: raise RuntimeError('PNG encoding failed')
    buffer.tofile(str(path))


def build(result,source,references,output,model):
    result,source,references,output=map(Path,(result,source,references,output))
    if output.exists(): raise FileExistsError(output)
    manifest=json.loads((result/'manifest.json').read_text(encoding='utf-8'))
    labels=json.loads(references.read_text(encoding='utf-8'))
    if labels['front_file']!=manifest['cameras']['front']['file']: raise ValueError('Reference clip mismatch')
    rows=[json.loads(l) for l in (result/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    timeline=FusedSceneTimeline(rows,manifest)
    detector=SceneDetector(model,threshold=.25,input_size=640,include_traffic_lights=True)
    renderer=DrivingRenderer()
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='review-',dir=output.parent) as temp:
        stage=Path(temp)/'package'; stage.mkdir()
        cases=[]; relations={}; target_maps={}; rendered=set()
        for reference in labels['frames']:
            t,camera=reference['time_s'],reference['camera']
            index=min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-t))
            if abs(rows[index]['time_s']-t)>.02: raise ValueError('Requested review time not sampled')
            row=rows[index]; camera_row=row['cameras'][camera]
            filename=manifest['cameras'][camera]['file']
            if Path(filename).name!=filename: raise ValueError('Unsafe source filename')
            path=source/filename
            if not path.is_file(): raise FileNotFoundError(path)
            capture=cv2.VideoCapture(str(path))
            try:
                capture.set(cv2.CAP_PROP_POS_FRAMES,camera_row['frame_index'])
                ok,image=capture.read()
                if not ok: raise ValueError(f'Cannot decode {path}')
            finally: capture.release()
            name=f'{index:05d}-{camera}.png'; save_image(stage/name,image)
            scene_name=f'{index:05d}-scene.png'
            if index not in rendered:
                save_image(stage/scene_name,renderer.render(timeline.at(index))); rendered.add(index)
            baseline=camera_row.get('objects',[])
            probe=[vars(d) for d in detector.detect(image)]
            old=compare_targets(reference['targets'],baseline)
            new=compare_targets(reference['targets'],probe)
            raw=camera_row.get('raw_detections',baseline)
            raw_review=compare_targets(reference['targets'],raw)
            visible={source_id:actor for actor in timeline.at(index).actors for source_id in actor.sources}
            mapping=target_maps.setdefault(t,{})
            for match in old['results']:
                if match['detection_index'] is None: continue
                detection=baseline[match['detection_index']]
                actor=visible.get(detection.get('track_id'))
                match['source_id']=detection.get('track_id')
                match['placement']='placed' if actor else 'not_placed'
                if actor: mapping[match['target_id']]={'x':actor.x,'z':actor.z,'id':actor.key}
            cases.append(dict(reference,image=name,scene_image=scene_name,frame_index=camera_row['frame_index'],
                source_file=filename,source_size=path.stat().st_size,pixel_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
                baseline=baseline,probe=probe,baseline_review=old,probe_review=new,
                raw=raw,raw_review=raw_review,observed_lanes=camera_row.get('lanes',[]),
                observed_signals=camera_row.get('signals',[]),review_assisted=manifest.get('review_assisted',False),
                fusion_audit=timeline.audit[index]))
        for t,mapping in target_maps.items(): relations[str(t)]=check_relations(labels['relations'],mapping)
        totals={name:{key:sum(c[name+'_review'][key] for c in cases) for key in ('targets','matched','missed')}
                for name in ('baseline','probe','raw')}
        gaps=['실측 거리와 카메라 외부 파라미터는 검증되지 않았습니다.',
              '후방 잘린 차량의 자동 검출 누락은 남아 있으며 검토 보정과 분리합니다.',
              '차선은 관측한 페인트 구간만 표시하며 전체 차로 수를 추론하지 않습니다.',
              '야간 빛 번짐 신호는 unknown이며 신호의 적용 차로와 화살표는 미판정입니다.']
        report={'schema_version':1,'front_file':labels['front_file'],'scope':labels['scope'],
            'known_gaps':gaps,'cases':cases,'totals':totals,'relations':relations,
            'review_assisted':manifest.get('review_assisted',False),
            'independent_accuracy_validation':False,
            'source_result_sha256':hashlib.sha256((result/'observations.jsonl').read_bytes()).hexdigest(),
            'references_sha256':hashlib.sha256(references.read_bytes()).hexdigest(),
            'probe_model_sha256':hashlib.sha256(Path(model).read_bytes()).hexdigest(),
            'probe_config':{'input_size':640,'threshold':.25,'traffic_light_class':9,'tracking':False},
            'gates':{'vehicle_reconstruction':'estimated_with_unplaced_objects','lane_detection':'observed_segments_only',
                     'signal_state':'unknown_not_validated','metric_positions':'not_validated'}}
        (stage/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        lines=['# a6 검출 검토 결과','',f"표시 파이프라인 (검토 보정={manifest.get('review_assisted',False)}): {totals['baseline']['matched']}/{totals['baseline']['targets']} 선택 참조 대응.",
               f"자동 원시 검출: {totals['raw']['matched']}/{totals['raw']['targets']} 선택 참조 대응.",
               f"신호등 클래스 포함 재검출: {totals['probe']['matched']}/{totals['probe']['targets']} 선택 참조 대응.",
               '', '4개 시점 × 4개 카메라. 같은 정차 물체의 반복 관측이며 독립적인 차량 수가 아니다.',
               'IoU 0.3 이상의 일대일 차량 계열 대응. 전체 precision/recall 평가가 아니다.',
               '기존 파이프라인은 추적 전후 상태가 포함되고 재검출은 단일 프레임 원시 결과이므로 추적 성능 비교가 아니다.',
               '보정에 사용한 참조로 다시 비교했으므로 보정 결과를 독립적인 자동 검출 정확도로 해석하면 안 됩니다.',
               '', '## 해결되지 않은 항목','']+['- '+gap for gap in gaps]
        for c in cases:
            missed=[r['target_id'] for r in c['baseline_review']['results'] if r['status']=='missed']
            if missed: lines.append(f"- {c['time_s']}초 {c['camera']}: 기존 검출 누락 {', '.join(missed)}")
        (stage/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
        stage.rename(output)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('result','source','references','output','model'): parser.add_argument('--'+name,required=True)
    args=parser.parse_args()
    report=build(args.result,args.source,args.references,args.output,args.model)
    print(json.dumps(report['totals'],ensure_ascii=False))
