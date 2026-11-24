"""Export one current prototype sample and decode the complete MP4 for QA."""
import json
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.render_mobile_comparison import render


if __name__=='__main__':
    root=Path(sys.argv[1])
    timeline_path=Path(sys.argv[3]) if len(sys.argv)>3 else root/'current.pkl'
    with timeline_path.open('rb') as f:timeline=pickle.load(f)
    manifest=json.loads((root/'sample-input/manifest.json').read_text(encoding='utf8'))
    output=root/(sys.argv[2] if len(sys.argv)>2 else 'CT_Dashcam_current_20261114_35s.mp4')
    render(root/'sample-input',Path(manifest['source_front_path']).parent,output,
        duration=35.,fps=15,timeline=timeline,
        title=sys.argv[4] if len(sys.argv)>4 else '정차 방향 / 트럭 형상 수정 · 35초',
        subtitle=sys.argv[5] if len(sys.argv)>5 else '실험 보정 · 실제 진입 회전 유지 / 차량 기준 영점 미검증',
        footer_lines=['반투명 차량 = 추정 중 · 밴 / SUV / 포터 차종 검토 포함',
            '하단: 원본 4채널 · OSM은 도로 윤곽 참고',
            '거리·차체 기준 영점 및 세부 차선 미검증 · 검토용 프로토타입'])
    cap=cv2.VideoCapture(str(output));fps=cap.get(cv2.CAP_PROP_FPS)
    count=0;black=[];samples=[];shape=None
    while True:
        ok,image=cap.read()
        if not ok:break
        shape=image.shape
        if shape!=(1280,720,3):raise ValueError(f'Unexpected frame: {shape}')
        if image[80:560].mean()<2:black.append(count)
        if count in (0,180,330,517):
            thumb=cv2.resize(image,(360,640),interpolation=cv2.INTER_AREA)
            cv2.putText(thumb,f'OUTPUT {count/fps:.1f}s',(160,298),0,.5,(0,255,255),1,cv2.LINE_AA)
            samples.append(thumb)
        count+=1
    cap.release()
    if count!=525 or fps!=15 or black:raise ValueError(dict(frames=count,fps=fps,black=black))
    if len(samples)!=4:raise ValueError('QA frames missing')
    cv2.imencode('.jpg',np.vstack([np.hstack(samples[:2]),np.hstack(samples[2:])]))[1].tofile(root/'export-proof.jpg')
    report=dict(path=str(output.resolve()),duration_s=count/fps,fps=fps,frames=count,
        dimensions=[720,1280],codec='H.264',all_frames_decoded=True,black_scene_frames=black,
        original_camera_order=['front','back','left_repeater','right_repeater'],
        fresh_analysis_samples=manifest['sample_count'],
        inference=manifest.get('inference'),metric_accepted=False,
        optical_calibration='explicit experimental profiles, not installed defaults',
        reviewed_subtypes=manifest.get('sample_provenance',{}).get('reviewed_subtypes'),
        bytes=output.stat().st_size)
    (root/'export-qa.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False),flush=True)
