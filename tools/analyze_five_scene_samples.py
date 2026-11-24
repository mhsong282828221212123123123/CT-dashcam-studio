"""GPU producer: five frozen source clips, fresh analysis with identical settings."""
import json
import sys
from pathlib import Path
import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import SceneDetector
from core.scene_pipeline import analyze

if __name__=='__main__':
    cv2.setNumThreads(2)
    root=Path(sys.argv[1]);cases=json.loads((root/'selected.json').read_text(encoding='utf8'))
    detector=SceneDetector('../scene-models/yolo11x.onnx',.25,640,include_traffic_lights=True,backend='directml')
    print('INFERENCE',detector.inference_info,flush=True)
    lenses={n:root/f'lens-{n}.json' for n in ['front','left_repeater','right_repeater']}
    for case in cases:
        target=root/case['id'];target.mkdir(exist_ok=True)
        if (target/'analysis/manifest.json').exists():continue
        print('ANALYZE',case['number'],case['id'],flush=True)
        analyze(case['source'],target/'analysis',start=1.,duration=35.,sample_hz=10.,detector=detector,camera_calibrations=lenses)
        print('ANALYSIS READY',case['number'],case['id'],flush=True)
    print('ALL ANALYSES READY',flush=True)
