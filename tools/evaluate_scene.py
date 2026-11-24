"""Evaluate manually reviewed target boxes, not dataset-wide precision/recall.

Cases JSON: {cases: [{image, targets: [{kind,bbox}], negatives: [bbox]}]}.
Image paths are relative to the cases JSON. Unlabelled regions are not scored.
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
from core.scene_analysis import SceneDetector, iou, optimal_pairs, vehicle_family


def evaluate(cases_path, detector):
    cases_path = Path(cases_path)
    cases = json.loads(cases_path.read_text(encoding='utf-8'))['cases']
    results = []
    for case in cases:
        frame = cv2.imdecode(np.frombuffer((cases_path.parent/case['image']).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"Unreadable image: {case['image']}")
        detections = detector.detect(frame)
        targets = case.get('targets', [])
        scores = np.zeros((len(targets), len(detections)))
        for i, target in enumerate(targets):
            for j, detection in enumerate(detections):
                overlap = iou(target['bbox'], detection.bbox)
                if vehicle_family(target['kind']) == vehicle_family(detection.kind) and overlap >= .3:
                    scores[i, j] = overlap
        matches = optimal_pairs(scores)
        negative_hits = []
        for region in case.get('negatives', []):
            hits = []
            for d in detections:
                x, y, w, h = d.bbox
                cx, cy = x+w/2, y+h/2
                if region[0] <= cx <= region[0]+region[2] and region[1] <= cy <= region[1]+region[3]:
                    hits.append({'kind': d.kind, 'confidence': d.confidence})
            negative_hits.append(hits)
        results.append({'image': case['image'], 'labelled_targets': len(targets),
                        'matched_targets': len(matches), 'negative_region_hits': negative_hits,
                        'detections': [vars(d) for d in detections]})
    return {'scope': 'manually reviewed sparse targets; not full precision/recall or independent benchmark',
            'model': Path(detector.model_path).name, 'input_size': detector.input_size,
            'threshold': detector.threshold, 'cases': results,
            'labelled_targets': sum(r['labelled_targets'] for r in results),
            'matched_targets': sum(r['matched_targets'] for r in results),
            'negative_region_detections': sum(len(h) for r in results for h in r['negative_region_hits'])}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases')
    parser.add_argument('--model')
    parser.add_argument('--input-size', type=int, default=320)
    parser.add_argument('--confidence', type=float, default=.35)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = evaluate(args.cases, SceneDetector(args.model, args.confidence, args.input_size))
    with Path(args.output).open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(f"Matched {result['matched_targets']}/{result['labelled_targets']} reviewed targets; "
          f"{result['negative_region_detections']} detections in reviewed negative regions")
