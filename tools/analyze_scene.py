"""Run: python tools/analyze_scene.py ORIGINAL-front.mp4 --output NEW_DIRECTORY"""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.scene_analysis import create_scene_detector as SceneDetector
from core.scene_pipeline import analyze, AnalysisCancelled, diagnostic_tile

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('front_video')
    parser.add_argument('--output', required=True)
    parser.add_argument('--sample-hz', type=float, default=10.)
    parser.add_argument('--start', type=float, default=0.)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--model', help='Optional YOLOv8 COCO ONNX model')
    parser.add_argument('--input-size', type=int, default=320)
    parser.add_argument('--confidence', type=float, default=.25)
    parser.add_argument('--backend',choices=['cpu','auto','directml'],default='cpu')
    parser.add_argument('--detail',choices=['standard','tiled'],default='standard')
    parser.add_argument('--device-id',type=int,default=0)
    parser.add_argument('--road-map',help='Cached Overpass JSON road geometry')
    parser.add_argument('--rear-calibration',help='Calibrated rear lens JSON; rectifies before detection/tracking')
    parser.add_argument('--camera-calibration', action='append', default=[], metavar='CAMERA=JSON',
                        help='Per-camera lens JSON; fresh inference/tracking in rectified pixels')
    parser.add_argument('--road-model',help='Optional YOLOP/YOLOPv2 road ONNX model')
    parser.add_argument('--tracker', choices=['motion', 'baseline'], default='motion')
    parser.add_argument('--review-reference',help='Explicit visually reviewed corrections; not automatic detections')
    parser.add_argument('--video', action='store_true', help='Also create diagnostic.mp4')
    args = parser.parse_args()
    camera_calibrations={}
    for value in args.camera_calibration:
        name, sep, path = value.partition('=')
        if not sep or not path or name in camera_calibrations:
            parser.error('Use one --camera-calibration CAMERA=JSON per camera')
        camera_calibrations[name]=path
    detector = SceneDetector(args.model, args.confidence, args.input_size,include_traffic_lights=True,backend=args.backend,detail=args.detail,device_id=args.device_id)
    print(detector.inference_info,flush=True)
    road_detector=None
    if args.road_model:
        from core.scene_road_model import RoadEvidenceDetector
        road_detector=RoadEvidenceDetector(args.road_model,'cpu' if args.backend=='cpu' else 'directml')
    metadata = analyze(args.front_video, args.output, args.sample_hz, args.duration, args.start,
                       detector=detector, tracker_type=args.tracker, make_video=args.video,review_reference=args.review_reference,road_map=args.road_map,rear_calibration=args.rear_calibration,road_detector=road_detector,camera_calibrations=camera_calibrations)
    print(f"Saved {metadata['sample_count']} samples to {args.output}")


if __name__ == '__main__':
    main()
