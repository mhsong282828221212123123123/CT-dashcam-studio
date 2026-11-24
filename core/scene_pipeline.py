"""Offline scene analysis with cancellable, atomic result publication."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np
from core.decoder import RealTeslaSEIDecoder
from core.scene_analysis import CAMERAS, CameraReader, PixelTracker, MotionTracker, SceneDetector, discover_clip
from core.scene_features import detect_lane_segments,SignalStateTracker
from core.scene_junction import junction_evidence,JunctionTracker
from core.scene_review_assist import ReviewedObservationAssist


class AnalysisCancelled(Exception):
    pass


def diagnostic_tile(frame, camera, timestamp, observations, lanes=(),signals=(),central_boundary=None):
    if frame is None:
        tile = np.zeros((240, 426, 3), np.uint8)
        cv2.putText(tile, 'MISSING / UNREADABLE', (15, 120), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 120, 255), 1)
    else:
        tile = frame.copy()
        # Keep image aspect ratio in the diagnostic grid as well.
        h, w = tile.shape[:2]
        scale = min(426/w, 240/h)
        resized = cv2.resize(tile, (max(1, round(w*scale)), max(1, round(h*scale))))
        tile = np.zeros((240, 426, 3), np.uint8)
        tile[:resized.shape[0], :resized.shape[1]] = resized
        for observation in observations:
            if observation.get('mask_polygon'):
                cv2.polylines(tile,[np.round(np.array(observation['mask_polygon'])*scale).astype(np.int32)],True,(255,120,0),1)
            x, y, w, h = [round(value*scale) for value in observation['bbox']]
            color = (60, 220, 80) if observation['track_status'] == 'confirmed' else (0, 180, 255)
            if observation.get('road_relation')=='beyond_central_boundary':color=(40,40,240)
            cv2.rectangle(tile, (x, y), (x+w, y+h), color, 1)
            confidence='REVIEW' if observation.get('confidence_type') else f"{observation['confidence']:.2f}"
            label = f"#{observation['track_id'].split(':')[-1]} {observation['kind']} {confidence}"
            cv2.putText(tile, label, (max(0, min(x, 290)), max(35, y-5)), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        for lane in lanes:
            cv2.polylines(tile,[np.round(np.array(lane['points'])*scale).astype(np.int32)],False,(255,0,255),2)
        if central_boundary and central_boundary.get('active'):
            cv2.polylines(tile,[np.round(np.asarray(central_boundary['points'])*scale).astype(np.int32)],False,(255,255,0),2)
        for signal in signals:
            x,y,w,h=[round(v*scale) for v in signal['bbox']]
            cv2.rectangle(tile,(x,y),(x+w,y+h),(0,230,255),1)
            cv2.putText(tile,signal['state'],(x,max(32,y-3)),0,.35,(0,230,255),1)
    cv2.putText(tile, f'{camera} {timestamp:.3f}s', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    return tile


def analyze(front_path, output, sample_hz=10., duration=10., start=0., detector=None,
            tracker_type='motion', progress=None, cancelled=None, make_video=False,review_reference=None,road_map=None,rear_calibration=None,road_detector=None,camera_calibrations=None):
    if not all(math.isfinite(v) for v in (sample_hz, duration, start)) or not 0 < sample_hz <= 30 or duration <= 0 or start < 0:
        raise ValueError('Use 0 < sample-hz <= 30, duration > 0, start >= 0')
    paths = discover_clip(front_path)
    destination = Path(output).resolve()
    if destination.exists():
        raise FileExistsError(f'Output already exists: {destination}')
    detector = detector or SceneDetector(include_traffic_lights=True)
    assist=ReviewedObservationAssist(review_reference,paths['front'].name) if review_reference else None
    signal_tracker=SignalStateTracker()
    junction_tracker=JunctionTracker()
    readers = {}
    try:
        for camera, path in paths.items():
            readers[camera] = CameraReader(path)
        front = readers['front']
        rectifiers={}
        lens_profiles = dict(camera_calibrations or {})
        if rear_calibration:
            if 'back' in lens_profiles:raise ValueError('Rear lens calibration supplied twice')
            lens_profiles['back'] = rear_calibration
        if lens_profiles:
            if set(lens_profiles)-set(readers):raise ValueError('Lens calibration supplied for missing or unknown camera')
            if assist:raise ValueError('Raw reviewed boxes cannot be mixed with rectified analysis')
            from core.scene_rectification import load_rectification
            for name, path in lens_profiles.items():
                reader=readers[name]
                rectifiers[name]=load_rectification(path,reader.width,reader.height)
        end = min(start + duration, front.count / front.fps)
        if start >= end:
            raise ValueError('Start is beyond the front video duration')
        decoder = RealTeslaSEIDecoder(str(paths['front']))
        from core.scene_sync import SequenceAlignment
        alignments = {}
        for camera in readers:
            target = decoder if camera == 'front' else RealTeslaSEIDecoder(str(paths[camera]))
            alignments[camera] = SequenceAlignment(decoder.telemetry_cache, target.telemetry_cache,
                                                  front.count, readers[camera].count)
        tracker_class = {'motion': MotionTracker, 'baseline': PixelTracker}[tracker_type]
        trackers = {camera: tracker_class(camera) for camera in readers}
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.ct-scene-', dir=destination.parent) as temporary:
            stage = Path(temporary) / 'result'
            stage.mkdir()
            metadata = {
                'schema_version': 1, 'status': 'experimental_observations',
                'vehicle': 'Model 3 2020 LR HW3 (user supplied; no calibration profile)',
                'sample_hz': sample_hz, 'start_s': start, 'end_s': end,
                'source_front_path':str(Path(front_path).resolve()),
                'model_sha256': hashlib.sha256(Path(detector.model_path).read_bytes()).hexdigest(),
                'camera_sync': 'nearest per-camera container PTS; nominal FPS fallback explicitly unverified; SEI counter is diagnostic only',
                'camera_sequence_sync': {name: item.report for name,item in alignments.items()},
                'tracking': tracker_type + '; camera-local only, no cross-camera fusion',
                'model_input_size': getattr(detector, 'input_size', None),
                'detection_threshold': getattr(detector, 'threshold', None),
                'coordinate_system': 'original image pixels; no metric positions',
                'lanes': 'image color/edge line segments, no map extrapolation', 'osm': 'not connected',
                'signals':'front detector + color support over three observations; unknown on glare',
                'review_assisted':bool(assist),
                'review_reference_sha256':hashlib.sha256(Path(review_reference).read_bytes()).hexdigest() if assist else None,
                'cameras': {key: {'file': path.name, 'fps': readers[key].fps,
                                  'width': readers[key].width, 'height': readers[key].height,
                                  'frame_count': readers[key].count} for key, path in paths.items()},
                'missing_cameras': [key for key in CAMERAS if key not in paths],
            }
            records = 0
            if road_detector:
                metadata['road_detector']=road_detector.info
                metadata['lanes']='camera segmentation curves; boundary semantics unverified'
            from core.scene_central_boundary import CentralBoundaryTracker
            from core.scene_fusion import cameras_for,load_profile
            boundary_trackers={key:CentralBoundaryTracker() for key in readers}
            for key in readers:
                metadata['cameras'][key]['timing']=readers[key].timing
                metadata['cameras'][key]['rectification']=rectifiers[key].metadata() if key in rectifiers else dict(status='not_applied_no_calibration',coordinate_space='original_pixels')
            if rectifiers:metadata['coordinate_system']='per-camera image pixels; consult rectification metadata; no metric positions'
            boundary_geometry=cameras_for(metadata,load_profile(metadata))
            if road_map:
                from core.scene_map import load_road_map
                metadata['road_map']=load_road_map(road_map)
                metadata['osm']='cached road geometry prior; not lane-level ground truth'
            with (stage / 'observations.jsonl').open('w', encoding='utf-8') as stream:
                for step in range(math.ceil((end-start)*sample_hz)):
                    if cancelled and cancelled():
                        raise AnalysisCancelled()
                    timestamp = start + step / sample_hz
                    if timestamp >= end:
                        break
                    row = {'time_s': timestamp, 'ego': None, 'cameras': {}}
                    tiles = []
                    reference_index = front.index_at(timestamp)
                    for camera, reader in readers.items():
                        index, frame = reader.read_at(timestamp)
                        if frame is not None and camera in rectifiers:
                            frame=rectifiers[camera].apply(frame)
                        detections=detector.detect(frame) if frame is not None else []
                        if camera in rectifiers:
                            valid=rectifiers[camera].valid
                            detections=[d for d in detections if valid[
                                int(np.clip(d.bbox[1]+d.bbox[3]/2,0,valid.shape[0]-1)),
                                int(np.clip(d.bbox[0]+d.bbox[2]/2,0,valid.shape[1]-1))]]
                        signals=signal_tracker.update(frame,[d for d in detections if d.kind=='traffic_light'],timestamp) if camera=='front' and frame is not None else []
                        tracker=trackers[camera];vehicle_detections=[d for d in detections if d.kind!='traffic_light']
                        objects=tracker.update(vehicle_detections,timestamp,frame=frame) if isinstance(tracker,MotionTracker) else tracker.update(vehicle_detections,timestamp)
                        from core.scene_analysis import iou
                        for obj in objects:
                            masked=[d for d in detections if getattr(d,'mask_polygon',None)]
                            best=max(masked,key=lambda d:iou(obj['bbox'],d.bbox),default=None)
                            if best is not None and iou(obj['bbox'],best.bbox)>.5:
                                obj['mask_polygon']=best.mask_polygon
                        review_log=[]
                        if assist and frame is not None:
                            objects,review_signals,review_log=assist.update(camera,frame,timestamp,objects)
                            if camera=='front' and review_signals: signals=review_signals
                        lanes=detect_lane_segments(frame,objects) if camera=='front' and frame is not None else []
                        source_time = (float(reader.timestamps[index]) if reader.timestamps is not None else index/reader.fps) if frame is not None else None
                        row['cameras'][camera] = {'frame_index': index, 'source_time_s': source_time,
                                                 'synchronization': reader.timing['status'],
                                                 'requested_time_error_ms': (source_time-timestamp)*1000 if source_time is not None else None,
                                                 'reference_frame_index': reference_index,
                                                 'available': frame is not None, 'objects': objects,
                                                 'raw_detections':[vars(d) for d in detections],
                                                 'lanes':lanes,'signals':signals,'review_log':review_log}
                        row['cameras'][camera]['central_boundary']=boundary_trackers[camera].update(
                            frame,objects,boundary_geometry.get(camera) if camera!='back' or camera in rectifiers else None,timestamp)
                        if camera=='front' and frame is not None and road_detector:
                            evidence=road_detector.detect(frame,objects)
                            row['cameras'][camera].update(pitch_lanes=lanes,lanes=evidence['lanes'],road_evidence=evidence)
                            lanes=evidence['lanes']
                        if camera == 'front' and frame is not None:
                            row['cameras'][camera]['junction_evidence']=junction_tracker.update(junction_evidence(frame,objects),timestamp)
                            # Do not turn missing SEI into invented zero-speed telemetry.
                            telemetry = decoder.telemetry_cache.get(index)
                            if telemetry is not None:
                                row['ego'] = {'data': telemetry, 'source': 'existing_sei_decoder',
                                              'alignment': 'decoder frame index; not independently verified'}
                        tiles.append(diagnostic_tile(frame, camera, timestamp, objects,lanes,signals,
                            row['cameras'][camera].get('central_boundary')))
                    while len(tiles) % 2:
                        tiles.append(np.zeros((240, 426, 3), np.uint8))
                    grid = np.vstack([np.hstack(tiles[i:i+2]) for i in range(0, len(tiles), 2)])
                    ok, encoded = cv2.imencode('.jpg', grid)
                    if not ok:
                        raise OSError('Could not encode diagnostic image')
                    (stage / f'frame_{step:05d}.jpg').write_bytes(encoded.tobytes())
                    stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                    records += 1
                    if progress:
                        progress(min(95, int(records / math.ceil((end-start)*sample_hz) * 95)))
            if cancelled and cancelled():
                raise AnalysisCancelled()
            if make_video:
                import subprocess
                import imageio_ffmpeg
                command = [imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-framerate', str(sample_hz),
                           '-i', str(stage/'frame_%05d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                           '-movflags', '+faststart', str(stage/'diagnostic.mp4')]
                flags = subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0
                with tempfile.TemporaryFile() as errors:
                    process = subprocess.Popen(command, stderr=errors, stdout=subprocess.DEVNULL, creationflags=flags)
                    try:
                        while True:
                            if cancelled and cancelled():
                                raise AnalysisCancelled()
                            try:
                                code = process.wait(timeout=.1)
                                break
                            except subprocess.TimeoutExpired:
                                continue
                        if code:
                            errors.seek(0)
                            raise RuntimeError(errors.read(4000).decode('utf-8', errors='replace'))
                    finally:
                        if process.poll() is None:
                            process.kill()
                            process.wait()
            if cancelled and cancelled():
                raise AnalysisCancelled()
            metadata['sample_count'] = records
            metadata['inference']=getattr(detector,'inference_info',{'effective':'unknown'})
            (stage / 'manifest.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
            stage.rename(destination)
            if progress:
                progress(100)
        return metadata
    finally:
        for reader in readers.values():
            reader.close()
