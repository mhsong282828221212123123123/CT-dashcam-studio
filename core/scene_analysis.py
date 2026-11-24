"""Offline scene observations. Pixel tracks are NOT calibrated 3D positions."""
from dataclasses import dataclass, asdict
from pathlib import Path
import math

import cv2
import numpy as np

from core.utils import resource_path

CAMERAS = ('front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar')
CLASSES = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}


@dataclass
class Detection:
    kind: str
    confidence: float
    bbox: list  # x, y, width, height in original image pixels


def letterbox(frame, size=320):
    height, width = frame.shape[:2]
    scale = min(size / width, size / height)
    new_w, new_h = round(width * scale), round(height * scale)
    left, top = (size - new_w) // 2, (size - new_h) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[top:top + new_h, left:left + new_w] = cv2.resize(frame, (new_w, new_h))
    return canvas, (new_w / width, new_h / height, left, top)


class SceneDetector:
    """YOLOv8 COCO ONNX detector; model and input size must match."""
    def __init__(self, model_path=None, threshold=0.35, input_size=320, include_traffic_lights=False, backend='cpu', detail='standard', device_id=0):
        if not 0 < threshold <= 1:
            raise ValueError('confidence threshold must be in (0, 1]')
        self.threshold = threshold
        self.classes = dict(CLASSES)
        if include_traffic_lights: self.classes[9] = 'traffic_light'
        if input_size < 32 or input_size % 32:
            raise ValueError('input size must be a positive multiple of 32')
        self.input_size = input_size
        self.model_path = str(model_path or resource_path('models/yolov8n.onnx'))
        if detail not in ('standard','tiled'):raise ValueError('Unknown detail mode')
        from core.scene_inference import InferenceEngine
        self.engine=InferenceEngine(self.model_path,input_size,backend,device_id)
        self.detail=detail

    @property
    def inference_info(self):
        return dict(self.engine.info,detail=self.detail)

    def detect(self, frame):
        result=self._detect_frame(frame)
        if self.detail=='tiled':
            full=list(result)
            h,w=frame.shape[:2];tw,th=round(w*.60),round(h*.60)
            for x,y in ((0,0),(w-tw,0),(0,h-th),(w-tw,h-th)):
                for detection in self._detect_frame(frame[y:y+th,x:x+tw]):
                    bx,by,bw,bh=detection.bbox
                    # Truncated crop detections cannot replace a full object.
                    if (x>0 and bx<2) or (y>0 and by<2) or (x+tw<w and bx+bw>tw-2) or (y+th<h and by+bh>th-2):continue
                    detection.bbox=[bx+x,by+y,bw,bh]
                    contained=False
                    for old in full:
                        if vehicle_family(old.kind)!=vehicle_family(detection.kind):continue
                        ox,oy,ow,oh=old.bbox;nx,ny,nw,nh=detection.bbox
                        intersection=max(0,min(ox+ow,nx+nw)-max(ox,nx))*max(0,min(oy+oh,ny+nh)-max(oy,ny))
                        # Crop passes sometimes rediscover only a vehicle's roof.
                        if intersection/max(1,nw*nh)>.9 and nw*nh<ow*oh*.8:
                            contained=True;break
                    if not contained:result.append(detection)
            result=self.suppress(result)
        return result

    def _detect_frame(self,frame):
        padded, transform = letterbox(frame, self.input_size)
        blob=cv2.dnn.blobFromImage(padded, 1 / 255., (self.input_size, self.input_size), swapRB=True)
        return self.decode(self.engine.run(blob), frame.shape[:2], transform)

    def decode(self, output, shape, transform):
        classes = getattr(self, 'classes', CLASSES)
        if output.ndim != 3 or output.shape[0] != 1 or output.shape[1] != 84:
            raise ValueError(f'Unexpected YOLO output shape: {output.shape}')
        height, width = shape
        sx, sy, left, top = transform
        candidates = []
        for row in output[0].T:
            if not np.isfinite(row).all():
                continue
            cid = int(np.argmax(row[4:]))
            score = float(row[4 + cid])
            if cid not in classes or score < self.threshold:
                continue
            cx, cy, w, h = map(float, row[:4])
            x1 = max(0., min(float(width), (cx - w / 2 - left) / sx))
            y1 = max(0., min(float(height), (cy - h / 2 - top) / sy))
            x2 = max(0., min(float(width), (cx + w / 2 - left) / sx))
            y2 = max(0., min(float(height), (cy + h / 2 - top) / sy))
            if x2 - x1 >= 2 and y2 - y1 >= 2:
                candidates.append(Detection(classes[cid], score, [x1, y1, x2-x1, y2-y1]))
        return self.suppress(candidates)

    def suppress(self,candidates):
        classes=getattr(self,'classes',CLASSES)
        selected = []
        for family in sorted({vehicle_family(kind) for kind in classes.values()}):
            group = [d for d in candidates if vehicle_family(d.kind) == family]
            if group:
                indices = cv2.dnn.NMSBoxes([d.bbox for d in group], [d.confidence for d in group], self.threshold, .45)
                selected.extend(group[int(i)] for i in np.asarray(indices).reshape(-1))
        return selected


def iou(a, b):
    x, y = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[0]+a[2], b[0]+b[2]), min(a[1]+a[3], b[1]+b[3])
    intersection = max(0., right-x) * max(0., bottom-y)
    union = a[2]*a[3] + b[2]*b[3] - intersection
    return intersection / union if union > 0 else 0.


def create_scene_detector(model_path=None,*args,**kwargs):
    if model_path and '-seg' in Path(model_path).stem.lower():
        from core.scene_instance_model import InstanceSceneDetector
        return InstanceSceneDetector(model_path,*args,**kwargs)
    return SceneDetector(model_path,*args,**kwargs)


class PixelTracker:
    """Conservative camera-local IoU baseline; never emits unobserved objects.

    IDs can change after occlusion/crossing. No cross-camera identity claim.
    """
    def __init__(self, camera, max_gap=.6, min_iou=.25):
        self.camera, self.max_gap, self.min_iou = camera, max_gap, min_iou
        self.tracks = {}
        self.next_id = 1
        self.last_time = None

    def update(self, detections, timestamp):
        if not math.isfinite(timestamp) or (self.last_time is not None and timestamp <= self.last_time):
            raise ValueError('Tracking requires strictly increasing finite timestamps')
        self.last_time = timestamp
        self.tracks = {k: v for k, v in self.tracks.items() if timestamp - v[1] <= self.max_gap}
        pairs = sorted(((iou(old.bbox, new.bbox), key, index)
                        for key, (old, _, _) in self.tracks.items()
                        for index, new in enumerate(detections) if old.kind == new.kind),
                       key=lambda pair: (-pair[0], pair[1], pair[2]))
        assignments, used = {}, set()
        for score, key, index in pairs:
            if score >= self.min_iou and key not in used and index not in assignments:
                assignments[index] = key
                used.add(key)
        result = []
        for index, detection in enumerate(detections):
            key = assignments.get(index)
            if key is None:
                key = self.next_id
                self.next_id += 1
            hits = self.tracks[key][2] + 1 if key in self.tracks else 1
            self.tracks[key] = (detection, timestamp, hits)
            result.append({**asdict(detection), 'track_id': f'{self.camera}:{key}',
                           'track_status': 'confirmed' if hits >= 3 else 'tentative',
                           'observation_count': hits,
                           'identity_scope': 'camera_local', 'position_m': None,
                           'position_status': 'uncalibrated', 'heading_deg': None})
        return result


def vehicle_family(kind):
    return 'vehicle' if kind in ('car', 'truck', 'bus') else kind


def optimal_pairs(scores):
    """Maximum-weight one-to-one matching, with zero-weight dummy matches.

    Hungarian shortest augmenting path implementation; no SciPy dependency.
    Non-positive scores are forbidden. Returns (row, column) pairs.
    """
    scores = np.asarray(scores, dtype=float)
    if scores.size == 0:
        return []
    rows, cols = scores.shape
    size = rows + cols
    cost = np.zeros((size, size))
    cost[:rows, :cols] = -np.maximum(scores, 0)
    u, v = np.zeros(size+1), np.zeros(size+1)
    p, way = np.zeros(size+1, dtype=int), np.zeros(size+1, dtype=int)
    for i in range(1, size+1):
        p[0] = i
        j0 = 0
        minimum = np.full(size+1, np.inf)
        used = np.zeros(size+1, dtype=bool)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], np.inf, 0
            for j in range(1, size+1):
                if not used[j]:
                    current = cost[i0-1, j-1] - u[i0] - v[j]
                    if current < minimum[j]:
                        minimum[j], way[j] = current, j0
                    if minimum[j] < delta:
                        delta, j1 = minimum[j], j
            for j in range(size+1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minimum[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    return [(p[j]-1, j-1) for j in range(1, cols+1)
            if 0 < p[j] <= rows and scores[p[j]-1, j-1] > 0]


class MotionTracker:
    """Two-stage pixel tracking, not world velocity or cross-camera identity.

    High confidence detections seed tracks; weaker detections can continue an
    existing track. Three high-confidence observations confirm it. Missing
    observations are never rendered as real objects.
    """
    def __init__(self, camera, new_threshold=.55, low_threshold=.25, max_gap=.8):
        self.camera = camera
        self.new_threshold, self.low_threshold, self.max_gap = new_threshold, low_threshold, max_gap
        self.tracks, self.next_id, self.last_time = {}, 1, None

    def update(self, detections, timestamp, frame=None):
        if not math.isfinite(timestamp) or (self.last_time is not None and timestamp <= self.last_time):
            raise ValueError('Tracking requires strictly increasing finite timestamps')
        self.last_time = timestamp
        from core.scene_appearance import signature,agreement
        appearance=[signature(frame,d.bbox) for d in detections]
        self.tracks = {key: state for key, state in self.tracks.items() if timestamp-state['time'] <= self.max_gap}
        high = [i for i, d in enumerate(detections) if d.confidence >= self.new_threshold]
        low = [i for i, d in enumerate(detections) if self.low_threshold <= d.confidence < self.new_threshold]
        assignments, used = {}, set()
        for indices in (high, low):
            keys = [key for key in self.tracks if key not in used]
            scores = np.zeros((len(keys), len(indices)))
            for row, key in enumerate(keys):
                state = self.tracks[key]
                predicted = np.array(state['detection'].bbox, dtype=float)
                predicted[:2] += state['velocity'] * (timestamp-state['time'])
                for col, index in enumerate(indices):
                    new = detections[index]
                    if vehicle_family(new.kind) != vehicle_family(state['detection'].kind):
                        continue
                    same_colour,similarity=agreement(state.get('appearance'),appearance[index])
                    if state.get('appearance_hits',0)>=3 and not same_colour:continue
                    overlap = iou(predicted, new.bbox)
                    # A displacement gate admits fast motion before velocity is known.
                    old_center = predicted[:2] + predicted[2:]/2
                    new_box = np.array(new.bbox)
                    distance = np.linalg.norm(new_box[:2]+new_box[2:]/2-old_center)
                    scale = max(1., np.linalg.norm(predicted[2:]))
                    area_ratio = (new_box[2]*new_box[3]) / max(1., predicted[2]*predicted[3])
                    ratios=new_box[2:]/np.maximum(predicted[2:],1.)
                    # Similar area can hide a small distant car switching to a
                    # tall narrow sliver of a newly entering adjacent vehicle.
                    shape_consistent=bool(np.all((ratios>=.4)&(ratios<=2.5)))
                    if shape_consistent and .4 <= area_ratio <= 2.5 and (overlap >= .1 or distance/scale < .65):
                        score=.7*overlap + .3*max(0., 1-distance/scale)
                        scores[row,col]=score*(.85+.15*similarity)
            for row, col in optimal_pairs(scores):
                assignments[indices[col]] = keys[row]
                used.add(keys[row])
        observations = []
        for index, detection in enumerate(detections):
            key = assignments.get(index)
            if key is None and index not in high:
                continue
            if key is None:
                key, self.next_id = self.next_id, self.next_id+1
                state = {'velocity': np.zeros(2), 'hits': 0, 'high_hits': 0, 'votes': {}}
            else:
                state = self.tracks[key]
                dt = timestamp-state['time']
                old = np.array(state['detection'].bbox)
                new = np.array(detection.bbox)
                measured_velocity = (new[:2]+new[2:]/2-old[:2]-old[2:]/2)/dt
                state['velocity'] = .7*measured_velocity + .3*state['velocity']
            state['hits'] += 1
            state['high_hits'] += detection.confidence >= self.new_threshold
            state['votes'][detection.kind] = state['votes'].get(detection.kind, 0.) + detection.confidence
            feature=appearance[index]
            if feature is not None:
                if state.get('appearance') is None:state['appearance']=feature
                else:
                    state['appearance']={k:(np.asarray(state['appearance'][k])*.95+np.asarray(feature[k])*.05).tolist()
                                         for k in ('hist','quantiles')}
                state['appearance_hits']=state.get('appearance_hits',0)+1
            state.update(detection=detection, time=timestamp)
            self.tracks[key] = state
            observations.append({**asdict(detection), 'kind': max(state['votes'], key=state['votes'].get),
                                 'detected_kind': detection.kind,
                                 'track_id': f'{self.camera}:{key}',
                                 'track_status': 'confirmed' if state['high_hits'] >= 3 else 'tentative',
                                 'observation_count': state['hits'], 'identity_scope': 'camera_local',
                                 'position_m': None, 'position_status': 'uncalibrated', 'heading_deg': None})
        return observations


def discover_clip(front_path):
    front = Path(front_path).resolve()
    if not front.name.endswith('-front.mp4') or not front.is_file():
        raise ValueError('Select an original TeslaCam *-front.mp4 file')
    prefix = front.name[:-len('-front.mp4')]
    return {camera: front.with_name(f'{prefix}-{camera}.mp4')
            for camera in CAMERAS if front.with_name(f'{prefix}-{camera}.mp4').is_file()}


class CameraReader:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(str(path))
        try:
            self.fps = float(self.cap.get(cv2.CAP_PROP_FPS))
            self.count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if not self.cap.isOpened() or not math.isfinite(self.fps) or not 1 <= self.fps <= 120 or self.count <= 0:
                raise ValueError(f'Invalid video metadata: {path}')
        except Exception:
            self.cap.release()
            raise
        self.next_index = 0
        self.cached_index, self.cached_frame = None, None
        from core.scene_sync import presentation_times
        self.timestamps, self.timing = presentation_times(path,self.count)
        self.pts_end = (self.timestamps[-1]+np.median(np.diff(self.timestamps))/2
                        if self.timestamps is not None else None)

    def index_at(self,time_s):
        if self.timestamps is None:
            return int(math.floor(time_s*self.fps+.5))
        if time_s<0 or time_s>self.pts_end:
            return -1
        right=int(np.searchsorted(self.timestamps,time_s,side='left'))
        if right==0:return 0
        if right>=self.count:return self.count-1
        return right if time_s>=(self.timestamps[right-1]+self.timestamps[right])/2 else right-1

    def read_at(self, time_s):
        index = self.index_at(time_s)
        return self.read_index(index)

    def read_index(self, index):
        """Read an explicit decoded-frame index, e.g. a verified SEI match."""
        index = int(index)
        if index < 0 or index >= self.count:
            return index, None
        if index == self.cached_index:
            return index, self.cached_frame
        if index < self.next_index:
            raise ValueError('CameraReader requires forward sampling')
        while self.next_index < index:
            if not self.cap.grab():
                return index, None
            self.next_index += 1
        ok, frame = self.cap.read()
        self.next_index += 1
        self.cached_index, self.cached_frame = index, frame if ok else None
        return index, self.cached_frame

    def close(self):
        self.cap.release()
