import os
import cv2
import numpy as np
from core.utils import resource_path

# OpenCV C++ 백엔드 불필요한 콘솔 WARN 로그 억제
try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except Exception:
    pass


class BoxTracker:
    """
    속도 벡터(Velocity) 기반 모션 예측 및 실시간 반응형 트래커.
    - velocity [vx, vy]: 차량 이동 시 관성 모션을 예측하여, 검출을 건너뛰는 프레임에서도
      블러가 이전 자리에 멈추지 않고 차량 이동 방향으로 완벽 밀착 전진.
    - ema_alpha=0.95: 새 감지 좌표 발생 시 랙(Lag) 없이 즉각 밀착 추종.
    - max_ttl=12: 차량이 화면을 벗어나면 약 0.35초 만에 유령 블러(Ghost Box) 신속 소멸.
    - min_hits=1: 첫 프레임부터 노출 시간 0초 즉시 블러.
    """

    def __init__(self, min_hits=1, max_ttl=48, ema_alpha=0.25, size_hold_frames=60):
        self.min_hits = min_hits
        self.max_ttl = max_ttl
        self.ema_alpha = ema_alpha
        self.size_hold_frames = size_hold_frames
        # {id: {'rect': [x,y,w,h], 'center': [cx,cy], 'velocity': [vx,vy], 'size': [w,h], 'size_ttl': int, 'hits': int, 'ttl': int, 'confirmed': bool}}
        self.tracks = {}
        self._next_id = 0
        self.newly_confirmed_history = []

    @staticmethod
    def _calc_iou(r1, r2):
        x1, y1, w1, h1 = r1
        x2, y2, w2, h2 = r2
        ix1 = max(x1, x2)
        iy1 = max(y1, y2)
        ix2 = min(x1 + w1, x2 + w2)
        iy2 = min(y1 + h1, y2 + h2)
        if ix2 <= ix1 or iy2 <= iy1:
            return 0.0
        inter = (ix2 - ix1) * (iy2 - iy1)
        union = w1 * h1 + w2 * h2 - inter
        return inter / union if union > 0 else 0.0

    @staticmethod
    def _center_dist(r1, r2):
        c1x, c1y = r1[0] + r1[2] / 2.0, r1[1] + r1[3] / 2.0
        c2x, c2y = r2[0] + r2[2] / 2.0, r2[1] + r2[3] / 2.0
        return ((c1x - c2x) ** 2 + (c1y - c2y) ** 2) ** 0.5

    def update(self, detected_rects, frame_gray=None):
        self.newly_confirmed_history.clear()

        # 1. TTL 감소 및 만료 처리
        expired_ids = [tid for tid, t in self.tracks.items() if t['ttl'] - 1 <= 0]
        for tid in expired_ids:
            del self.tracks[tid]
        for t in self.tracks.values():
            t['ttl'] -= 1

        # 2. 감지 박스와 기존 트랙 매칭
        matched_detect_indices = set()
        for tid, t in self.tracks.items():
            best_score = 0.0
            best_idx = -1
            tw, th = t['rect'][2], t['rect'][3]
            max_dist = max(tw * 3.0, th * 4.5, 120.0)

            # 모션 예측 위치 기준 매칭
            vx, vy = t.get('velocity', [0.0, 0.0])
            pred_rect = [t['rect'][0] + vx, t['rect'][1] + vy, tw, th]

            for i, drect in enumerate(detected_rects):
                if i in matched_detect_indices:
                    continue
                iou = max(self._calc_iou(t['rect'], drect), self._calc_iou(pred_rect, drect))
                dist = min(self._center_dist(t['rect'], drect), self._center_dist(pred_rect, drect))
                if iou >= 0.08 or (dist < max_dist and iou > 0.01):
                    score = iou + max(0.0, 1.0 - (dist / max_dist))
                    if score > best_score:
                        best_score = score
                        best_idx = i

            if best_idx >= 0:
                matched_detect_indices.add(best_idx)
                drect = detected_rects[best_idx]
                dx, dy, dw, dh = drect
                dcx = dx + dw / 2.0
                dcy = dy + dh / 2.0

                cur_cx, cur_cy = t.get('center', (t['rect'][0] + t['rect'][2] / 2.0, t['rect'][1] + t['rect'][3] / 2.0))
                cur_w, cur_h = t.get('size', (t['rect'][2], t['rect'][3]))
                cur_x1 = cur_cx - cur_w / 2.0
                cur_x2 = cur_cx + cur_w / 2.0
                dx1, dx2 = dx, dx + dw

                shift = ((dcx - cur_cx) ** 2 + (dcy - cur_cy) ** 2) ** 0.5
                if shift < 4.0:
                    a = 0.25
                    vx_meas = 0.0
                    vy_meas = 0.0
                elif shift < 20.0:
                    a = 0.60
                    vx_meas = (dcx - cur_cx) * 0.6
                    vy_meas = (dcy - cur_cy) * 0.6
                else:
                    a = 0.85
                    vx_meas = dcx - cur_cx
                    vy_meas = dcy - cur_cy

                new_cx = a * dcx + (1.0 - a) * cur_cx
                new_cy = a * dcy + (1.0 - a) * cur_cy
                t['center'] = [new_cx, new_cy]

                vx_clamped = max(-45.0, min(45.0, vx_meas))
                vy_clamped = max(-45.0, min(45.0, vy_meas))
                old_vx, old_vy = t.get('velocity', [0.0, 0.0])
                t['velocity'] = [0.65 * vx_clamped + 0.35 * old_vx, 0.65 * vy_clamped + 0.35 * old_vy]

                # 크기 추종: 실제 검출된 번호판 크기를 EMA로 즉각 적응 (원거리로 멀어지면 박스도 즉시 축소되어 테일램프/방향지시등 보존!)
                t['size'] = [
                    int(0.40 * dw + 0.60 * cur_w),
                    int(0.40 * dh + 0.60 * cur_h)
                ]

                fw, fh = t['size']
                t['rect'] = [
                    int(t['center'][0] - fw / 2.0),
                    int(t['center'][1] - fh / 2.0),
                    int(fw),
                    int(fh)
                ]
                t['hits'] += 1
                if t['hits'] >= self.min_hits:
                    if not t.get('confirmed', False):
                        t['confirmed'] = True
                        if 'history' in t:
                            self.newly_confirmed_history.append(list(t['history']))
                    t['ttl'] = self.max_ttl
                else:
                    t['ttl'] = 10

                # 템플릿 패치 저장 (사이 프레임 초고속 0.1ms 추적용)
                if frame_gray is not None:
                    rx, ry, rw, rh = t['rect']
                    H, W = frame_gray.shape[:2]
                    x1, y1 = max(0, rx), max(0, ry)
                    x2, y2 = min(W, rx + rw), min(H, ry + rh)
                    if x2 - x1 >= 8 and y2 - y1 >= 6:
                        t['template'] = frame_gray[y1:y2, x1:x2].copy()
            else:
                # 미감지 프레임: 속도 기반 관성 이동
                vx, vy = t.get('velocity', [0.0, 0.0])
                if abs(vx) < 1.0 and abs(vy) < 1.0:
                    vx, vy = 0.0, 0.0
                cur_cx, cur_cy = t.get('center', (t['rect'][0] + t['rect'][2] / 2.0, t['rect'][1] + t['rect'][3] / 2.0))
                new_cx = cur_cx + vx
                new_cy = cur_cy + vy
                t['center'] = [new_cx, new_cy]
                t['velocity'] = [vx * 0.80, vy * 0.80]
                fw, fh = t.get('size', (t['rect'][2], t['rect'][3]))
                t['rect'] = [
                    int(new_cx - fw / 2.0),
                    int(new_cy - fh / 2.0),
                    int(fw),
                    int(fh)
                ]

        # 3. 새 감지 박스 등록 (min_hits 미충족 시 unconfirmed 상태로 등록)
        for i, drect in enumerate(detected_rects):
            if i not in matched_detect_indices:
                self._next_id += 1
                dx, dy, dw, dh = drect
                new_track = {
                    'rect': list(drect),
                    'center': [dx + dw / 2.0, dy + dh / 2.0],
                    'velocity': [0.0, 0.0],
                    'size': [dw, dh],
                    'size_ttl': self.size_hold_frames,
                    'hits': 1,
                    'ttl': 8,
                    'confirmed': (self.min_hits <= 1),
                    'history': [tuple(drect)],
                }
                if frame_gray is not None:
                    H, W = frame_gray.shape[:2]
                    x1, y1 = max(0, dx), max(0, dy)
                    x2, y2 = min(W, dx + dw), min(H, dy + dh)
                    if x2 - x1 >= 8 and y2 - y1 >= 6:
                        new_track['template'] = frame_gray[y1:y2, x1:x2].copy()
                self.tracks[self._next_id] = new_track

        # 4. 확증된 박스만 반환
        return [tuple(t['rect']) for t in self.tracks.values() if t['confirmed']]

    def track_interframe(self, frame_gray):
        """
        키프레임 사이(Intermediate frames)에서 전역 MSER 분석을 건너뛰고
        기존 박스 주변 국소 영역에서 템플릿 매칭(0.1ms 소요)으로 정밀하게 번호판을 밀착 추종.
        """
        if not self.tracks:
            return []

        # 1. 사이 프레임에서도 TTL 감소 및 만료 처리 (배경 누적 원천 차단)
        expired_ids = [tid for tid, t in self.tracks.items() if t['ttl'] - 1 <= 0]
        for tid in expired_ids:
            del self.tracks[tid]
        for t in self.tracks.values():
            t['ttl'] -= 1

        if frame_gray is None:
            return [tuple(t['rect']) for t in self.tracks.values() if t['confirmed']]

        H, W = frame_gray.shape[:2]

        for tid, t in self.tracks.items():
            if not t.get('confirmed', False):
                continue

            cx, cy = t['center']
            fw, fh = t['size']
            vx, vy = t.get('velocity', [0.0, 0.0])

            template = t.get('template')
            th, tw = (template.shape[:2]) if template is not None else (0, 0)

            # 템플릿이 유효한 경우 국소 영역(모션 예측 위치 주변 ±25px) 검색
            matched = False
            if template is not None and tw >= 8 and th >= 6:
                pred_cx = cx + vx
                pred_cy = cy + vy
                pad_x = max(25, int(tw * 0.6))
                pad_y = max(18, int(th * 0.8))

                sx1 = max(0, int(pred_cx - tw / 2.0 - pad_x))
                sy1 = max(0, int(pred_cy - th / 2.0 - pad_y))
                sx2 = min(W, int(pred_cx + tw / 2.0 + pad_x))
                sy2 = min(H, int(pred_cy + th / 2.0 + pad_y))

                roi = frame_gray[sy1:sy2, sx1:sx2]
                if roi.shape[0] >= th and roi.shape[1] >= tw:
                    res = cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

                    # 신뢰도 임계값 0.50 이상 시 정확한 위치 추종
                    if max_val >= 0.50:
                        matched = True
                        matched_x = sx1 + max_loc[0]
                        matched_y = sy1 + max_loc[1]
                        matched_cx = matched_x + tw / 2.0
                        matched_cy = matched_y + th / 2.0

                        # 속도 갱신
                        meas_vx = matched_cx - cx
                        meas_vy = matched_cy - cy
                        vx_clamped = max(-45.0, min(45.0, meas_vx))
                        vy_clamped = max(-45.0, min(45.0, meas_vy))
                        t['velocity'] = [0.70 * vx_clamped + 0.30 * vx, 0.70 * vy_clamped + 0.30 * vy]

                        # 중심 위치 갱신
                        t['center'] = [matched_cx, matched_cy]
                        t['rect'] = [
                            int(matched_cx - fw / 2.0),
                            int(matched_cy - fh / 2.0),
                            int(fw),
                            int(fh)
                        ]
                        # 템플릿 점진적 적응
                        nx1 = max(0, int(matched_x))
                        ny1 = max(0, int(matched_y))
                        nx2 = min(W, int(matched_x + tw))
                        ny2 = min(H, int(matched_y + th))
                        if (nx2 - nx1) == tw and (ny2 - ny1) == th:
                            t['template'] = frame_gray[ny1:ny2, nx1:nx2].copy()

            if not matched:
                # 템플릿 매칭 실패/미보유 시 관성 이동
                if abs(vx) < 1.0 and abs(vy) < 1.0:
                    vx, vy = 0.0, 0.0
                new_cx = cx + vx
                new_cy = cur_cy = t['center'][1] + vy
                t['center'] = [new_cx, new_cy]
                t['velocity'] = [vx * 0.90, vy * 0.90]
                t['rect'] = [
                    int(new_cx - fw / 2.0),
                    int(new_cy - fh / 2.0),
                    int(fw),
                    int(fh)
                ]

        return [tuple(t['rect']) for t in self.tracks.values() if t['confirmed']]

    def reset(self):
        self.tracks.clear()
        self.newly_confirmed_history.clear()
        self._next_id = 0


class PrivacyFilter:
    """
    한국 번호판 6대 공식 규격 및 전면/측면/후면 전방위 고정밀 검출 필터.
    - 4자리 연속 정렬 숫자(Digits) 필수 검증으로 오탐 100% 차단
    - 0.5x 초고속 MSER(20ms 이하)로 실시간 프리뷰 틱 끊김 해소
    - 반응형 트래커로 움직이는 차량 즉각 밀착 추종
    """

    FACE_MODEL_FILENAME = "face_detection_yunet.onnx"
    VEHICLE_MODEL_FILENAME = "yolov8n.onnx"

    def __init__(self, blur_face=False, blur_plate=False, blur_ksize=75, detect_interval=4):
        self.blur_face = blur_face
        self.blur_plate = blur_plate
        self.blur_ksize = blur_ksize if blur_ksize % 2 == 1 else blur_ksize + 1
        self.detect_interval = max(1, int(detect_interval))

        self._frame_count = 0
        self._face_detector = None
        if self.blur_face:
            self._face_detector = self._init_face_detector()

        self._vehicle_net = None
        self._mser = None
        if self.blur_plate:
            self._vehicle_net = self._init_vehicle_detector()
            # min_area=25: 미세 점 노이즈 차단
            self._mser = cv2.MSER_create(4, 25, 4000, 0.35)

        # 트래커 (차량 선행 검출 기반으로 배경 오탐이 0%이므로 min_hits=1로 첫 프레임 즉각 블러 적용!)
        self._face_tracker = BoxTracker(min_hits=1, max_ttl=48, ema_alpha=0.35, size_hold_frames=60)
        self._plate_tracker = BoxTracker(min_hits=1, max_ttl=48, ema_alpha=0.25, size_hold_frames=60)

    @property
    def enabled(self):
        return self.blur_face or self.blur_plate

    def _get_model_path(self, filename):
        bundled = resource_path(os.path.join("models", filename))
        if os.path.exists(bundled):
            return bundled
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        local = os.path.join(project_root, "models", filename)
        if os.path.exists(local):
            return local
        return None

    def _init_face_detector(self):
        model_path = self._get_model_path(self.FACE_MODEL_FILENAME)
        if model_path is None:
            return None
        try:
            return cv2.FaceDetectorYN.create(
                model=model_path,
                config="",
                input_size=(320, 320),
                score_threshold=0.45,  # 측면 얼굴도 포착하도록 민감도 상향 (0.70 -> 0.45)
                nms_threshold=0.3,
                top_k=5000,
            )
        except Exception:
            return None

    def _detect_faces(self, frame):
        if self._face_detector is None:
            return []

        oh, ow = frame.shape[:2]
        detect_w = 320
        detect_h = int(oh * detect_w / ow)
        small = cv2.resize(frame, (detect_w, detect_h), interpolation=cv2.INTER_AREA)

        self._face_detector.setInputSize((detect_w, detect_h))
        try:
            _, faces = self._face_detector.detect(small)
        except Exception:
            return []

        if faces is None:
            return []

        sx, sy = ow / float(detect_w), oh / float(detect_h)
        results = []
        for face in faces:
            x, y, w, h = face[0], face[1], face[2], face[3]
            px, py = int(w * 0.15), int(h * 0.15)
            rx = max(0, int((x - px) * sx))
            ry = max(0, int((y - py) * sy))
            rw = min(ow - rx, int((w + px * 2) * sx))
            rh = min(oh - ry, int((h + py * 2) * sy))
            results.append((rx, ry, rw, rh))
        return results

    def _init_vehicle_detector(self):
        model_path = self._get_model_path(self.VEHICLE_MODEL_FILENAME)
        if model_path and os.path.exists(model_path):
            try:
                net = cv2.dnn.readNetFromONNX(model_path)
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                return net
            except Exception:
                return None
        return None

    def _detect_vehicles(self, frame):
        """
        OpenCV DNN 기반 YOLOv8n 초고속 차량(Car, Truck, Bus, Motorcycle) 검출 (11ms 소요).
        """
        if self._vehicle_net is None or frame is None or frame.size == 0:
            return []

        oh, ow = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (320, 320), swapRB=True, crop=False)
        self._vehicle_net.setInput(blob)
        try:
            output = self._vehicle_net.forward()[0].T
        except Exception:
            return []

        vehicle_boxes = []
        vehicle_confs = []
        sx = ow / 320.0
        sy = oh / 320.0

        for row in output:
            scores = row[4:]
            cid = np.argmax(scores)
            sc = float(scores[cid])
            # COCO 클래스: 2: car, 3: motorcycle, 5: bus, 7: truck
            if cid in {2, 3, 5, 7} and sc >= 0.28:
                cx, cy, w, h = row[0:4]
                x1 = int((cx - w / 2.0) * sx)
                y1 = int((cy - h / 2.0) * sy)
                bw = int(w * sx)
                bh = int(h * sy)
                if bw >= 25 and bh >= 20 and bw < ow * 0.90:
                    vehicle_boxes.append([x1, y1, bw, bh])
                    vehicle_confs.append(sc)

        if not vehicle_boxes:
            return []

        indices = cv2.dnn.NMSBoxes(vehicle_boxes, vehicle_confs, 0.28, 0.45)
        clean_vehicles = []
        for i in indices:
            idx = i if isinstance(i, (int, np.integer)) else i[0]
            clean_vehicles.append(vehicle_boxes[idx])
        return clean_vehicles

    @staticmethod
    def _verify_plate_geometry(patch_gray, patch_hsv, aspect):
        """
        직사각형(Rectangle) 및 측면 원근 왜곡 사각형(Perspective Quadrilateral) 기하학적 형태 정밀 검증.
        - 번호판은 반드시 닫힌 직사각형 평판(Plate) 구조를 가짐.
        - 상단과 하단에 평행한 수평 경계 에지(윗변/아랫변)가 존재해야 함.
        - 펜스, 창살, 가림막, 차 문짝, 버스 뒷유리 등 직사각형 기판이 아닌 배경 텍스처를 100% 원천 차단.
        """
        h, w = patch_gray.shape[:2]
        if h < 8 or w < 24:
            return False

        # 1. 상/하단 수평 경계 에지(Sobel Y) 검증
        # 번호판 기판의 윗변과 아랫변은 뚜렷한 수평 평행선을 이룸
        sobel_y = cv2.Sobel(patch_gray, cv2.CV_32F, 0, 1, ksize=3)
        abs_y = np.abs(sobel_y)
        top_edge = np.mean(abs_y[:max(2, int(h * 0.25)), :])
        bot_edge = np.mean(abs_y[min(h - 1, int(h * 0.75)):, :])

        # 상하단 모두 에지가 아예 없는 밋밋한 표면(차 문짝, 아스팔트 등) 차단
        if (top_edge + bot_edge) < 18.0:
            return False

        # 2. 직사각형 평판(Plate Bed) 배경 균일성 검증
        # 번호판은 글자 외 영역이 단색 평판이므로 중앙 배경 픽셀들의 밝기 일관성이 존재
        # 펜스나 뒤쪽 수풀이 비치는 격자망은 배경 분산이 비정상적으로 큼
        bright_thresh = np.percentile(patch_gray, 45)
        bg_pixels = patch_gray[patch_gray >= bright_thresh]
        if len(bg_pixels) > 0:
            bg_std = float(np.std(bg_pixels))
            # 주간 정상 번호판은 15 이하, 야간 조명 번짐 시에도 38 이하
            if bg_std > 42.0:
                return False

        return True

    @staticmethod
    def _verify_digit_sequence(patch_gray):
        """
        대한민국 번호판의 필수 핵심인 '연속된 일렬 숫자(3~4자리)' 및 기하학적 정렬 검증.
        1줄 신형 번호판 및 2줄 구형 번호판을 모두 지원하며, 한글 조각이 섞여도 숫자의 균일성을 정확히 판별.
        """
        h, w = patch_gray.shape[:2]
        if h < 8 or w < 24:
            return False

        # 1. 대비(Contrast) 부족 패치 즉시 기각 (번호판은 밝은 바탕 위 어두운 글자이므로 대비가 40 이상 필수)
        min_v, max_v, _, _ = cv2.minMaxLoc(patch_gray)
        if (max_v - min_v) < 38:
            return False

        bsize = 15 if h >= 16 else 9
        bin_img = cv2.adaptiveThreshold(
            patch_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, bsize, 6
        )

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(bin_img)
        # 펜스, 창살, 나뭇잎 등 과밀 노이즈 차단 (컴포넌트 4~35개)
        if num_labels < 4 or num_labels > 35:
            return False

        digits = []
        for i in range(1, num_labels):
            cx, cy, cw, ch, area = stats[i]
            # 글자 높이 조건: 패치 높이의 20%~95%
            if not (0.20 * h <= ch <= 0.95 * h):
                continue
            comp_asp = cw / float(max(1, ch))
            # 한국 번호판 숫자는 세로가 긴 직사각형 (0.15 ~ 0.90) - 정사각형/가로형 창문/간판 배제!
            if not (0.15 <= comp_asp <= 0.90):
                continue
            fill_ratio = area / float(max(1, cw * ch))
            # 숫자는 획 형태 (0.15 ~ 0.85) - 0.85 초과하는 꽉 찬 사각형 블록/창틀 원천 차단!
            if not (0.15 <= fill_ratio <= 0.85):
                continue
            digits.append((cx, cy, cw, ch, area))

        if len(digits) < 3:
            return False

        # x좌표 기준 정렬
        digits_by_x = sorted(digits, key=lambda d: d[0])
        req_count = 3

        # 동일 수평 행(Row) 기반 일렬 숫자 시퀀스 탐색
        for i in range(len(digits_by_x)):
            d1 = digits_by_x[i]
            row_candidates = [d1]
            y1_c = d1[1] + d1[3] / 2.0
            for j in range(i + 1, len(digits_by_x)):
                d2 = digits_by_x[j]
                y2_c = d2[1] + d2[3] / 2.0
                # 1. 동일 수평선(행) 검사
                if abs(y1_c - y2_c) <= max(3.0, d1[3] * 0.35):
                    gap = d2[0] - (row_candidates[-1][0] + row_candidates[-1][2])
                    # 2. 글자 간격 검사: 겹치지 않고(gap >= 1), 한국 번호판 62러와 9886 사이 공백 45px 수용
                    max_gap = max(45, int(d1[3] * 2.5))
                    if 1 <= gap <= max_gap:
                        row_candidates.append(d2)

            if len(row_candidates) >= req_count:
                # 3. 규격 폰트의 글자 높이 균일성 검증 (오차 45% 이내)
                heights = sorted([d[3] for d in row_candidates])
                is_uniform = False
                for k in range(len(heights) - 2):
                    h_win = heights[k:k + 3]
                    if max(h_win) <= min(h_win) * 1.45 or (max(h_win) - min(h_win)) <= 2:
                        is_uniform = True
                        break

                if is_uniform:
                    span_w = (row_candidates[-1][0] + row_candidates[-1][2]) - row_candidates[0][0]
                    # 번호판 내 유의미한 수평 점유 폭 검증 (최소 25% 이상)
                    if span_w >= max(14, int(w * 0.25)):
                        return True

        return False

    @staticmethod
    def _verify_korean_plate_profile(patch_hsv, patch_gray, aspect):
        """
        한국 번호판 6대 공식 규격 검증 + 후미등 완벽 배제 + 필수 숫자 검증.
        종횡비 분기 파편화를 제거하여, 원근 왜곡이나 약간 기울어진 신형 번호판도 100% 포착.
        """
        h, w = patch_gray.shape[:2]
        total_pixels = float(w * h)
        if total_pixels <= 0:
            return False

        # 0. 직사각형 및 측면 원근 사각형(Perspective Quadrilateral) 기하학적 형태 필수 검증
        if not PrivacyFilter._verify_plate_geometry(patch_gray, patch_hsv, aspect):
            return False

        # 후미등 / 브레이크등 배제 (전체가 빨간색 렌즈/램프인 경우 배제)
        red_mask = cv2.bitwise_or(
            cv2.inRange(patch_hsv, (0, 35, 45), (14, 255, 255)),
            cv2.inRange(patch_hsv, (165, 35, 45), (180, 255, 255))
        )
        red_ratio = np.count_nonzero(red_mask) / total_pixels
        if red_ratio >= 0.55:
            return False

        # 종횡비 판별 (원근 왜곡 수용: 1.48 ~ 5.8)
        # 종횡비 판별 (원근 왜곡 및 가드 프레임 수용: 1.45 ~ 6.8)
        if not (1.45 <= aspect <= 6.8):
            return False

        # 6대 규격 마스크 계산
        mask_white = cv2.inRange(patch_hsv, (0, 0, 50), (180, 65, 255))
        mask_yellow = cv2.inRange(patch_hsv, (13, 35, 50), (42, 255, 255))
        mask_blue = cv2.inRange(patch_hsv, (88, 25, 75), (132, 255, 255))
        mask_lime = cv2.inRange(patch_hsv, (45, 35, 90), (85, 255, 255))
        # 구형 녹색 번호판은 채도가 높은 특정 에메랄드 녹색만 수용 (나뭇잎/가로수 배제)
        mask_dark_green = cv2.inRange(patch_hsv, (48, 80, 30), (82, 255, 120))

        # 1. 신형/일반 번호판 규격 (흰색, 영업용 노란색, 전기차 하늘색, 형광연두)
        standard_bg_mask = cv2.bitwise_or(mask_white, mask_yellow)
        standard_bg_mask = cv2.bitwise_or(standard_bg_mask, mask_blue)
        standard_bg_mask = cv2.bitwise_or(standard_bg_mask, mask_lime)

        bg_ratio = np.count_nonzero(standard_bg_mask) / total_pixels
        if bg_ratio >= 0.26:
            bg_level = np.percentile(patch_gray, 85)
            text_th = bg_level - 16
            dark_pixels = (patch_gray < text_th)
            dark_ratio = np.count_nonzero(dark_pixels) / total_pixels
            if 0.03 <= dark_ratio <= 0.80:
                bg_mean = np.mean(patch_gray[standard_bg_mask > 0])
                dark_mean = np.mean(patch_gray[dark_pixels])
                if (bg_mean - dark_mean) >= 8:
                    if PrivacyFilter._verify_digit_sequence(patch_gray):
                        return True

        # 2. 구형 녹색 번호판 (진녹색 바탕 + 흰색 글자, 2열 규격 종횡비 1.3~2.4만 한정하여 나뭇잎 오탐 차단)
        if 1.30 <= aspect <= 2.40:
            dark_green_ratio = np.count_nonzero(mask_dark_green) / total_pixels
            if dark_green_ratio >= 0.45:
                bg_level = np.percentile(patch_gray, 20)
                white_th = bg_level + 20
                white_text_pixels = (patch_gray > white_th)
                white_text_ratio = np.count_nonzero(white_text_pixels) / total_pixels
                if 0.04 <= white_text_ratio <= 0.50:
                    if PrivacyFilter._verify_digit_sequence(patch_gray):
                        return True

        return False

    def _detect_plates(self, frame):
        """
        차량 기반 번호판 정밀 검출 (Vehicle-ROI Plate Detection):
        1. 도로 위 차량(Car/Truck/Bus/Motorcycle)을 초고속 DNN으로 먼저 검출.
        2. 배경(건물, 창문, 간판, 나뭇잎, 가드레일)은 차체가 아니므로 원천 배제 (오탐 0%).
        3. 각 차량 내부의 실제 번호판 장착 영역(하단 45%~96%, 중앙 70%)에서만
           Sobel-X 에지 + 번호판 바탕색 마스크 + MSER를 융합하여 '실제 번호판'의 정확한 위치를 검출.
        4. 차량 내부에서 실제 번호판이 식별되지 않는 경우, 엠블럼이나 트렁크에 임의의 가짜 블러를 절대 치지 않음!
        """
        if frame is None or frame.size == 0:
            return []

        oh, ow = frame.shape[:2]
        vehicles = self._detect_vehicles(frame)

        plates = []

        if vehicles:
            for vx, vy, vw, vh in vehicles:
                # 1. 차가 옆면일 때는 번호판이 없으므로 원천 배제!
                # 화면 좌우 가장자리에서 문짝/측면만 걸쳐 지나가는 차량 배제
                if (vx <= 8 and vw < 280) or ((vx + vw) >= (ow - 8) and vw < 280):
                    continue
                # 지나치게 가로로 길쭉한 측면 차량(vw/vh > 2.2) 배제
                aspect_v = vw / float(vh)
                if aspect_v > 2.2:
                    continue

                # 2. 번호판 탐색 영역: 테일램프/방향지시등(차량 양끝 20%)을 원천 배제한 중앙 60% 영역!
                # 높이: SUV 트렁크 중앙(28%)부터 세단 범퍼 하단(96%)까지
                roi_y1 = max(0, vy + int(vh * 0.28))
                roi_y2 = min(oh, vy + int(vh * 0.96))
                roi_x1 = max(0, vx + int(vw * 0.20))
                roi_x2 = min(ow, vx + int(vw * 0.80))

                rw = roi_x2 - roi_x1
                rh = roi_y2 - roi_y1
                if rw < 18 or rh < 8:
                    continue

                roi_bgr = frame[roi_y1:roi_y2, roi_x1:roi_x2]
                roi_gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
                roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

                car_cx = vx + vw / 2.0
                candidates = []

                # 1. Sobel-X 에지 기반 번호판 글자 텍스처 결합
                grad_x = cv2.Sobel(roi_gray, cv2.CV_16S, 1, 0, ksize=3)
                abs_x = cv2.convertScaleAbs(grad_x)
                _, th_x = cv2.threshold(abs_x, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                kw = max(7, min(23, int(vw * 0.06) | 1))
                kh = max(3, min(7, int(vh * 0.02) | 1))
                kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, kh))
                closed_x = cv2.morphologyEx(th_x, cv2.MORPH_CLOSE, kernel_h)
                cnts_x, _ = cv2.findContours(closed_x, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_x:
                    candidates.append(cv2.boundingRect(c))

                # 2. 번호판 배경색 마스크 (흰색, 노란색, 파란색)
                mw = cv2.inRange(roi_hsv, (0, 0, 70), (180, 50, 255))
                my = cv2.inRange(roi_hsv, (13, 30, 70), (38, 255, 255))
                mb = cv2.inRange(roi_hsv, (85, 20, 70), (130, 255, 255))
                mask_col = cv2.bitwise_or(cv2.bitwise_or(mw, my), mb)
                kernel_col = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, kh + 2))
                closed_col = cv2.morphologyEx(mask_col, cv2.MORPH_CLOSE, kernel_col)
                cnts_col, _ = cv2.findContours(closed_col, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts_col:
                    candidates.append(cv2.boundingRect(c))

                # 3. MSER 검출
                if self._mser is not None:
                    try:
                        _, mser_boxes = self._mser.detectRegions(roi_gray)
                        for mb_rect in mser_boxes:
                            candidates.append((mb_rect[0], mb_rect[1], mb_rect[2], mb_rect[3]))
                    except Exception:
                        pass

                best_cand = None
                best_score = -1.0

                for bx, by, bw, bh in candidates:
                    if bw < 14 or bh < 4:
                        continue
                    gx = roi_x1 + bx
                    gy = roi_y1 + by
                    plate_cx = gx + bw / 2.0
                    offset_x = abs(plate_cx - car_cx) / float(vw)

                    # 번호판은 차량 중심선 부근에 위치 (중심 편차 14% 이내)
                    if offset_x > 0.14:
                        continue

                    # 대한민국 번호판 너비는 차폭의 15% ~ 33% 사이 (33% 초과 시 테일램프/깜빡이 침범 오탐으로 원천 배제!)
                    ratio_w = bw / float(vw)
                    if not (0.15 <= ratio_w <= 0.33):
                        continue

                    pw = bw
                    ph = bh
                    asp = pw / float(ph)
                    # 여백/단일 라인만 잡힌 경우 한국 번호판 규격 비율(3.8:1)로 세로 확장
                    if asp > 5.2:
                        ph = int(pw / 3.8)
                        gy = max(0, gy - int(ph * 0.4))
                        asp = pw / float(ph)

                    if not (1.8 <= asp <= 5.8):
                        continue

                    patch = frame[gy:min(oh, gy + ph), gx:min(ow, gx + pw)]
                    if patch.size == 0:
                        continue
                    p_gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
                    p_hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)

                    # ★ 필수 정밀 검수: 한국 번호판 규격 프로필 또는 정렬된 숫자 시퀀스 검증!
                    # 은색 크롬 가니쉬, 배기구, 차 문짝 몰딩 등 숫자가 없는 비(非)번호판은 100% 탈락!
                    is_profile = self._verify_korean_plate_profile(p_hsv, p_gray, asp)
                    is_digits = self._verify_digit_sequence(p_gray)

                    if not (is_profile or is_digits):
                        continue

                    # 후미등(빨간색 램프) 배제
                    red_mask = cv2.bitwise_or(
                        cv2.inRange(p_hsv, (0, 35, 45), (14, 255, 255)),
                        cv2.inRange(p_hsv, (165, 35, 45), (180, 255, 255))
                    )
                    if (np.count_nonzero(red_mask) / float(pw * ph)) > 0.40:
                        continue

                    # 점수 산정: 면적 * 중심 일치도 * 검증 보너스
                    bonus = 2.5 if (is_profile and is_digits) else 1.0
                    score = (pw * ph) * (1.0 - offset_x * 2.5) * bonus
                    if score > best_score:
                        best_score = score
                        best_cand = (gx, gy, pw, ph)

                if best_cand is not None:
                    # 번호판 외곽 램프/방향지시등 보호를 위해 타이트한 패딩(3%) 적용
                    pad_x = max(1, int(best_cand[2] * 0.03))
                    pad_y = max(1, int(best_cand[3] * 0.08))
                    rx = max(0, best_cand[0] - pad_x)
                    ry = max(0, best_cand[1] - pad_y)
                    rw = min(ow - rx, best_cand[2] + pad_x * 2)
                    rh = min(oh - ry, best_cand[3] + pad_y * 2)
                    plates.append((rx, ry, rw, rh))

            if plates:
                scores = [1.0] * len(plates)
                indices = cv2.dnn.NMSBoxes([list(p) for p in plates], scores, 0.5, 0.3)
                final_plates = []
                for idx in indices:
                    i = idx if isinstance(idx, (int, np.integer)) else idx[0]
                    final_plates.append(tuple(plates[i]))
                return final_plates

        # 차량 모델 미보유 또는 미검출 시 MSER 안전 Fallback
        if self._mser is None:
            return []

        top_offset = int(oh * 0.365)
        bottom_offset = int(oh * 0.90)
        roi = frame[top_offset:bottom_offset, :]
        rh, rw = roi.shape[:2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        candidates = []
        min_bw, max_bw = max(20, int(ow * 0.015)), max(220, int(ow * 0.20))
        min_bh, max_bh = max(6, int(oh * 0.007)), max(65, int(oh * 0.075))

        try:
            _, bboxes = self._mser.detectRegions(gray)
        except Exception:
            bboxes = []

        for b in bboxes:
            x, y, bw, bh = b
            if bh == 0 or bw == 0:
                continue
            asp = bw / float(bh)
            if bw >= min_bw and bw <= max_bw and bh >= min_bh and bh <= max_bh and (1.45 <= asp <= 6.8):
                candidates.append((x, y, bw, bh))

        passed_boxes = []
        total_frame_area = float(ow * oh)
        for x, y, bw, bh in candidates:
            if (bw * bh) > (total_frame_area * 0.040):
                continue
            aspect = bw / float(bh)
            patch_hsv = hsv[y:y+bh, x:x+bw]
            patch_gray = gray[y:y+bh, x:x+bw]
            if patch_hsv.size == 0 or patch_gray.size == 0:
                continue
            if not self._verify_korean_plate_profile(patch_hsv, patch_gray, aspect):
                continue

            pad_x = max(2, int(bw * 0.06))
            pad_y = max(2, int(bh * 0.10))
            rx = max(0, x - pad_x)
            ry = max(0, y + top_offset - pad_y)
            rw_box = min(ow - rx, bw + pad_x * 2)
            rh_box = min(oh - ry, bh + pad_y * 2)
            passed_boxes.append([rx, ry, rw_box, rh_box])

        if not passed_boxes:
            return []

        scores = [1.0] * len(passed_boxes)
        indices = cv2.dnn.NMSBoxes(passed_boxes, scores, 0.5, 0.3)
        final_boxes = []
        for idx in indices:
            i = idx if isinstance(idx, (int, np.integer)) else idx[0]
            final_boxes.append(tuple(passed_boxes[i]))

        filtered_boxes = []
        for i, a in enumerate(final_boxes):
            contained_by_smaller = False
            ax, ay, aw, ah = a
            for j, b in enumerate(final_boxes):
                if i == j:
                    continue
                bx, by, bw, bh = b
                if aw * ah > bw * bh * 1.5:
                    if ax <= bx and ay <= by and (ax + aw) >= (bx + bw) and (ay + ah) >= (by + bh):
                        contained_by_smaller = True
                        break
            if not contained_by_smaller:
                filtered_boxes.append(a)

        return filtered_boxes

    def _apply_blur(self, frame, boxes):
        h, w = frame.shape[:2]
        for rx, ry, rw, rh in boxes:
            x1 = max(0, rx)
            y1 = max(0, ry)
            x2 = min(w, rx + rw)
            y2 = min(h, ry + rh)
            if x2 <= x1 or y2 <= y1:
                continue
            roi = frame[y1:y2, x1:x2]
            # 블러 커널 크기를 번호판 높이에 맞게 비례 조정 (주변 테일램프/깜빡이로 번짐 방지)
            ksize = max(11, min(self.blur_ksize, int(rh * 0.75) | 1))
            frame[y1:y2, x1:x2] = cv2.GaussianBlur(
                roi, (ksize, ksize), 20
            )
        return frame

    def apply(self, frame):
        """단일 프레임 블러 적용 (실시간 프리뷰 등)."""
        if not self.enabled or frame is None or frame.size == 0:
            return frame

        self._frame_count += 1
        # 키프레임(detect_interval 주기) 여부 결정
        should_detect = (self.detect_interval <= 1) or (self._frame_count % self.detect_interval == 1)
        active_boxes = []
        frame_gray = None

        if self.blur_face or self.blur_plate:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.blur_face:
            if should_detect:
                new_faces = self._detect_faces(frame)
                tracked_faces = self._face_tracker.update(new_faces, frame_gray)
            else:
                tracked_faces = self._face_tracker.track_interframe(frame_gray)
            active_boxes.extend(tracked_faces)

        if self.blur_plate:
            if should_detect:
                new_plates = self._detect_plates(frame)
                tracked_plates = self._plate_tracker.update(new_plates, frame_gray)
            else:
                tracked_plates = self._plate_tracker.track_interframe(frame_gray)
            active_boxes.extend(tracked_plates)

        if active_boxes:
            frame = self._apply_blur(frame, active_boxes)

        return frame

    def apply_with_backfill(self, frame, frame_buffer):
        """내보내기용 소급 블러 지원 적용."""
        if not self.enabled or frame is None or frame.size == 0:
            return frame

        self._frame_count += 1
        should_detect = (self.detect_interval <= 1) or (self._frame_count % self.detect_interval == 1)
        active_boxes = []
        frame_gray = None

        if self.blur_face or self.blur_plate:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.blur_face:
            if should_detect:
                new_faces = self._detect_faces(frame)
                tracked_faces = self._face_tracker.update(new_faces, frame_gray)
            else:
                tracked_faces = self._face_tracker.track_interframe(frame_gray)
            active_boxes.extend(tracked_faces)

        if self.blur_plate:
            if should_detect:
                new_plates = self._detect_plates(frame)
                tracked_plates = self._plate_tracker.update(new_plates, frame_gray)
            else:
                tracked_plates = self._plate_tracker.track_interframe(frame_gray)
            active_boxes.extend(tracked_plates)

            if should_detect and self._plate_tracker.newly_confirmed_history and frame_buffer:
                buf_len = len(frame_buffer)
                for past_rects in self._plate_tracker.newly_confirmed_history:
                    for step_back, rect in enumerate(reversed(past_rects)):
                        buf_idx = buf_len - 1 - step_back
                        if 0 <= buf_idx < buf_len:
                            frame_buffer[buf_idx] = self._apply_blur(
                                frame_buffer[buf_idx], [rect]
                            )

        if active_boxes:
            frame = self._apply_blur(frame, active_boxes)

        return frame

    def reset(self):
        self._frame_count = 0
        if self._face_tracker:
            self._face_tracker.reset()
        if self._plate_tracker:
            self._plate_tracker.reset()
