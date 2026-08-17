import sys
import os
import re
import math
import struct
import cv2
import numpy as np
import urllib.request
from datetime import datetime, timedelta
from PIL import Image, ImageDraw, ImageFont

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QCheckBox, QSlider, QFileDialog, QGroupBox,
    QProgressBar, QMessageBox, QComboBox, QTreeWidget, QTreeWidgetItem,
    QSplitter, QStyleFactory, QStyle, QStyleOptionSlider
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QRect
from PyQt6.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QBrush, QIcon

cv2.ocl.setUseOpenCL(True)


# 해상도별 현실적인 MP4V 인코딩 비트레이트 반영
RESOLUTIONS = {
    "QHD (2560x1440) - 최고화질": {"size": (2560, 1440), "bitrate_mbps": 50.0},
    "FHD (1920x1080) - 권장 표준": {"size": (1920, 1080), "bitrate_mbps": 35.0},
    "HD (1280x720) - 용량 절감": {"size": (1280, 720), "bitrate_mbps": 18.0},
    "Compact (960x540) - 모바일용": {"size": (960, 540), "bitrate_mbps": 9.0},
}

SENSITIVITY_LEVELS = {
    "낮음": {"pixel_diff": 35, "min_area": 800},
    "보통": {"pixel_diff": 25, "min_area": 400},
    "높음": {"pixel_diff": 15, "min_area": 150},
    "매우높음": {"pixel_diff": 10, "min_area": 50}
}

OSM_TILE_CACHE = {}


class HighlightSlider(QSlider):
    def __init__(self, orientation=Qt.Orientation.Horizontal, parent=None):
        super().__init__(orientation, parent)
        self.event_blocks = []
        self.in_frame = None
        self.out_frame = None

    def set_event_blocks(self, blocks):
        self.event_blocks = blocks if blocks else []
        self.update()

    def set_range_points(self, in_f, out_f):
        self.in_frame = in_f
        self.out_frame = out_f
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            opt = QStyleOptionSlider()
            self.initStyleOption(opt)
            sr = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, opt, QStyle.SubControl.SC_SliderGroove, self)
            hr = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, opt, QStyle.SubControl.SC_SliderHandle, self)
            
            click_x = event.pos().x()
            half_h = hr.width() // 2
            groove_x = sr.x() + half_h
            groove_w = sr.width() - hr.width()
            
            if groove_w > 0:
                val_pct = (click_x - groove_x) / float(groove_w)
                val_pct = max(0.0, min(1.0, val_pct))
                new_val = int(self.minimum() + val_pct * (self.maximum() - self.minimum()))
                self.setValue(new_val)
                event.accept()
                return
        super().mousePressEvent(event)

    def paintEvent(self, event):
        super().paintEvent(event)

        max_val = self.maximum()
        if max_val <= 0:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        track_y = h // 2

        if self.in_frame is not None or self.out_frame is not None:
            in_pos = int((self.in_frame / max_val) * w) if self.in_frame is not None else 0
            out_pos = int((self.out_frame / max_val) * w) if self.out_frame is not None else w

            rect_x = min(in_pos, out_pos)
            rect_w = abs(out_pos - in_pos)
            
            if rect_w > 0:
                painter.fillRect(QRect(rect_x, track_y - 6, max(2, rect_w), 12), QColor(0, 230, 255, 90))

        if self.event_blocks:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(255, 60, 60, 180))

            for start_f, end_f in self.event_blocks:
                x1 = int((start_f / max_val) * w)
                x2 = int((end_f / max_val) * w)
                block_w = max(4, x2 - x1)
                painter.drawRoundedRect(QRect(x1, track_y - 3, block_w, 6), 2, 2)

        curr_val = self.value()
        x_head = int((curr_val / max_val) * w)
        
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(0, 0, 0, 220))
        painter.drawRoundedRect(QRect(x_head - 4, track_y - 12, 8, 24), 3, 3)

        painter.setBrush(QColor(255, 255, 255, 255))
        painter.drawRoundedRect(QRect(x_head - 3, track_y - 11, 6, 22), 2, 2)
        
        painter.setBrush(QColor(0, 230, 255, 255))
        painter.drawRoundedRect(QRect(x_head - 1, track_y - 9, 2, 18), 1, 1)


class OpenStreetMapTileLoader:
    @staticmethod
    def latlon_to_pixel(lat, lon, zoom):
        lat_rad = math.radians(lat)
        n = 2.0 ** zoom
        x = (lon + 180.0) / 360.0 * n * 256.0
        y = (1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n * 256.0
        return x, y

    @classmethod
    def fetch_tile(cls, x, y, zoom):
        key = (zoom, x, y)
        if key in OSM_TILE_CACHE:
            return OSM_TILE_CACHE[key]

        url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
        req = urllib.request.Request(url, headers={'User-Agent': 'DashcamStudioPro/1.0'})
        try:
            with urllib.request.urlopen(req, timeout=0.2) as resp:
                arr = np.frombuffer(resp.read(), dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    OSM_TILE_CACHE[key] = img
                    return img
        except Exception:
            pass
        return None

    @classmethod
    def generate_minimap(cls, lat, lon, zoom=16, map_w=360, map_h=180):
        if lat == 0.0 and lon == 0.0:
            return cls._draw_offline_radar(map_w, map_h, "NO GPS")

        px, py = cls.latlon_to_pixel(lat, lon, zoom)
        tile_x = int(px // 256)
        tile_y = int(py // 256)

        stitched = np.full((256 * 3, 256 * 3, 3), (30, 30, 30), dtype=np.uint8)
        has_any_tile = False

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                tx, ty = tile_x + dx, tile_y + dy
                tile_img = cls.fetch_tile(tx, ty, zoom)
                
                pos_x = (dx + 1) * 256
                pos_y = (dy + 1) * 256

                if tile_img is not None:
                    stitched[pos_y:pos_y+256, pos_x:pos_x+256] = tile_img
                    has_any_tile = True
                else:
                    cv2.rectangle(stitched, (pos_x, pos_y), (pos_x+256, pos_y+256), (45, 45, 45), 1)

        if not has_any_tile:
            return cls._draw_offline_radar(map_w, map_h, f"{lat:.3f},{lon:.3f}")

        offset_x = 256 + int(px % 256)
        offset_y = 256 + int(py % 256)
        half_w = map_w // 2
        half_h = map_h // 2

        crop_y1 = max(0, offset_y - half_h)
        crop_y2 = min(stitched.shape[0], offset_y + half_h)
        crop_x1 = max(0, offset_x - half_w)
        crop_x2 = min(stitched.shape[1], offset_x + half_w)

        crop = stitched[crop_y1:crop_y2, crop_x1:crop_x2].copy()
        if crop.shape[0] != map_h or crop.shape[1] != map_w:
            crop = cv2.resize(crop, (map_w, map_h), interpolation=cv2.INTER_NEAREST)

        cv2.circle(crop, (half_w, half_h), 7, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(crop, (half_w, half_h), 10, (255, 255, 255), 2, cv2.LINE_AA)

        arrow_x = map_w - 24
        cv2.arrowedLine(crop, (arrow_x, 34), (arrow_x, 10), (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.4)
        cv2.putText(crop, "N", (arrow_x - 18, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.rectangle(crop, (0, 0), (map_w - 1, map_h - 1), (60, 60, 60), 2)
        return crop

    @staticmethod
    def _draw_offline_radar(map_w, map_h, text):
        img = np.full((map_h, map_w, 3), (20, 24, 28), dtype=np.uint8)
        cx, cy = map_w // 2, map_h // 2
        r = min(map_w, map_h)
        cv2.circle(img, (cx, cy), int(r * 0.4), (50, 60, 70), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), int(r * 0.2), (50, 60, 70), 1, cv2.LINE_AA)
        cv2.line(img, (cx, 10), (cx, map_h - 10), (40, 50, 60), 1)
        cv2.line(img, (10, cy), (map_w - 10, cy), (40, 50, 60), 1)
        
        cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 9, (255, 255, 255), 2, cv2.LINE_AA)

        arrow_x = map_w - 24
        cv2.arrowedLine(img, (arrow_x, 34), (arrow_x, 10), (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.4)
        cv2.putText(img, "N", (arrow_x - 18, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(img, text, (12, map_h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 185), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0, 0), (map_w - 1, map_h - 1), (60, 60, 60), 1)
        return img


class RealTeslaSEIDecoder:
    def __init__(self, mp4_path):
        self.mp4_path = mp4_path
        self.telemetry_cache = {}
        self.has_sei_data = False
        self.max_speed_kmh = 0
        self.parse_mp4_sei()

    @classmethod
    def quick_check_is_parking(cls, mp4_path):
        if not os.path.exists(mp4_path):
            return False
        try:
            inst = cls.__new__(cls)
            with open(mp4_path, 'rb') as fp:
                offset, size = inst.find_mdat(fp)
                if offset is None:
                    return False

                file_size = os.path.getsize(mp4_path)
                mdat_len = size if size > 0 else (file_size - offset)
                if mdat_len <= 0:
                    return True

                chunk_size = 65536
                sample_ratios = [0.0, 0.25, 0.50, 0.75, 0.90]
                sample_offsets = [offset + int(mdat_len * r) for r in sample_ratios]

                max_speed = 0
                has_gps = False
                found_sei = False

                for s_off in sample_offsets:
                    fp.seek(s_off)
                    buf = fp.read(chunk_size)
                    if not buf:
                        continue

                    pos = 0
                    buf_len = len(buf)
                    while pos < buf_len - 4:
                        nal_size = struct.unpack(">I", buf[pos:pos+4])[0]
                        pos += 4
                        if nal_size < 2 or pos + nal_size > buf_len:
                            break

                        if (buf[pos] & 0x1F) == 6:
                            peek = buf[pos:pos+min(nal_size, 256)]
                            payload = inst.extract_proto_payload(peek)
                            if payload:
                                telemetry = inst.parse_protobuf_wire(payload)
                                if telemetry:
                                    found_sei = True
                                    spd = telemetry.get("speed_kmh", 0)
                                    ap_mode = telemetry.get("ap_mode", "MANUAL")
                                    if spd > max_speed:
                                        max_speed = spd
                                    lat = telemetry.get("lat", 0.0)
                                    lon = telemetry.get("lon", 0.0)
                                    if lat != 0.0 or lon != 0.0:
                                        has_gps = True

                                    if spd > 5 or ap_mode != "MANUAL":
                                        return False
                        pos += nal_size

                if not found_sei or not has_gps:
                    return True
                return max_speed <= 3
        except Exception:
            pass
        return False

    def find_mdat(self, fp):
        fp.seek(0)
        while True:
            header = fp.read(8)
            if len(header) < 8:
                return None, None
            size32, atom_type = struct.unpack(">I4s", header)
            if size32 == 1:
                large = fp.read(8)
                if len(large) < 8:
                    return None, None
                size = struct.unpack(">Q", large)[0] - 16
            else:
                size = size32 - 8
            if atom_type == b'mdat':
                return fp.tell(), size
            else:
                if size > 0:
                    fp.seek(size, 1)

    def strip_emulation_prevention_bytes(self, data: bytes) -> bytes:
        stripped = bytearray()
        zero_count = 0
        for byte in data:
            if zero_count >= 2 and byte == 0x03:
                zero_count = 0
                continue
            stripped.append(byte)
            zero_count = 0 if byte != 0 else zero_count + 1
        return bytes(stripped)

    def extract_proto_payload(self, nal: bytes):
        if not isinstance(nal, bytes) or len(nal) < 2:
            return None
        for i in range(3, len(nal) - 1):
            byte = nal[i]
            if byte == 0x42:
                continue
            if byte == 0x69:
                if i > 2:
                    return self.strip_emulation_prevention_bytes(nal[i + 1:-1])
                break
            break
        return None

    def iter_nals(self, fp, offset: int, size: int):
        NAL_ID_SEI = 6
        fp.seek(offset)
        consumed = 0
        while size == 0 or consumed < size:
            header = fp.read(4)
            if len(header) < 4:
                break
            nal_size = struct.unpack(">I", header)[0]
            consumed += 4
            if nal_size < 2:
                fp.seek(nal_size, 1)
                consumed += nal_size
                continue
            
            nal = fp.read(nal_size)
            consumed += nal_size
            if len(nal) < nal_size:
                break
                
            nal_type = nal[0] & 0x1F
            if nal_type == NAL_ID_SEI:
                yield nal

    def parse_protobuf_wire(self, payload: bytes):
        pos = 0
        length = len(payload)
        
        speed_mps = 0.0
        accel_pct = 0.0
        steering_deg = 0.0
        brake_applied = False
        autopilot_state = 0
        left_blinker = False
        right_blinker = False
        lat_deg = 0.0
        lon_deg = 0.0
        
        while pos < length:
            try:
                shift = 0
                tag = 0
                while True:
                    if pos >= length: break
                    b = payload[pos]
                    pos += 1
                    tag |= (b & 0x7f) << shift
                    if not (b & 0x80):
                        break
                    shift += 7
                
                field_number = tag >> 3
                wire_type = tag & 0x07
                
                if wire_type == 0:
                    shift = 0
                    val = 0
                    while True:
                        if pos >= length: break
                        b = payload[pos]
                        pos += 1
                        val |= (b & 0x7f) << shift
                        if not (b & 0x80):
                            break
                        shift += 7
                    if field_number == 7:
                        left_blinker = bool(val)
                    elif field_number == 8:
                        right_blinker = bool(val)
                    elif field_number == 9: 
                        brake_applied = bool(val)
                    elif field_number == 10: 
                        autopilot_state = val

                elif wire_type == 5:
                    if pos + 4 > length: break
                    val_float = struct.unpack('<f', payload[pos:pos+4])[0]
                    pos += 4
                    if field_number == 4:
                        speed_mps = val_float
                    elif field_number == 5:
                        accel_pct = val_float
                    elif field_number == 6:
                        steering_deg = val_float

                elif wire_type == 1:
                    if pos + 8 > length: break
                    val_double = struct.unpack('<d', payload[pos:pos+8])[0]
                    pos += 8
                    if field_number == 11 and -90.0 <= val_double <= 90.0:
                        lat_deg = val_double
                    elif field_number == 12 and -180.0 <= val_double <= 180.0:
                        lon_deg = val_double

                elif wire_type == 2:
                    shift = 0
                    sub_len = 0
                    while True:
                        if pos >= length: break
                        b = payload[pos]
                        pos += 1
                        sub_len |= (b & 0x7f) << shift
                        if not (b & 0x80):
                            break
                        shift += 7
                    pos += sub_len

                else:
                    pos += 1
            except Exception:
                pos += 1

        if math.isnan(speed_mps):
            speed_mps = 0.0

        speed_kmh = int(round(speed_mps * 3.6))
        
        ap_modes = {
            0: "MANUAL",
            1: "FSD ACTIVE",
            2: "AUTOPILOT",
            3: "TACC"
        }
        ap_mode_str = ap_modes.get(autopilot_state, "MANUAL")

        return {
            "speed_kmh": max(0, speed_kmh),
            "steering_deg": int(round(steering_deg)),
            "accel_pct": min(100, max(0, int(round(accel_pct)))),
            "brake_pct": 100 if brake_applied else 0,
            "ap_mode": ap_mode_str,
            "left_blinker": left_blinker,
            "right_blinker": right_blinker,
            "lat": lat_deg,
            "lon": lon_deg
        }

    def parse_mp4_sei(self):
        if not os.path.exists(self.mp4_path):
            return

        try:
            with open(self.mp4_path, 'rb') as fp:
                offset, size = self.find_mdat(fp)
                if offset is None:
                    return

                frame_no = 0
                last_valid = None

                for nal in self.iter_nals(fp, offset, size):
                    payload = self.extract_proto_payload(nal)
                    if not payload:
                        continue

                    parsed = self.parse_protobuf_wire(payload)
                    if parsed:
                        last_valid = parsed
                        self.telemetry_cache[frame_no] = parsed
                        self.has_sei_data = True
                        if parsed["speed_kmh"] > self.max_speed_kmh:
                            self.max_speed_kmh = parsed["speed_kmh"]
                    elif last_valid:
                        self.telemetry_cache[frame_no] = dict(last_valid)

                    frame_no += 1

        except Exception as e:
            print(f"[!] SEI Parse Notice: {e}")

    def get_frame_telemetry(self, frame_idx, fps=30.0):
        if self.has_sei_data and frame_idx in self.telemetry_cache:
            return self.telemetry_cache[frame_idx]

        if self.has_sei_data and self.telemetry_cache:
            closest_idx = min(self.telemetry_cache.keys(), key=lambda k: abs(k - frame_idx))
            return self.telemetry_cache[closest_idx]

        return {
            "speed_kmh": 0,
            "steering_deg": 0,
            "accel_pct": 0,
            "brake_pct": 0,
            "ap_mode": "MANUAL",
            "left_blinker": False,
            "right_blinker": False,
            "lat": 0.0,
            "lon": 0.0
        }


class LightMotionScanner(QThread):
    events_found = pyqtSignal(list)

    def __init__(self, clip_list, fps=36.0, sensitivity_preset="보통", target_scan_fps=6.0):
        super().__init__()
        self.clip_list = clip_list
        self.fps = fps
        self.target_scan_fps = target_scan_fps
        self.preset = SENSITIVITY_LEVELS.get(sensitivity_preset, SENSITIVITY_LEVELS["보통"])
        self.is_stopped = False

    def stop(self):
        self.is_stopped = True

    def run(self):
        if not self.clip_list:
            return

        raw_events = set()
        pixel_diff_thresh = self.preset["pixel_diff"]
        min_area = self.preset["min_area"]

        global_frame_offset = 0
        step = max(1, int(round(self.fps / self.target_scan_fps)))

        delay_frames = max(3, int(self.target_scan_fps * 0.5))

        for clip_data in self.clip_list:
            if self.is_stopped: break
            
            valid_cams = {k: v for k, v in clip_data["cams"].items() 
                          if k in ['front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar'] and os.path.exists(v)}
            if not valid_cams: continue

            caps = {k: cv2.VideoCapture(v) for k, v in valid_cams.items()}
            ring_buffers = {k: [] for k in valid_cams.keys()}

            frame_idx = 0
            while not self.is_stopped:
                active_any = False
                for k, cap in caps.items():
                    if self.is_stopped: break
                    
                    if frame_idx % step != 0:
                        ret = cap.grab()
                        if ret: active_any = True
                        continue

                    ret, frame = cap.read()
                    if not ret or frame is None:
                        continue
                    active_any = True

                    small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_NEAREST)
                    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

                    ring_buffers[k].append(gray)
                    if len(ring_buffers[k]) > delay_frames:
                        old_gray = ring_buffers[k].pop(0)
                        
                        diff = cv2.absdiff(gray, old_gray)
                        _, thresh = cv2.threshold(diff, pixel_diff_thresh, 255, cv2.THRESH_BINARY)
                        
                        changed_pixels = cv2.countNonZero(thresh)
                        if changed_pixels > (min_area // 4):
                            raw_events.add(global_frame_offset + frame_idx)

                if not active_any:
                    break
                frame_idx += 1

            for cap in caps.values(): cap.release()
            global_frame_offset += frame_idx

        if not self.is_stopped:
            sorted_raw = sorted(list(raw_events))
            
            event_blocks = []
            merge_gap = int(self.fps * 4.0)

            lead_in = int(self.fps * 1.5)
            lead_out = int(self.fps * 2.0)

            for ef in sorted_raw:
                start_f = max(0, ef - lead_in)
                end_f = ef + lead_out

                if not event_blocks:
                    event_blocks.append([start_f, end_f])
                else:
                    last_start, last_end = event_blocks[-1]
                    if start_f <= last_end + merge_gap:
                        event_blocks[-1][1] = max(last_end, end_f)
                    else:
                        event_blocks.append([start_f, end_f])

            formatted_blocks = [(b[0], b[1]) for b in event_blocks]
            self.events_found.emit(formatted_blocks)


class OverlayRenderer:
    @staticmethod
    def create_tesla_steering_wheel(size=90):
        img = np.zeros((size, size, 4), dtype=np.uint8)
        center = (size // 2, size // 2)
        radius = int(size * 0.45)
        
        cv2.circle(img, center, radius, (180, 180, 185, 255), max(2, int(size * 0.06)), cv2.LINE_AA)
        
        hub_r = int(size * 0.16)
        cv2.circle(img, center, hub_r, (100, 100, 105, 255), -1, cv2.LINE_AA)
        
        thick = max(2, int(size * 0.08))
        cv2.line(img, (center[0] - radius, center[1]), (center[0] - hub_r, center[1]), (120, 120, 125, 255), thick, cv2.LINE_AA)
        cv2.line(img, (center[0] + hub_r, center[1]), (center[0] + radius, center[1]), (120, 120, 125, 255), thick, cv2.LINE_AA)
        cv2.line(img, (center[0], center[1] + hub_r), (center[0], center[1] + radius), (120, 120, 125, 255), thick, cv2.LINE_AA)
        
        cv2.circle(img, center, int(hub_r * 0.5), (150, 150, 155, 255), max(1, int(size * 0.02)), cv2.LINE_AA)

        return img

    @classmethod
    def draw_rotated_steering(cls, canvas, center, angle_deg, scale=1.0):
        wheel_size = int(90 * scale)
        wheel_img = cls.create_tesla_steering_wheel(wheel_size)
        
        h, w = wheel_img.shape[:2]
        M = cv2.getRotationMatrix2D((w / 2, h / 2), -angle_deg, 1.0)
        rotated = cv2.warpAffine(wheel_img, M, (w, h), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0,0))
        
        x_top = int(center[0] - w / 2)
        y_top = int(center[1] - h / 2)
        
        ch_h, ch_w = canvas.shape[:2]
        x1, x2 = max(0, x_top), min(ch_w, x_top + w)
        y1, y2 = max(0, y_top), min(ch_h, y_top + h)
        
        rx1, rx2 = x1 - x_top, x2 - x_top
        ry1, ry2 = y1 - y_top, y2 - y_top
        
        if x1 < x2 and y1 < y2:
            alpha = rotated[ry1:ry2, rx1:rx2, 3] / 255.0
            for c in range(3):
                canvas[y1:y2, x1:x2, c] = (
                    alpha * rotated[ry1:ry2, rx1:rx2, c] +
                    (1.0 - alpha) * canvas[y1:y2, x1:x2, c]
                )

    @staticmethod
    def draw_turn_signals(canvas, center_x, center_y, left_on, right_on, frame_idx, fps=30.0, scale=1.0):
        time_sec = frame_idx / fps
        is_blink_phase = (int(time_sec * 3.5) % 2) == 0

        l_active = left_on and is_blink_phase
        r_active = right_on and is_blink_phase

        l_color = (0, 255, 100) if l_active else (45, 50, 55)
        r_color = (0, 255, 100) if r_active else (45, 50, 55)
        
        thick = max(1, int(2 * scale))

        l_tip_x = center_x - int(42 * scale)
        l_head_x = center_x - int(16 * scale)
        l_stem_end_x = center_x - int(6 * scale)

        head_y = int(18 * scale)
        stem_y = int(8 * scale)

        pts_left = np.array([
            [l_tip_x, center_y],
            [l_head_x, center_y - head_y],
            [l_head_x, center_y - stem_y],
            [l_stem_end_x, center_y - stem_y],
            [l_stem_end_x, center_y + stem_y],
            [l_head_x, center_y + stem_y],
            [l_head_x, center_y + head_y]
        ], np.int32)

        r_tip_x = center_x + int(42 * scale)
        r_head_x = center_x + int(16 * scale)
        r_stem_end_x = center_x + int(6 * scale)

        pts_right = np.array([
            [r_tip_x, center_y],
            [r_head_x, center_y - head_y],
            [r_head_x, center_y - stem_y],
            [r_stem_end_x, center_y - stem_y],
            [r_stem_end_x, center_y + stem_y],
            [r_head_x, center_y + stem_y],
            [r_head_x, center_y + head_y]
        ], np.int32)

        cv2.fillPoly(canvas, [pts_left], l_color)
        cv2.polylines(canvas, [pts_left], True, (255, 255, 255) if l_active else (70, 75, 80), thick)

        cv2.fillPoly(canvas, [pts_right], r_color)
        cv2.polylines(canvas, [pts_right], True, (255, 255, 255) if r_active else (70, 75, 80), thick)

    @staticmethod
    def draw_pedal(frame, top_left, size, val_percent, label, color, scale=1.0):
        x, y = top_left
        w, h = size
        thick = max(1, int(1 * scale))
        cv2.rectangle(frame, (x, y), (x + w, y + h), (70, 70, 75), thick)
        fill_h = int((h - 2) * (val_percent / 100.0))
        if fill_h > 0:
            cv2.rectangle(frame, (x + 1, y + h - 1 - fill_h), (x + w - 1, y + h - 1), color, -1)
        
        f1 = 0.45 * scale
        f2 = 0.45 * scale
        cv2.putText(frame, label, (x + int(2 * scale), y - int(8 * scale)), cv2.FONT_HERSHEY_SIMPLEX, f1, (200, 200, 200), thick)
        cv2.putText(frame, f"{val_percent}%", (x - int(4 * scale), y + h + int(18 * scale)), cv2.FONT_HERSHEY_SIMPLEX, f2, (160, 160, 160), thick)

    @staticmethod
    def draw_sub_slot_cover(canvas, x, y, w, h, frame, label, scale=1.0):
        if frame is not None and frame.size > 0:
            fh, fw = frame.shape[:2]
            slot_aspect = w / h
            frame_aspect = fw / fh

            if frame_aspect > slot_aspect:
                crop_w = int(fh * slot_aspect)
                start_x = (fw - crop_w) // 2
                cropped = frame[:, start_x:start_x+crop_w]
            else:
                crop_h = int(fw / slot_aspect)
                start_y = (fh - crop_h) // 2
                cropped = frame[start_y:start_y+crop_h, :]

            resized = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_NEAREST)
            canvas[y:y+h, x:x+w] = resized
        else:
            cv2.rectangle(canvas, (x, y), (x+w, y+h), (20, 22, 26), -1)
            cv2.putText(canvas, "NO CAMERA", (x + int(w * 0.28), y + int(h * 0.58)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, (60, 65, 75), 1)
            
        tag_w = int(140 * scale)
        tag_h = int(24 * scale)
        cv2.rectangle(canvas, (x + 4, y + 4), (x + 4 + tag_w, y + 4 + tag_h), (0, 0, 0), -1)
        cv2.rectangle(canvas, (x + 4, y + 4), (x + 4 + tag_w, y + 4 + tag_h), (70, 75, 80), 1)
        cv2.putText(canvas, label, (x + 8, y + int(20 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, (0, 230, 255), 1)
        cv2.rectangle(canvas, (x, y), (x+w, y+h), (45, 50, 55), 1)

    @staticmethod
    def apply_rendering_overlay(grid_img, progress_pct):
        h, w = grid_img.shape[:2]
        
        white_layer = np.full_like(grid_img, 255)
        blended = cv2.addWeighted(white_layer, 0.55, grid_img, 0.45, 0)
        
        box_w, box_h = int(680 * (w / 1920.0)), int(180 * (h / 1080.0))
        cx, cy = w // 2, h // 2
        x1, y1 = cx - box_w // 2, cy - box_h // 2
        x2, y2 = cx + box_w // 2, cy + box_h // 2
        
        cv2.rectangle(blended, (x1, y1), (x2, y2), (18, 22, 28), -1)
        cv2.rectangle(blended, (x1, y1), (x2, y2), (0, 200, 255), max(2, int(3 * (w / 1920.0))))
        
        text = f"영상 저장 렌더링 중... {progress_pct}%"
        font_size = int(36 * (w / 1920.0))
        
        try:
            img_pil = Image.fromarray(cv2.cvtColor(blended, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(img_pil)
            
            font_paths = [
                "c:/windows/fonts/malgun.ttf",
                "c:/windows/fonts/gulim.ttc",
                "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
                "/System/Library/Fonts/AppleSDGothicNeo.ttc"
            ]
            font = None
            for fp in font_paths:
                try:
                    font = ImageFont.truetype(fp, font_size)
                    break
                except Exception:
                    pass
                    
            if font is not None:
                bbox = draw.textbbox((0, 0), text, font=font)
                tw = bbox[2] - bbox[0]
                th = bbox[3] - bbox[1]
                draw.text((cx - tw // 2, cy - th // 2), text, font=font, fill=(0, 230, 255))
                return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        except Exception:
            pass

        fallback_text = f"RENDERING... {progress_pct}%"
        cv2.putText(blended, fallback_text, (cx - int(210 * (w/1920.0)), cy + int(15 * (h/1080.0))), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.25 * (w/1920.0), (0, 230, 255), 3)
        return blended

    @classmethod
    def render(cls, frames, frame_idx, base_time, options, decoder, fps=36.0, target_size=(1920, 1080)):
        out_w, out_h = target_size
        scale = out_w / 1920.0

        bottom_h = int(180 * scale)
        front_h = out_h - bottom_h
        sub_w = int(480 * scale)
        front_w = out_w - sub_w

        canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)

        f_front = frames.get('front')
        if f_front is not None and f_front.size > 0:
            fh, fw = f_front.shape[:2]
            f_aspect = fw / fh
            s_aspect = front_w / front_h
            if f_aspect > s_aspect:
                crop_w = int(fh * s_aspect)
                sx = (fw - crop_w) // 2
                cropped = f_front[:, sx:sx+crop_w]
            else:
                crop_h = int(fw / s_aspect)
                sy = (fh - crop_h) // 2
                cropped = f_front[sy:sy+crop_h, :]

            canvas[0:front_h, 0:front_w] = cv2.resize(cropped, (front_w, front_h), interpolation=cv2.INTER_LINEAR)
        
        tag_w = int(140 * scale)
        tag_h = int(24 * scale)
        cv2.rectangle(canvas, (8, 8), (8 + tag_w, 8 + tag_h), (0, 0, 0), -1)
        cv2.rectangle(canvas, (8, 8), (8 + tag_w, 8 + tag_h), (80, 80, 80), 1)
        cv2.putText(canvas, "FRONT MAIN", (14, int(24 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, (0, 230, 255), 1)

        export_speed = float(options.get("export_speed", 1.0))
        if abs(export_speed - 1.0) > 0.01:
            speed_text = f"{export_speed:.1f}x SPEED"
            sp_tag_w = int(130 * scale)
            sp_tag_h = int(28 * scale)
            sp_x = front_w - sp_tag_w - int(8 * scale)
            sp_y = int(8 * scale)

            cv2.rectangle(canvas, (sp_x, sp_y), (sp_x + sp_tag_w, sp_y + sp_tag_h), (10, 12, 16), -1)
            cv2.rectangle(canvas, (sp_x, sp_y), (sp_x + sp_tag_w, sp_y + sp_tag_h), (0, 230, 255), max(1, int(1.5 * scale)))
            cv2.putText(canvas, speed_text, (sp_x + int(10 * scale), sp_y + int(20 * scale)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52 * scale, (0, 230, 255), max(1, int(2 * scale)))

        sub_configs = [
            ("REAR", frames.get('back')),
            ("LEFT REPEATER", frames.get('left_repeater')),
            ("RIGHT REPEATER", frames.get('right_repeater')),
        ]

        num_slots = len(sub_configs)
        sub_h_single = front_h // num_slots

        for i, (label, f_img) in enumerate(sub_configs):
            sy = i * sub_h_single
            sh = sub_h_single if i < num_slots - 1 else (front_h - sy)
            cls.draw_sub_slot_cover(canvas, front_w, sy, sub_w, sh, f_img, label, scale)

        show_pillar_pip = options.get("pillar_pip", False)
        if show_pillar_pip:
            pillar_w = int(320 * scale)
            pillar_h = int(180 * scale)

            if 'left_pillar' in frames:
                lp_x = int(8 * scale)
                lp_y = front_h - pillar_h - int(8 * scale)
                cls.draw_sub_slot_cover(canvas, lp_x, lp_y, pillar_w, pillar_h, frames.get('left_pillar'), "LEFT PILLAR", scale)

            if 'right_pillar' in frames:
                rp_x = front_w - pillar_w - int(8 * scale)
                rp_y = front_h - pillar_h - int(8 * scale)
                cls.draw_sub_slot_cover(canvas, rp_x, rp_y, pillar_w, pillar_h, frames.get('right_pillar'), "RIGHT PILLAR", scale)

        canvas[front_h:out_h, 0:out_w] = (16, 18, 22)
        cv2.line(canvas, (0, front_h), (out_w, front_h), (60, 60, 60), max(1, int(2*scale)))

        data = decoder.get_frame_telemetry(frame_idx, fps) if decoder else {
            "speed_kmh": 0, "steering_deg": 0, "accel_pct": 0, "brake_pct": 0,
            "ap_mode": "MANUAL", "left_blinker": False, "right_blinker": False, "lat": 0.0, "lon": 0.0
        }

        map_w = int(360 * scale) if options.get("map") else 0
        map_h = bottom_h

        if options.get("map"):
            minimap = OpenStreetMapTileLoader.generate_minimap(
                data.get("lat", 0.0), data.get("lon", 0.0), zoom=16, map_w=map_w, map_h=map_h
            )
            canvas[front_h:out_h, 0:map_w] = minimap
            cv2.line(canvas, (map_w, front_h), (map_w, out_h), (60, 60, 60), max(1, int(2*scale)))

        offset_x = map_w
        thick_lbl = max(1, int(2 * scale))
        thick_val = max(2, int(2 * scale))
        thick_num = max(2, int(3 * scale))
        time_sec = frame_idx / fps

        lbl_y = front_h + int(42 * scale)
        val_y = front_h + int(118 * scale)
        mid_y = front_h + int(100 * scale)

        lbl_scale = 0.70 * scale
        lbl_color = (185, 185, 190)

        if options.get("timestamp"):
            t_str = (base_time + timedelta(seconds=time_sec)).strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(canvas, "TIME", (offset_x + int(20 * scale), lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cv2.putText(canvas, t_str, (offset_x + int(20 * scale), val_y), cv2.FONT_HERSHEY_SIMPLEX, 0.85 * scale, (255, 255, 255), thick_val)

        if options.get("speed"):
            cv2.putText(canvas, "SPEED", (offset_x + int(310 * scale), lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cv2.putText(canvas, f"{data['speed_kmh']} km/h", (offset_x + int(310 * scale), val_y + int(4 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 1.25 * scale, (0, 230, 255), thick_num)

        if options.get("fsd"):
            ap_text = data.get("ap_mode", "MANUAL")
            if ap_text == "FSD ACTIVE":
                color = (255, 160, 0)
            elif ap_text == "AUTOPILOT":
                color = (0, 220, 100)
            elif ap_text == "TACC":
                color = (0, 140, 255)
            else:
                color = (140, 140, 140)

            cv2.putText(canvas, "AUTOPILOT", (offset_x + int(520 * scale), lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cv2.putText(canvas, ap_text, (offset_x + int(520 * scale), val_y), cv2.FONT_HERSHEY_SIMPLEX, 0.90 * scale, color, thick_val)

        if options.get("turn_signal"):
            cv2.putText(canvas, "BLINKER", (offset_x + int(760 * scale), lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cls.draw_turn_signals(canvas, offset_x + int(820 * scale), mid_y + int(10 * scale), data['left_blinker'], data['right_blinker'], frame_idx, fps, scale)

        if options.get("steering"):
            cv2.putText(canvas, "STEERING", (offset_x + int(960 * scale), lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cls.draw_rotated_steering(canvas, (offset_x + int(1010 * scale), mid_y + int(12 * scale)), data['steering_deg'], scale)
            cv2.putText(canvas, f"{data['steering_deg']} deg", (offset_x + int(1070 * scale), val_y), cv2.FONT_HERSHEY_SIMPLEX, 0.85 * scale, (220, 220, 220), thick_val)

        if options.get("pedal"):
            cv2.putText(canvas, "PEDALS", (offset_x + int(1220 * scale), lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            pedal_y = front_h + int(82 * scale)
            cls.draw_pedal(canvas, (offset_x + int(1225 * scale), pedal_y), (int(28 * scale), int(50 * scale)), data['accel_pct'], "ACC", (0, 220, 0), scale)
            cls.draw_pedal(canvas, (offset_x + int(1305 * scale), pedal_y), (int(28 * scale), int(50 * scale)), data['brake_pct'], "BRK", (0, 0, 220), scale)

        wm_x = out_w - int(185 * scale)
        wm_y = out_h - int(16 * scale)
        cv2.putText(canvas, "by Companion Turret", (wm_x, wm_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42 * scale, (100, 105, 115), 1)

        return canvas


class ExportWorker(QThread):
    progress = pyqtSignal(int)
    finished = pyqtSignal(str)
    error = pyqtSignal(str)
    cancelled = pyqtSignal()

    def __init__(self, target_clips, options, source_fps, target_fps, export_speed, target_size, out_path):
        super().__init__()
        self.target_clips = target_clips
        self.options = options
        self.source_fps = source_fps
        self.target_fps = target_fps
        self.export_speed = export_speed
        self.target_size = target_size
        self.out_path = out_path
        self.is_stopped = False

    def stop(self):
        self.is_stopped = True

    def run(self):
        out = None
        try:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(self.out_path, fourcc, self.target_fps, self.target_size)

            frame_step = (self.source_fps * self.export_speed) / self.target_fps

            total_output_frames = sum(
                math.ceil((c["end_f"] - c["start_f"] + 1) / frame_step)
                for c in self.target_clips
            )
            processed_frames = 0

            for clip_info in self.target_clips:
                if self.is_stopped: break

                cams = {k: cv2.VideoCapture(v) for k, v in clip_info["cams"].items() if os.path.exists(v)}
                decoder = RealTeslaSEIDecoder(clip_info["cams"]["front"])
                
                start_f = clip_info["start_f"]
                end_f = clip_info["end_f"]

                for k, cap in cams.items():
                    cap.set(cv2.CAP_PROP_POS_FRAMES, start_f)

                curr_f = float(start_f)
                current_cap_pos = {k: start_f for k in cams.keys()}
                last_valid_frames = {}

                while curr_f <= end_f:
                    if self.is_stopped: break

                    target_f = int(curr_f)
                    frames = {}

                    for k, cap in cams.items():
                        pos = current_cap_pos[k]
                        
                        while pos < target_f:
                            cap.grab()
                            pos += 1

                        if pos == target_f:
                            ret, img = cap.read()
                            pos += 1
                            if ret and img is not None and img.size > 0:
                                last_valid_frames[k] = img
                                frames[k] = img
                            elif k in last_valid_frames:
                                frames[k] = last_valid_frames[k]
                            else:
                                frames[k] = None
                        else:
                            cap.set(cv2.CAP_PROP_POS_FRAMES, target_f)
                            ret, img = cap.read()
                            pos = target_f + 1
                            if ret and img is not None and img.size > 0:
                                last_valid_frames[k] = img
                                frames[k] = img
                            elif k in last_valid_frames:
                                frames[k] = last_valid_frames[k]
                            else:
                                frames[k] = None
                        
                        current_cap_pos[k] = pos

                    grid = OverlayRenderer.render(
                        frames, target_f, clip_info["base_time"], self.options, decoder, self.source_fps, self.target_size
                    )
                    out.write(grid)

                    curr_f += frame_step
                    processed_frames += 1
                    
                    if processed_frames % 5 == 0 or processed_frames == total_output_frames:
                        pct = min(100, int((processed_frames / max(1, total_output_frames)) * 100))
                        self.progress.emit(pct)

                for cap in cams.values(): cap.release()

            if out is not None:
                out.release()

            if self.is_stopped:
                if os.path.exists(self.out_path):
                    try:
                        os.remove(self.out_path)
                    except Exception:
                        pass
                self.cancelled.emit()
            else:
                self.progress.emit(100)
                self.finished.emit(self.out_path)

        except Exception as e:
            if out is not None:
                out.release()
            self.error.emit(str(e))


class TeslaStudioPro(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CT Dashcam Studio")
        self.setWindowIcon(self.create_camera_icon())
        self.setGeometry(50, 50, 1600, 920)
        self.apply_dark_theme()

        self.active_clip_list = []
        self.clip_frame_offsets = []
        self.active_decoders = []
        self.caps = {}
        self.current_active_clip_idx = -1

        self.current_item = None
        self.is_current_sentry = False
        self.base_time = datetime.now()
        self.fps, self.total_frames = 36.0, 0
        
        self.last_grid_image = None
        self.last_valid_preview_frames = {}
        self.is_exporting = False
        self.worker = None

        self.start_point = None
        self.end_point = None
        self.detected_event_blocks = []
        self.scanner = None

        self.motion_cache = {}

        self.timer = QTimer()
        self.timer.timeout.connect(self.play_next_frame)
        self.init_ui()

    def create_camera_icon(self):
        pixmap = QPixmap(64, 64)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        painter.setBrush(QColor("#00E6FF"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(8, 24, 48, 32, 6, 6)
        
        painter.drawRoundedRect(18, 14, 28, 16, 4, 4)
        
        painter.setBrush(QColor("#16181c"))
        painter.drawEllipse(20, 26, 24, 24)
        
        painter.setBrush(QColor("#00E6FF"))
        painter.drawEllipse(26, 32, 12, 12)
        
        painter.end()
        return QIcon(pixmap)

    def apply_dark_theme(self):
        self.setStyle(QStyleFactory.create("Fusion"))
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #16181c; color: #d4d4d4; }
            QGroupBox { border: 1px solid #2d3139; border-radius: 6px; margin-top: 8px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px 0 5px; color: #00E6FF; }
            QPushButton { background-color: #0078D7; color: white; border-radius: 4px; padding: 8px; font-weight: bold; font-size: 13px; }
            QPushButton:hover { background-color: #1084ea; }
            QPushButton:disabled { background-color: #2a2d34; color: #666666; }
            QCheckBox { spacing: 8px; font-size: 12px; color: #e0e0e0; }
            QCheckBox::indicator { width: 16px; height: 16px; }
            QCheckBox:disabled { color: #555555; }
            QComboBox { background-color: #22252c; border: 1px solid #2d3139; border-radius: 4px; padding: 5px; color: #ffffff; font-size: 12px; }
            QComboBox:disabled { background-color: #1a1c22; color: #777777; border: 1px solid #22252c; }
            QComboBox::drop-down { border: 0px; }
            QTreeWidget { background-color: #121417; border: 1px solid #2d3139; border-radius: 6px; color: #e0e0e0; font-size: 12px; }
            QTreeWidget::item:selected { background-color: #0078D7; color: #ffffff; }
            QTreeWidget::item:hover { background-color: #22252c; }
        """)

    def init_ui(self):
        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(6, 6, 6, 6)

        self.btn_load = QPushButton("📁 테슬라 폴더 지정 (TeslaCam)")
        self.btn_load.setFixedHeight(38)
        self.btn_load.clicked.connect(self.load_directory_dialog)
        left_layout.addWidget(self.btn_load)

        self.tree_explorer = QTreeWidget()
        self.tree_explorer.setHeaderLabels(["클립 탐색기", "유형 / 채널"])
        self.tree_explorer.setColumnWidth(0, 240)
        self.tree_explorer.itemClicked.connect(self.on_tree_item_clicked)
        left_layout.addWidget(self.tree_explorer, stretch=1)

        opt_box = QGroupBox("오버레이 요소")
        opt_layout = QVBoxLayout()
        self.chks = {
            "timestamp": QCheckBox("실제 시간 (Timestamp)"),
            "speed": QCheckBox("실제 속도 (km/h)"),
            "fsd": QCheckBox("FSD / 자율주행 모드"),
            "turn_signal": QCheckBox("방향지시등 (점멸 깜빡이)"),
            "steering": QCheckBox("스티어링 핸들 각도"),
            "pedal": QCheckBox("가속 / 브레이크 페달"),
            "pillar_pip": QCheckBox("필러 카메라 (PiP 오버레이)"),
            "map": QCheckBox("GPS 미니맵 (OSM)")
        }
        for key, chk in self.chks.items():
            chk.setChecked(True if key != "pillar_pip" else False)
            chk.stateChanged.connect(self.sync_preview)
            opt_layout.addWidget(chk)
            opt_layout.addSpacing(1)
            
        self.chks["pillar_pip"].setEnabled(False)
        opt_box.setLayout(opt_layout)
        left_layout.addWidget(opt_box)

        exp_box = QGroupBox("내보내기 설정")
        exp_layout = QVBoxLayout()
        
        lbl_res_tag = QLabel("출력 해상도:")
        lbl_res_tag.setStyleSheet("font-size: 11px; color: #aaaaaa;")
        exp_layout.addWidget(lbl_res_tag)

        self.combo_res = QComboBox()
        for res_name in RESOLUTIONS.keys():
            self.combo_res.addItem(res_name)
        self.combo_res.currentIndexChanged.connect(self.update_estimated_size)
        exp_layout.addWidget(self.combo_res)

        lbl_exp_speed_tag = QLabel("영상 저장 배속:")
        lbl_exp_speed_tag.setStyleSheet("font-size: 11px; color: #aaaaaa; margin-top: 4px;")
        exp_layout.addWidget(lbl_exp_speed_tag)

        self.combo_export_speed = QComboBox()
        self.combo_export_speed.addItems(["1.0x (표준)", "0.5x (슬로우)", "1.5x (빠르게)", "2.0x (2배속)", "4.0x (4배속)", "5.0x (5배속)"])
        self.combo_export_speed.currentIndexChanged.connect(self.on_export_speed_changed)
        exp_layout.addWidget(self.combo_export_speed)

        lbl_fps_tag = QLabel("출력 프레임레이트 (FPS):")
        lbl_fps_tag.setStyleSheet("font-size: 11px; color: #aaaaaa; margin-top: 4px;")
        exp_layout.addWidget(lbl_fps_tag)

        self.combo_fps = QComboBox()
        self.combo_fps.currentIndexChanged.connect(self.update_estimated_size)
        exp_layout.addWidget(self.combo_fps)

        self.lbl_est_size = QLabel("예상 크기: 약 0 MB")
        self.lbl_est_size.setStyleSheet("color: #00E6FF; font-weight: bold; font-size: 11px; margin-top: 4px; margin-bottom: 4px;")
        exp_layout.addWidget(self.lbl_est_size)

        self.btn_export = QPushButton("선택 구간 내보내기 (MP4)")
        self.btn_export.setFixedHeight(38)
        self.btn_export.clicked.connect(self.on_click_export_button)
        exp_layout.addWidget(self.btn_export)

        self.pbar = QProgressBar()
        self.pbar.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pbar.setFixedHeight(18)
        exp_layout.addWidget(self.pbar)

        exp_box.setLayout(exp_layout)
        left_layout.addWidget(exp_box)

        self.main_splitter.addWidget(left_panel)

        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)
        center_layout.setContentsMargins(6, 6, 6, 6)

        self.preview_lbl = QLabel("좌측 '테슬라 폴더 지정'으로 TeslaCam 폴더를 로드하세요.")
        self.preview_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_lbl.setStyleSheet("background-color: #000; border-radius: 8px; font-size: 15px; color: #888;")
        self.preview_lbl.setMinimumSize(960, 540)
        center_layout.addWidget(self.preview_lbl, stretch=1)

        time_box = QGroupBox("타임라인 컨트롤")
        t_layout = QVBoxLayout()
        
        self.slider = HighlightSlider(Qt.Orientation.Horizontal)
        self.slider.setFixedHeight(24)
        self.slider.valueChanged.connect(self.on_slider_manual_seek)
        t_layout.addWidget(self.slider)

        time_ticks_layout = QHBoxLayout()
        time_ticks_layout.setContentsMargins(2, 0, 2, 0)
        self.lbl_ticks = [QLabel("00:00") for _ in range(5)]
        alignments = [
            Qt.AlignmentFlag.AlignLeft,
            Qt.AlignmentFlag.AlignCenter,
            Qt.AlignmentFlag.AlignCenter,
            Qt.AlignmentFlag.AlignCenter,
            Qt.AlignmentFlag.AlignRight
        ]
        for i, lbl in enumerate(self.lbl_ticks):
            lbl.setStyleSheet("font-size: 10px; color: #888888; font-weight: bold;")
            lbl.setAlignment(alignments[i])
            time_ticks_layout.addWidget(lbl, stretch=1 if 0 < i < 4 else 0)
        t_layout.addLayout(time_ticks_layout)

        status_box = QHBoxLayout()
        status_box.setContentsMargins(0, 2, 0, 2)
        
        self.chk_motion = QCheckBox("모션 분석")
        self.chk_motion.setStyleSheet("font-size: 11px; font-weight: bold; color: #FF9800;")
        self.chk_motion.setVisible(False)
        self.chk_motion.stateChanged.connect(self.on_motion_chk_changed)
        status_box.addWidget(self.chk_motion)

        self.lbl_sensitivity_tag = QLabel("민감도 :")
        self.lbl_sensitivity_tag.setStyleSheet("font-size: 11px; color: #d4d4d4; margin-left: 6px;")
        self.lbl_sensitivity_tag.setVisible(False)
        status_box.addWidget(self.lbl_sensitivity_tag)

        self.combo_sensitivity = QComboBox()
        self.combo_sensitivity.addItems(list(SENSITIVITY_LEVELS.keys()))
        self.combo_sensitivity.setCurrentText("보통")
        self.combo_sensitivity.setFixedWidth(80)
        self.combo_sensitivity.setVisible(False)
        self.combo_sensitivity.currentIndexChanged.connect(self.on_sensitivity_changed)
        status_box.addWidget(self.combo_sensitivity)

        self.lbl_motion_status = QLabel("모션 분석: 대기 중")
        self.lbl_motion_status.setStyleSheet("font-size: 11px; color: #00E6FF; margin-left: 6px;")
        status_box.addWidget(self.lbl_motion_status, stretch=1)

        self.btn_prev_event = QPushButton("◀ 이전 이벤트")
        self.btn_prev_event.setStyleSheet("background-color: #D32F2F; font-size: 11px; padding: 3px 8px;")
        self.btn_prev_event.clicked.connect(self.jump_prev_event)

        self.btn_next_event = QPushButton("다음 이벤트 ▶")
        self.btn_next_event.setStyleSheet("background-color: #D32F2F; font-size: 11px; padding: 3px 8px;")
        self.btn_next_event.clicked.connect(self.jump_next_event)

        self.btn_prev_event.setVisible(False)
        self.btn_next_event.setVisible(False)

        status_box.addWidget(self.btn_prev_event)
        status_box.addWidget(self.btn_next_event)

        t_layout.addLayout(status_box)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.setContentsMargins(0, 4, 0, 0)
        ctrl_layout.setSpacing(6)

        self.btn_prev = QPushButton("◀ 이전 클립")
        self.btn_prev.clicked.connect(self.play_prev_clip)

        self.btn_play = QPushButton("▶ 재생")
        self.btn_play.clicked.connect(self.toggle_play)

        self.btn_next = QPushButton("다음 클립 ▶")
        self.btn_next.clicked.connect(self.play_next_clip)

        self.btn_in = QPushButton("[ 시작점")
        self.btn_in.clicked.connect(self.set_in_point)
        self.btn_out = QPushButton("] 끝점")
        self.btn_out.clicked.connect(self.set_out_point)

        self.btn_reset = QPushButton("↺ 구간 초기화")
        self.btn_reset.setStyleSheet("background-color: #4A5568;")
        self.btn_reset.clicked.connect(self.reset_range_points)

        self.chk_auto_next = QCheckBox("자동 다음 영상")
        self.chk_auto_next.setStyleSheet("font-size: 12px; font-weight: bold; color: #00E6FF; margin-left: 4px;")

        ctrl_layout.addWidget(self.btn_prev)
        ctrl_layout.addWidget(self.btn_play)
        ctrl_layout.addWidget(self.btn_next)
        ctrl_layout.addSpacing(6)
        ctrl_layout.addWidget(self.chk_auto_next)
        ctrl_layout.addSpacing(6)
        ctrl_layout.addWidget(self.btn_in)
        ctrl_layout.addWidget(self.btn_out)
        ctrl_layout.addWidget(self.btn_reset)

        ctrl_layout.addSpacing(12)

        # 배속 조작부 (구간 초기화 버튼 바로 옆에 고정)
        lbl_title = QLabel("배속:")
        lbl_title.setStyleSheet("font-weight: bold; font-size: 12px; color: #d4d4d4;")

        self.slider_preview_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_preview_speed.setRange(1, 10)  # 0.5x ~ 5.0x
        self.slider_preview_speed.setValue(2)     # 기본값 1.0x
        self.slider_preview_speed.setSingleStep(1)
        self.slider_preview_speed.setFixedWidth(70)
        self.slider_preview_speed.valueChanged.connect(self.on_preview_speed_slider_changed)

        self.lbl_preview_speed_val = QLabel("1.0x")
        self.lbl_preview_speed_val.setStyleSheet("font-weight: bold; font-size: 12px; color: #00E6FF;")

        ctrl_layout.addWidget(lbl_title)
        ctrl_layout.addWidget(self.slider_preview_speed)
        ctrl_layout.addWidget(self.lbl_preview_speed_val)

        # 유일한 가변 여백: 배속 조작부까지 모두 좌측에 고정한 뒤 남은 공간 전체를 벌림
        ctrl_layout.addStretch()

        self.lbl_range = QLabel("선택 구간: 미지정 (최대 30분 가능)")
        self.lbl_range.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_range.setStyleSheet("font-size: 15px; font-weight: bold; color: #00E6FF;")
        ctrl_layout.addWidget(self.lbl_range)

        t_layout.addLayout(ctrl_layout)
        time_box.setLayout(t_layout)
        center_layout.addWidget(time_box)

        footer_layout = QHBoxLayout()
        footer_layout.addStretch()
        lbl_footer = QLabel("♥ Companion Turret")
        lbl_footer.setStyleSheet("font-size: 11px; color: #777777; padding-right: 4px;")
        footer_layout.addWidget(lbl_footer)
        center_layout.addLayout(footer_layout)

        self.main_splitter.addWidget(center_panel)

        self.main_splitter.setSizes([350, 1250])
        self.setCentralWidget(self.main_splitter)
        self.update_fps_options()

    def stop_motion_scanner(self):
        if self.scanner is not None:
            self.scanner.stop()
            self.scanner.wait()
            self.scanner = None

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.sync_preview()

    def update_time_ticks(self):
        if self.total_frames <= 0 or self.fps <= 0:
            for lbl in self.lbl_ticks: lbl.setText("00:00")
            return

        total_sec = self.total_frames / float(self.fps)
        for i, lbl in enumerate(self.lbl_ticks):
            sec = (i / 4.0) * total_sec
            m, s = divmod(int(sec), 60)
            lbl.setText(f"{m:02d}:{s:02d}")

    def update_range_highlight(self):
        if not self.active_clip_list or (not self.start_point and not self.end_point):
            self.slider.set_range_points(None, None)
            return

        item_start_dt = self.base_time
        item_end_dt = self.base_time + timedelta(seconds=(self.total_frames / self.fps))

        in_f = None
        out_f = None

        if self.start_point:
            s_dt = self.start_point["dt"]
            if s_dt >= item_end_dt:
                self.slider.set_range_points(None, None)
                return
            elif s_dt <= item_start_dt:
                in_f = 0
            else:
                in_f = int((s_dt - item_start_dt).total_seconds() * self.fps)
        elif self.end_point:
            in_f = 0

        if self.end_point:
            e_dt = self.end_point["dt"]
            if e_dt <= item_start_dt:
                self.slider.set_range_points(None, None)
                return
            elif e_dt >= item_end_dt:
                out_f = self.total_frames
            else:
                out_f = int((e_dt - item_start_dt).total_seconds() * self.fps)
        elif self.start_point:
            out_f = self.total_frames

        if in_f is not None and out_f is not None and in_f < out_f:
            self.slider.set_range_points(in_f, out_f)
        else:
            self.slider.set_range_points(None, None)

    def on_preview_speed_slider_changed(self, val):
        speed_val = val * 0.5
        self.lbl_preview_speed_val.setText(f"{speed_val:.1f}x")
        if self.timer.isActive():
            interval = max(10, int(1000 / (self.fps * speed_val)))
            self.timer.setInterval(interval)

    def load_directory_dialog(self):
        default_dir = os.path.dirname(os.path.abspath(__file__))
        folder = QFileDialog.getExistingDirectory(self, "테슬라 Dashcam 폴더 선택", default_dir)
        if not folder: return

        self.motion_cache.clear()
        self.scan_and_populate_tree(folder)

    def scan_and_populate_tree(self, root_path):
        self.tree_explorer.clear()
        self.reset_range_points()
        self.slider.set_event_blocks([])
        
        cam_suffixes = {
            "front": ["-front.mp4"],
            "back": ["-back.mp4"],
            "left_repeater": ["-left_repeater.mp4"],
            "right_repeater": ["-right_repeater.mp4"],
            "left_pillar": ["-left_pillar.mp4", "-front_left.mp4", "-left_b_pillar.mp4"],
            "right_pillar": ["-right_pillar.mp4", "-front_right.mp4", "-right_b_pillar.mp4"]
        }

        tree_data = {}
        for dirpath, _, filenames in os.walk(root_path):
            front_files = [f for f in filenames if f.endswith("-front.mp4")]
            if not front_files:
                continue

            front_files.sort()
            rel = os.path.relpath(dirpath, root_path)
            parts = rel.split(os.sep) if rel != "." else [os.path.basename(dirpath)]

            category = parts[0] if len(parts) > 1 else "TeslaCam"
            event_name = parts[-1]

            if category not in tree_data:
                tree_data[category] = {}
            if event_name not in tree_data[category]:
                tree_data[category][event_name] = []

            dir_lower = dirpath.lower()
            is_sentry_folder = ("sentryclips" in dir_lower or "sentry" in dir_lower) and not ("savedclips" in dir_lower)
            is_recent_folder = "recentclips" in dir_lower or "recent" in dir_lower

            for ff in front_files:
                prefix = ff[:-10]
                front_path = os.path.join(dirpath, ff)
                cams = {"front": front_path}

                for c_key, s_list in cam_suffixes.items():
                    if c_key == "front": continue
                    for s in s_list:
                        candidate = os.path.join(dirpath, prefix + s)
                        if os.path.exists(candidate):
                            cams[c_key] = candidate
                            break

                ts_display = prefix
                m = re.match(r'(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})', prefix)
                if m:
                    ts_display = f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}"

                tree_data[category][event_name].append({
                    "prefix": prefix,
                    "display": ts_display,
                    "cams": cams,
                    "dirpath": dirpath,
                    "is_sentry": is_sentry_folder,
                    "is_recent": is_recent_folder
                })

        if not tree_data:
            QMessageBox.warning(self, "경고", "선택한 폴더 내에서 테슬라 영상 파일(*-front.mp4)을 찾지 못했습니다.")
            return

        brush_white = QBrush(QColor("#FFFFFF"))
        brush_green = QBrush(QColor("#00FF66"))
        brush_red = QBrush(QColor("#FF3333"))

        for cat_name, events in tree_data.items():
            cat_item = QTreeWidgetItem(self.tree_explorer, [f"📁 {cat_name}", f"{len(events)}개 그룹"])
            cat_item.setExpanded(True)

            for ev_name, clips in events.items():
                is_sentry_group = any(c["is_sentry"] for c in clips)
                is_recent_group = any(c["is_recent"] for c in clips)

                if is_sentry_group and not is_recent_group:
                    tag = "[센트리]"
                    ev_item = QTreeWidgetItem(cat_item, [f"📂 {ev_name} (주차 세션)", f"{tag} {len(clips)}개 영상 통합"])
                    ev_item.setForeground(1, brush_red)
                    ev_item.setData(0, Qt.ItemDataRole.UserRole, {
                        "is_group": True,
                        "is_sentry": True,
                        "clip_list": clips
                    })
                else:
                    ev_item = QTreeWidgetItem(cat_item, [f"📂 {ev_name}", f"{len(clips)}개 클립"])
                    ev_item.setExpanded(True)

                    for clip in clips:
                        front_path = clip["cams"]["front"]
                        ch_count = len(clip["cams"])
                        
                        is_park = RealTeslaSEIDecoder.quick_check_is_parking(front_path)
                        tag = "[주차]" if is_park else "[주행]"
                        
                        clip_item = QTreeWidgetItem(ev_item, [f"🎬 {clip['display']}", f"{tag} {ch_count}채널"])
                        clip_item.setForeground(1, brush_green if is_park else brush_white)
                        clip_item.setData(0, Qt.ItemDataRole.UserRole, {
                            "is_group": False,
                            "is_sentry": is_park,
                            "clip_list": [clip]
                        })

    def get_current_clip_key(self):
        if not self.active_clip_list:
            return None
        return f"{self.active_clip_list[0]['prefix']}_{len(self.active_clip_list)}"

    def on_tree_item_clicked(self, item, column):
        item_data = item.data(0, Qt.ItemDataRole.UserRole)
        if not item_data or "clip_list" not in item_data: return

        self.stop_motion_scanner()

        self.current_item = item
        self.active_clip_list = item_data["clip_list"]
        if not self.active_clip_list: return

        self.active_decoders = []
        self.clip_frame_offsets = [0]
        total_f = 0

        for c_info in self.active_clip_list:
            dec = RealTeslaSEIDecoder(c_info["cams"]["front"])
            self.active_decoders.append(dec)

            cap = cv2.VideoCapture(c_info["cams"]["front"])
            fc = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 2160
            cap.release()

            total_f += fc
            self.clip_frame_offsets.append(total_f)

        self.total_frames = total_f
        self.is_current_sentry = item_data.get("is_sentry", False)

        match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', self.active_clip_list[0]["prefix"])
        if match:
            self.base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S")

        has_pillars = ('left_pillar' in self.active_clip_list[0]["cams"]) or ('right_pillar' in self.active_clip_list[0]["cams"])
        self.chks["pillar_pip"].blockSignals(True)
        if has_pillars:
            self.chks["pillar_pip"].setEnabled(True)
            self.chks["pillar_pip"].setChecked(True)
        else:
            self.chks["pillar_pip"].setChecked(False)
            self.chks["pillar_pip"].setEnabled(False)
        self.chks["pillar_pip"].blockSignals(False)

        self.last_valid_preview_frames.clear()
        self.current_active_clip_idx = -1

        self.fps = 24.0 if has_pillars else 36.0

        self.slider.blockSignals(True)
        self.slider.setRange(0, self.total_frames - 1)
        self.slider.setValue(0)
        self.slider.set_event_blocks([])
        self.slider.blockSignals(False)
        
        self.update_fps_options()
        self.update_time_ticks()
        self.update_range_highlight()
        
        frames = self.read_frames_with_cache(0, is_seeking=True)
        self.render_and_display(frames, 0)

        if self.is_current_sentry:
            self.chk_motion.setVisible(True)
            self.lbl_sensitivity_tag.setVisible(True)
            self.combo_sensitivity.setVisible(True)
            
            if self.chk_motion.isChecked():
                self.btn_prev_event.setVisible(True)
                self.btn_next_event.setVisible(True)
                
                clip_key = self.get_current_clip_key()
                if clip_key in self.motion_cache:
                    cached_blocks = self.motion_cache[clip_key]
                    self.detected_event_blocks = cached_blocks
                    self.slider.set_event_blocks(cached_blocks)
                    self.lbl_motion_status.setText(f"모션 분석 완료: {len(cached_blocks)}개 구간 감지")
                else:
                    self.start_light_motion_scanner(self.active_clip_list)
            else:
                self.btn_prev_event.setVisible(False)
                self.btn_next_event.setVisible(False)
                self.lbl_motion_status.setText("모션 분석: 대기 중")
        else:
            self.chk_motion.setVisible(False)
            self.lbl_sensitivity_tag.setVisible(False)
            self.combo_sensitivity.setVisible(False)
            self.btn_prev_event.setVisible(False)
            self.btn_next_event.setVisible(False)
            self.detected_event_blocks = []
            self.slider.set_event_blocks([])
            self.lbl_motion_status.setText("모션 분석: 비활성화")

    def switch_to_clip_index(self, clip_idx):
        if self.current_active_clip_idx == clip_idx:
            return

        for cap in self.caps.values():
            cap.release()

        clip_info = self.active_clip_list[clip_idx]
        self.cams = clip_info["cams"]
        self.caps = {k: cv2.VideoCapture(v) for k, v in self.cams.items() if os.path.exists(v)}
        self.decoder = self.active_decoders[clip_idx]
        self.current_active_clip_idx = clip_idx

    def on_motion_chk_changed(self, state):
        if self.is_current_sentry:
            if state == Qt.CheckState.Checked.value or state == 2:
                self.btn_prev_event.setVisible(True)
                self.btn_next_event.setVisible(True)
                
                clip_key = self.get_current_clip_key()
                if clip_key in self.motion_cache:
                    cached_blocks = self.motion_cache[clip_key]
                    self.detected_event_blocks = cached_blocks
                    self.slider.set_event_blocks(cached_blocks)
                    self.lbl_motion_status.setText(f"모션 분석 완료: {len(cached_blocks)}개 구간 감지")
                else:
                    self.start_light_motion_scanner(self.active_clip_list)
            else:
                self.stop_motion_scanner()
                self.slider.set_event_blocks([])
                self.detected_event_blocks = []
                self.btn_prev_event.setVisible(False)
                self.btn_next_event.setVisible(False)
                self.lbl_motion_status.setText("모션 분석: 대기 중")

    def on_sensitivity_changed(self, idx):
        if self.is_current_sentry and self.chk_motion.isChecked():
            clip_key = self.get_current_clip_key()
            if clip_key in self.motion_cache: del self.motion_cache[clip_key]
            self.start_light_motion_scanner(self.active_clip_list)

    def start_light_motion_scanner(self, clip_list):
        if not self.is_current_sentry:
            return

        self.stop_motion_scanner()

        self.lbl_motion_status.setText("모션 분석 중...")
        self.detected_event_blocks = []

        sensitivity_preset = self.combo_sensitivity.currentText()
        self.scanner = LightMotionScanner(clip_list, fps=self.fps, sensitivity_preset=sensitivity_preset, target_scan_fps=6.0)
        self.scanner.events_found.connect(self.on_motion_events_found)
        self.scanner.start()

    def on_motion_events_found(self, event_blocks):
        if not self.is_current_sentry:
            return

        self.detected_event_blocks = event_blocks
        self.slider.set_event_blocks(event_blocks)

        clip_key = self.get_current_clip_key()
        if clip_key:
            self.motion_cache[clip_key] = event_blocks

        if event_blocks:
            self.lbl_motion_status.setText(f"모션 분석 완료: {len(event_blocks)}개 구간 감지")
        else:
            self.lbl_motion_status.setText("모션 분석 완료: 감지 구간 없음")

    def jump_prev_event(self):
        if not self.detected_event_blocks or not self.is_current_sentry: return
        curr = self.slider.value()
        prev_blocks = [b for b in self.detected_event_blocks if b[0] < curr - 10]
        if prev_blocks:
            self.slider.setValue(prev_blocks[-1][0])

    def jump_next_event(self):
        if not self.detected_event_blocks or not self.is_current_sentry: return
        curr = self.slider.value()
        next_blocks = [b for b in self.detected_event_blocks if b[0] > curr + 10]
        if next_blocks:
            self.slider.setValue(next_blocks[0][0])

    def get_sibling_clip_item(self, direction=1):
        curr_item = self.tree_explorer.currentItem()
        if not curr_item: return None

        parent_item = curr_item.parent()
        if not parent_item: return None

        idx = parent_item.indexOfChild(curr_item)
        target_idx = idx + direction

        if 0 <= target_idx < parent_item.childCount():
            return parent_item.child(target_idx)
        return None

    def play_prev_clip(self):
        target_item = self.get_sibling_clip_item(-1)
        if target_item:
            self.tree_explorer.setCurrentItem(target_item)
            self.on_tree_item_clicked(target_item, 0)
            if not self.timer.isActive():
                self.toggle_play()

    def play_next_clip(self):
        target_item = self.get_sibling_clip_item(1)
        if target_item:
            self.tree_explorer.setCurrentItem(target_item)
            self.on_tree_item_clicked(target_item, 0)
            if not self.timer.isActive():
                self.toggle_play()
        else:
            if self.timer.isActive():
                self.toggle_play()

    def on_slider_manual_seek(self):
        if not self.active_clip_list or self.is_exporting: return
        idx = self.slider.value()
        frames = self.read_frames_with_cache(idx, is_seeking=True)
        self.render_and_display(frames, idx)

    def sync_preview(self):
        if not self.active_clip_list or self.is_exporting: return
        idx = self.slider.value()
        frames = self.read_frames_with_cache(idx, is_seeking=False)
        self.render_and_display(frames, idx)

    def read_frames_with_cache(self, global_idx, is_seeking=False):
        clip_idx = 0
        for i in range(len(self.clip_frame_offsets) - 1):
            if self.clip_frame_offsets[i] <= global_idx < self.clip_frame_offsets[i+1]:
                clip_idx = i
                break
        else:
            clip_idx = len(self.active_clip_list) - 1

        local_idx = global_idx - self.clip_frame_offsets[clip_idx]

        if self.current_active_clip_idx != clip_idx:
            self.switch_to_clip_index(clip_idx)
            is_seeking = True

        frames = {}
        for k in ['front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar']:
            if k in self.caps:
                cap = self.caps[k]
                if is_seeking:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, local_idx)
                ret, img = cap.read()
                if ret and img is not None and img.size > 0:
                    if k != 'front' and not self.is_exporting:
                        img = cv2.resize(img, (480, 270), interpolation=cv2.INTER_NEAREST)
                    self.last_valid_preview_frames[k] = img
                    frames[k] = img
                elif k in self.last_valid_preview_frames:
                    frames[k] = self.last_valid_preview_frames[k]
                else:
                    frames[k] = None
        return frames

    def render_and_display(self, frames, global_idx):
        opts = self.get_current_options()
        clip_idx = self.current_active_clip_idx if self.current_active_clip_idx >= 0 else 0
        local_idx = global_idx - self.clip_frame_offsets[clip_idx]

        grid = OverlayRenderer.render(
            frames, local_idx, self.base_time, opts, self.decoder, self.fps, target_size=(1920, 1080)
        )
        self.last_grid_image = grid.copy()
        self._display_grid_image(grid)

    def _display_grid_image(self, grid_mat):
        grid_rgb = cv2.cvtColor(grid_mat, cv2.COLOR_BGR2RGB)
        h, w, ch = grid_rgb.shape
        qimg = QImage(grid_rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        scaled_pixmap = pixmap.scaled(
            self.preview_lbl.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation
        )
        self.preview_lbl.setPixmap(scaled_pixmap)

    def toggle_play(self):
        if self.is_exporting: return
        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText("▶ 재생")
        else:
            speed_val = self.slider_preview_speed.value() * 0.5
            interval = max(10, int(1000 / (self.fps * speed_val)))
            self.timer.setInterval(interval)
            self.btn_play.setText("⏸ 일시정지")

    def play_next_frame(self):
        if self.slider.value() >= self.total_frames - 1:
            if self.chk_auto_next.isChecked():
                self.play_next_clip()
            else:
                self.toggle_play()
            return

        speed_val = self.slider_preview_speed.value() * 0.5
        step = max(1, int(round(speed_val))) if speed_val >= 1.5 else 1
        next_idx = min(self.total_frames - 1, self.slider.value() + step)

        self.slider.blockSignals(True)
        self.slider.setValue(next_idx)
        self.slider.blockSignals(False)

        frames = self.read_frames_with_cache(next_idx, is_seeking=False)
        self.render_and_display(frames, next_idx)

    def set_in_point(self):
        if self.is_exporting or not self.current_item: return
        curr_frame = self.slider.value()
        curr_dt = self.base_time + timedelta(seconds=(curr_frame / self.fps))

        if self.end_point:
            if curr_dt >= self.end_point["dt"]:
                QMessageBox.warning(self, "경고", "시작점이 끝점보다 뒤이거나 같습니다.\n시간이 역순이므로 지정할 수 없습니다.")
                return

            dur_sec = (self.end_point["dt"] - curr_dt).total_seconds()
            if dur_sec > 1800.0:
                QMessageBox.warning(self, "경고", f"선택 구간이 {dur_sec / 60.0:.1f}분입니다.\n최대 내보내기 가능 길이는 30분(1,800초)입니다.")
                return

        self.start_point = {
            "item": self.current_item,
            "frame": curr_frame,
            "dt": curr_dt,
            "base_time": self.base_time
        }
        
        self.update_range_highlight()
        self.update_labels()
        self.update_estimated_size()

    def set_out_point(self):
        if self.is_exporting or not self.current_item: return
        curr_frame = self.slider.value()
        curr_dt = self.base_time + timedelta(seconds=(curr_frame / self.fps))

        if self.start_point:
            if curr_dt <= self.start_point["dt"]:
                QMessageBox.warning(self, "경고", "끝점이 시작점보다 앞서거나 같습니다.\n시간이 역순이므로 지정할 수 없습니다.")
                return

            dur_sec = (curr_dt - self.start_point["dt"]).total_seconds()
            if dur_sec > 1800.0:
                QMessageBox.warning(self, "경고", f"선택 구간이 {dur_sec / 60.0:.1f}분입니다.\n최대 내보내기 가능 길이는 30분(1,800초)입니다.")
                return

        self.end_point = {
            "item": self.current_item,
            "frame": curr_frame,
            "dt": curr_dt,
            "base_time": self.base_time
        }

        self.update_range_highlight()
        self.update_labels()
        self.update_estimated_size()

    def reset_range_points(self):
        if self.is_exporting: return
        self.start_point = None
        self.end_point = None
        self.update_range_highlight()
        self.update_labels()
        self.update_estimated_size()

    def update_labels(self):
        if self.start_point and self.end_point:
            dur = (self.end_point["dt"] - self.start_point["dt"]).total_seconds()
            t1 = self.start_point["dt"].strftime("%H:%M:%S")
            t2 = self.end_point["dt"].strftime("%H:%M:%S")
            self.lbl_range.setText(f"구간: {t1} ~ {t2} ({int(dur)}초)")
        elif self.start_point:
            t1 = self.start_point["dt"].strftime("%H:%M:%S")
            self.lbl_range.setText(f"시작점: {t1} (끝점 선택 필요)")
        elif self.end_point:
            t2 = self.end_point["dt"].strftime("%H:%M:%S")
            self.lbl_range.setText(f"끝점: {t2} (시작점 선택 필요)")
        else:
            self.lbl_range.setText("선택 구간: 미지정 (최대 30분 가능)")

    def update_fps_options(self):
        has_pillars = False
        if self.active_clip_list:
            has_pillars = ('left_pillar' in self.active_clip_list[0]["cams"]) or ('right_pillar' in self.active_clip_list[0]["cams"])
        base_fps = 24.0 if has_pillars else 36.0

        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0

        max_fps = min(120, int(round(base_fps * export_speed)))

        std_fps_list = [12, 15, 18, 24, 30, 36, 48, 50, 54, 60, 72, 90, 100, 120]
        filtered_fps_list = [val for val in std_fps_list if val <= max_fps]
        if max_fps not in filtered_fps_list:
            filtered_fps_list.append(max_fps)
            filtered_fps_list.sort()

        self.combo_fps.blockSignals(True)
        self.combo_fps.clear()
        for val in filtered_fps_list:
            tag = f"{val} fps (1배속 기준 표준)" if val == int(base_fps) else f"{val} fps"
            self.combo_fps.addItem(tag, val)

        idx = self.combo_fps.findData(max_fps)
        if idx >= 0:
            self.combo_fps.setCurrentIndex(idx)
        self.combo_fps.blockSignals(False)

        if not self.is_exporting:
            self.combo_fps.setEnabled(export_speed > 1.0)

        self.update_estimated_size()

    def on_export_speed_changed(self):
        self.update_fps_options()

    def update_estimated_size(self):
        if not self.start_point or not self.end_point:
            self.lbl_est_size.setText("예상 크기: 약 0 MB")
            return

        dur_sec = (self.end_point["dt"] - self.start_point["dt"]).total_seconds()
        if dur_sec <= 0:
            self.lbl_est_size.setText("예상 크기: 약 0 MB")
            return

        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0

        out_duration_sec = dur_sec / export_speed

        res_key = self.combo_res.currentText()
        preset = RESOLUTIONS.get(res_key, RESOLUTIONS["QHD (2560x1440) - 최고화질"])
        mbps = preset["bitrate_mbps"]
        
        est_mb = (mbps * 1_000_000 / 8.0 * out_duration_sec) / (1024.0 * 1024.0)
        self.lbl_est_size.setText(f"예상 크기: 약 {est_mb:.1f} MB")

    def get_current_options(self):
        opts = {k: v.isChecked() for k, v in self.chks.items()}
        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        opts["export_speed"] = float(m.group(1)) if m else 1.0
        return opts

    def build_target_clip_chain(self):
        if not self.start_point or not self.end_point:
            return None

        start_item = self.start_point["item"]
        end_item = self.end_point["item"]

        if start_item == end_item:
            cdata = start_item.data(0, Qt.ItemDataRole.UserRole)
            if cdata.get("is_group", False):
                s_global = self.start_point["frame"]
                e_global = self.end_point["frame"]
                target_clips = []
                for i, c_info in enumerate(self.active_clip_list):
                    c_start_f = self.clip_frame_offsets[i]
                    c_end_f = self.clip_frame_offsets[i+1] - 1
                    if c_end_f < s_global or c_start_f > e_global:
                        continue
                    local_s = max(0, s_global - c_start_f)
                    local_e = min(c_end_f - c_start_f, e_global - c_start_f)
                    b_time = datetime.now()
                    m = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
                    if m: b_time = datetime.strptime(m.group(1), "%Y-%m-%d_%H-%M-%S")
                    target_clips.append({
                        "cams": c_info["cams"],
                        "base_time": b_time,
                        "start_f": local_s,
                        "end_f": local_e
                    })
                return target_clips
            else:
                c_info = cdata["clip_list"][0]
                b_time = datetime.now()
                m = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
                if m: b_time = datetime.strptime(m.group(1), "%Y-%m-%d_%H-%M-%S")
                return [{
                    "cams": c_info["cams"],
                    "base_time": b_time,
                    "start_f": self.start_point["frame"],
                    "end_f": self.end_point["frame"]
                }]

        parent_item = start_item.parent()
        if not parent_item or parent_item != end_item.parent():
            QMessageBox.warning(self, "경고", "시작점과 끝점은 동일한 폴더 내 클립이어야 합니다.")
            return None

        start_idx = parent_item.indexOfChild(start_item)
        end_idx = parent_item.indexOfChild(end_item)

        if start_idx > end_idx:
            QMessageBox.warning(self, "경고", "시간 순서가 올바르지 않습니다.")
            return None

        target_clips = []
        for i in range(start_idx, end_idx + 1):
            child_item = parent_item.child(i)
            cdata = child_item.data(0, Qt.ItemDataRole.UserRole)
            if not cdata or not cdata.get("clip_list"): continue

            c_info = cdata["clip_list"][0]
            cap = cv2.VideoCapture(c_info["cams"]["front"])
            fcount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 2160
            cap.release()

            s_frame = self.start_point["frame"] if i == start_idx else 0
            e_frame = self.end_point["frame"] if i == end_idx else (fcount - 1)

            b_time = datetime.now()
            m = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
            if m: b_time = datetime.strptime(m.group(1), "%Y-%m-%d_%H-%M-%S")

            target_clips.append({
                "cams": c_info["cams"],
                "base_time": b_time,
                "start_f": s_frame,
                "end_f": e_frame
            })

        return target_clips

    def set_controls_enabled(self, enabled):
        self.btn_load.setEnabled(enabled)
        self.combo_res.setEnabled(enabled)
        self.combo_export_speed.setEnabled(enabled)
        
        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0
        self.combo_fps.setEnabled(enabled and export_speed > 1.0)

        self.btn_prev.setEnabled(enabled)
        self.btn_play.setEnabled(enabled)
        self.btn_next.setEnabled(enabled)
        self.chk_auto_next.setEnabled(enabled)
        self.btn_in.setEnabled(enabled)
        self.btn_out.setEnabled(enabled)
        self.btn_reset.setEnabled(enabled)
        self.btn_export.setEnabled(enabled)
        self.slider.setEnabled(enabled)
        self.slider_preview_speed.setEnabled(enabled)

    def on_click_export_button(self):
        if self.is_exporting:
            self.cancel_export()
        else:
            self.start_export()

    def start_export(self):
        if not self.start_point or not self.end_point:
            QMessageBox.warning(self, "경고", "시작점과 끝점을 모두 지정해야 합니다.")
            return

        target_clips = self.build_target_clip_chain()
        if not target_clips: return

        default_path = os.path.join(os.getcwd(), "tesla_pro_output.mp4")
        path, _ = QFileDialog.getSaveFileName(self, "저장", default_path, "MP4 (*.mp4)")
        if not path: return

        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText("▶ 재생")

        self.is_exporting = True
        self.set_controls_enabled(False)
        self.btn_export.setText("🛑 렌더링 취소")
        self.btn_export.setEnabled(True)
        self.btn_export.setStyleSheet("background-color: #D32F2F; color: white; border-radius: 4px; padding: 8px; font-weight: bold; font-size: 13px;")

        self.on_export_progress(0)

        res_key = self.combo_res.currentText()
        target_size = RESOLUTIONS.get(res_key, RESOLUTIONS["QHD (2560x1440) - 최고화질"])["size"]

        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0

        source_fps = self.fps or 36.0
        target_fps = self.combo_fps.currentData() or min(120, int(round(source_fps * export_speed)))
        opts = self.get_current_options()
        
        self.worker = ExportWorker(
            target_clips, opts, source_fps, target_fps, export_speed, target_size, path
        )
        self.worker.progress.connect(self.on_export_progress)
        self.worker.finished.connect(self.export_done)
        self.worker.error.connect(self.export_err)
        self.worker.cancelled.connect(self.export_cancelled)
        self.worker.start()

    def cancel_export(self):
        if self.worker is not None and self.is_exporting:
            self.btn_export.setText("취소 중...")
            self.btn_export.setEnabled(False)
            self.worker.stop()

    def export_cancelled(self):
        self.is_exporting = False
        self.set_controls_enabled(True)
        self.btn_export.setText("선택 구간 내보내기 (MP4)")
        self.btn_export.setStyleSheet("")
        self.pbar.setValue(0)
        self.sync_preview()
        QMessageBox.information(self, "취소됨", "영상 렌더링 작업이 취소되었습니다.")

    def on_export_progress(self, val):
        self.pbar.setValue(val)
        if self.last_grid_image is not None:
            overlay_frame = OverlayRenderer.apply_rendering_overlay(self.last_grid_image, val)
            self._display_grid_image(overlay_frame)

    def export_done(self, path):
        self.is_exporting = False
        self.set_controls_enabled(True)
        self.btn_export.setText("선택 구간 내보내기 (MP4)")
        self.btn_export.setStyleSheet("")
        self.sync_preview()
        QMessageBox.information(self, "완료", f"메타데이터 및 대시보드 오버레이 렌더링 완료:\n{path}")

    def export_err(self, err):
        self.is_exporting = False
        self.set_controls_enabled(True)
        self.btn_export.setText("선택 구간 내보내기 (MP4)")
        self.btn_export.setStyleSheet("")
        self.sync_preview()
        QMessageBox.critical(self, "오류", f"렌더링 실패:\n{err}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = TeslaStudioPro()
    win.show()
    sys.exit(app.exec())