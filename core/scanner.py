import os
import cv2
from PyQt6.QtCore import QThread, pyqtSignal
from core.constants import SENSITIVITY_LEVELS


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
            if self.is_stopped:
                break
            
            valid_cams = {k: v for k, v in clip_data["cams"].items() 
                          if k in ['front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar'] and os.path.exists(v)}
            if not valid_cams:
                continue

            caps = {k: cv2.VideoCapture(v) for k, v in valid_cams.items()}
            ring_buffers = {k: [] for k in valid_cams.keys()}

            frame_idx = 0
            while not self.is_stopped:
                active_any = False
                for k, cap in caps.items():
                    if self.is_stopped:
                        break
                    
                    if frame_idx % step != 0:
                        ret = cap.grab()
                        if ret:
                            active_any = True
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

            for cap in caps.values():
                cap.release()
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
