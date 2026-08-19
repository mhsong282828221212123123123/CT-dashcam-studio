import os
import cv2
from PyQt6.QtCore import QThread, pyqtSignal
from core.decoder import RealTeslaSEIDecoder


class ClipLoaderWorker(QThread):
    progress = pyqtSignal(int, str)
    finished = pyqtSignal(list, list, int, object)

    def __init__(self, clip_list, parent=None):
        super().__init__(parent)
        self.clip_list = clip_list
        self.is_stopped = False

    def run(self):
        try:
            decoders = []
            offsets = [0]
            total_f = 0
            detected_fps = None

            total_clips = len(self.clip_list)
            for i, c_info in enumerate(self.clip_list):
                if self.is_stopped:
                    return
                
                self.progress.emit(int((i / total_clips) * 100), f"비디오 정보 로딩 중... ({i+1}/{total_clips})")
                
                front_cam = c_info.get("cams", {}).get("front")
                if front_cam and os.path.exists(front_cam):
                    try:
                        dec = RealTeslaSEIDecoder(front_cam)
                    except Exception:
                        dec = None
                    decoders.append(dec)

                    try:
                        cap = cv2.VideoCapture(front_cam)
                        fc = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 2160
                        v_fps = cap.get(cv2.CAP_PROP_FPS)
                        cap.release()
                    except Exception:
                        fc = 2160
                        v_fps = None

                    if detected_fps is None and v_fps and 10.0 <= v_fps <= 120.0:
                        detected_fps = round(v_fps, 2)
                else:
                    decoders.append(None)
                    fc = 2160

                total_f += fc
                offsets.append(total_f)

            if not self.is_stopped:
                self.progress.emit(100, "로딩 완료")
                self.finished.emit(decoders, offsets, total_f, detected_fps)
        except Exception:
            pass

    def stop(self):
        self.is_stopped = True
