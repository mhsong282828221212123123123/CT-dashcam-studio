import sys
import os
import math
import subprocess
import cv2
import imageio_ffmpeg
from PyQt6.QtCore import QThread, pyqtSignal
from core.decoder import RealTeslaSEIDecoder
from core.renderer import OverlayRenderer


class ExportWorker(QThread):
    progress = pyqtSignal(int)
    finished = pyqtSignal(str)
    error = pyqtSignal(str)
    cancelled = pyqtSignal()

    def __init__(self, target_clips, options, source_fps, target_fps, export_speed, target_size, out_path, active_decoders=None):
        super().__init__()
        self.target_clips = target_clips
        self.options = options
        self.source_fps = source_fps
        self.target_fps = target_fps
        self.export_speed = export_speed
        self.target_size = target_size
        self.out_path = out_path
        self.active_decoders = active_decoders or []
        self.is_stopped = False

    def stop(self):
        self.is_stopped = True

    def run(self):
        ffmpeg_proc = None
        try:
            try:
                ffmpeg_exe_path = imageio_ffmpeg.get_ffmpeg_exe()
            except Exception as e:
                self.error.emit(f"내장 FFmpeg 모듈 연결 실패: {str(e)}")
                return

            if not ffmpeg_exe_path or not os.path.exists(ffmpeg_exe_path):
                self.error.emit("내장 FFmpeg 바이너리를 찾을 수 없습니다. 렌더링을 취소합니다.")
                return

            cmd = [
                ffmpeg_exe_path, "-y",
                "-f", "rawvideo",
                "-vcodec", "rawvideo",
                "-s", f"{self.target_size[0]}x{self.target_size[1]}",
                "-pix_fmt", "bgr24",
                "-r", str(self.target_fps),
                "-i", "-",
                "-c:v", "libx264",
                "-preset", "fast",
                "-crf", "26",
                "-pix_fmt", "yuv420p",
                self.out_path
            ]
            creation_flags = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0
            ffmpeg_proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                creationflags=creation_flags
            )

            frame_step = (self.source_fps * self.export_speed) / self.target_fps

            total_output_frames = sum(
                math.ceil((c["end_f"] - c["start_f"] + 1) / frame_step)
                for c in self.target_clips
            )
            processed_frames = 0

            for clip_idx, clip_info in enumerate(self.target_clips):
                if self.is_stopped:
                    break

                cams = {k: cv2.VideoCapture(v) for k, v in clip_info["cams"].items() if os.path.exists(v)}
                
                # 기존 디코더가 있으면 재사용, 없으면 새로 파싱
                if clip_idx < len(self.active_decoders) and self.active_decoders[clip_idx] is not None:
                    decoder = self.active_decoders[clip_idx]
                else:
                    decoder = RealTeslaSEIDecoder(clip_info["cams"]["front"])
                
                start_f = clip_info["start_f"]
                end_f = clip_info["end_f"]

                for k, cap in cams.items():
                    cap.set(cv2.CAP_PROP_POS_FRAMES, start_f)

                curr_f = float(start_f)
                current_cap_pos = {k: start_f for k in cams.keys()}
                last_valid_frames = {}

                while curr_f <= end_f:
                    if self.is_stopped:
                        break

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

                    try:
                        ffmpeg_proc.stdin.write(grid.tobytes())
                    except Exception as e:
                        self.error.emit(f"FFmpeg 파이프 쓰기 오류: {str(e)}")
                        return

                    processed_frames += 1
                    curr_f += frame_step

                    if total_output_frames > 0:
                        pct = int((processed_frames / total_output_frames) * 100)
                        self.progress.emit(min(99, pct))

                for cap in cams.values():
                    cap.release()

            if self.is_stopped:
                if ffmpeg_proc:
                    ffmpeg_proc.stdin.close()
                    ffmpeg_proc.terminate()
                if os.path.exists(self.out_path):
                    try:
                        os.remove(self.out_path)
                    except Exception:
                        pass
                self.cancelled.emit()
            else:
                if ffmpeg_proc:
                    ffmpeg_proc.stdin.close()
                    ffmpeg_proc.wait()
                self.progress.emit(100)
                self.finished.emit(self.out_path)

        except Exception as e:
            if ffmpeg_proc:
                try:
                    ffmpeg_proc.kill()
                except Exception:
                    pass
            self.error.emit(f"영상 렌더링 중 오류 발생: {str(e)}")
