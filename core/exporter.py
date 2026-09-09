import sys
import os
import math
import subprocess
import cv2
import imageio_ffmpeg
from PyQt6.QtCore import QThread, pyqtSignal
from core.decoder import RealTeslaSEIDecoder
from core.renderer import OverlayRenderer
from core.privacy_filter import PrivacyFilter


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

            # 블러 필터 초기화 (카메라 채널별 독립 인스턴스 관리 및 멀티스레드 병렬 가속)
            blur_face = self.options.get("blur_face", False)
            blur_plate = self.options.get("blur_plate", False)
            privacy_enabled = blur_face or blur_plate
            privacy_filters = {}
            camera_buffers = {}
            privacy_executor = None
            if privacy_enabled:
                from concurrent.futures import ThreadPoolExecutor
                privacy_executor = ThreadPoolExecutor(max_workers=min(4, os.cpu_count() or 4))

            from collections import deque
            render_queue = deque()

            for clip_idx, clip_info in enumerate(self.target_clips):
                if self.is_stopped:
                    break

                # 클립 전환 시 트래커 상태 및 버퍼 리셋
                for pf in privacy_filters.values():
                    pf.reset()
                camera_buffers.clear()
                render_queue.clear()

                cams = {k: cv2.VideoCapture(v) for k, v in clip_info["cams"].items() if os.path.exists(v)}
                
                # 기존 디코더가 있으면 재사용, 없으면 새로 파싱
                if clip_idx < len(self.active_decoders) and self.active_decoders[clip_idx] is not None:
                    decoder = self.active_decoders[clip_idx]
                else:
                    decoder = RealTeslaSEIDecoder(clip_info["cams"]["front"])
                
                start_f = clip_info["start_f"]
                end_f = clip_info["end_f"]

                cam_fps = {}
                cam_fc = {}
                for k, cap in cams.items():
                    v_fps = cap.get(cv2.CAP_PROP_FPS)
                    cam_fps[k] = v_fps if v_fps and 10.0 <= v_fps <= 120.0 else self.source_fps
                    cam_fc[k] = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 2160

                front_fps = cam_fps.get("front", self.source_fps if self.source_fps > 0 else 36.0)

                current_cap_pos = {}
                for k, cap in cams.items():
                    t_start = start_f / front_fps if front_fps > 0 else 0.0
                    k_start_f = max(0, min(cam_fc[k] - 1, int(round(t_start * cam_fps[k]))))
                    cap.set(cv2.CAP_PROP_POS_FRAMES, k_start_f)
                    current_cap_pos[k] = k_start_f

                curr_f = float(start_f)
                last_valid_frames = {}
                last_emitted_pct = -1

                while curr_f <= end_f:
                    if self.is_stopped:
                        break

                    target_f = int(curr_f)
                    t_sec = target_f / front_fps if front_fps > 0 else 0.0
                    frames = {}

                    # 레이아웃에 필요한 카메라만 선별하여 불필요한 디코딩 및 블러 연산 완전 차단
                    layout_mode = self.options.get("layout", "기본 (1:3 세로배치)")
                    if layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]:
                        active_cam_keys = {'front'}
                    else:
                        active_cam_keys = {'front', 'back', 'left_repeater', 'right_repeater'}

                    for k in ['front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar']:
                        if k not in active_cam_keys or k not in cams:
                            frames[k] = None
                            continue

                        cap = cams[k]
                        k_fps = cam_fps.get(k, front_fps)
                        k_fc = cam_fc.get(k, 2160)
                        target_k_f = max(0, min(k_fc - 1, int(round(t_sec * k_fps))))

                        pos = current_cap_pos.get(k, -1)
                        img = None
                        ret = False

                        if abs(target_k_f - pos) > 15 or target_k_f < pos:
                            cap.set(cv2.CAP_PROP_POS_FRAMES, target_k_f)
                            ret, img = cap.read()
                            current_cap_pos[k] = target_k_f + 1
                        elif target_k_f == pos:
                            ret, img = cap.read()
                            current_cap_pos[k] = pos + 1
                        elif target_k_f > pos:
                            while pos < target_k_f:
                                cap.grab()
                                pos += 1
                            ret, img = cap.read()
                            current_cap_pos[k] = target_k_f + 1
                        else:
                            # target_k_f == pos - 1
                            img = last_valid_frames.get(k)
                            ret = img is not None

                        if ret and img is not None and img.size > 0:
                            last_valid_frames[k] = img
                            frames[k] = img
                        elif k in last_valid_frames:
                            frames[k] = last_valid_frames[k]
                        else:
                            frames[k] = None

                    # 블러 필터 적용 (4개 카메라 병렬 스레드 가속 + 소급 블러 지원 + detect_interval=2 최적화)
                    if privacy_enabled:
                        valid_keys = [k for k in frames if frames[k] is not None]
                        for k in valid_keys:
                            if k not in privacy_filters:
                                privacy_filters[k] = PrivacyFilter(
                                    blur_face=blur_face,
                                    blur_plate=blur_plate,
                                    detect_interval=4
                                )
                            if k not in camera_buffers:
                                camera_buffers[k] = []

                        def _proc_cam(k):
                            buf = camera_buffers[k]
                            processed = privacy_filters[k].apply_with_backfill(frames[k], buf)
                            buf.append(processed)
                            if len(buf) > 4:
                                buf.pop(0)
                            return k, processed

                        if len(valid_keys) > 1 and privacy_executor is not None:
                            futs = [privacy_executor.submit(_proc_cam, k) for k in valid_keys]
                            for fut in futs:
                                k, processed = fut.result()
                                frames[k] = processed
                        elif len(valid_keys) == 1:
                            k, processed = _proc_cam(valid_keys[0])
                            frames[k] = processed

                    render_queue.append((frames, target_f, clip_info["base_time"], decoder))

                    # 3프레임 지연 큐를 통해 소급 블러 처리 후 인코더로 출력
                    if len(render_queue) >= 3:
                        q_frames, q_target_f, q_base_time, q_decoder = render_queue.popleft()
                        grid = OverlayRenderer.render(
                            q_frames, q_target_f, q_base_time, self.options, q_decoder, self.source_fps, self.target_size
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
                        if pct != last_emitted_pct:
                            last_emitted_pct = pct
                            self.progress.emit(min(99, pct))

                # 클립 종료 시 큐에 남은 프레임 모두 출력
                while render_queue:
                    if self.is_stopped:
                        break
                    q_frames, q_target_f, q_base_time, q_decoder = render_queue.popleft()
                    grid = OverlayRenderer.render(
                        q_frames, q_target_f, q_base_time, self.options, q_decoder, self.source_fps, self.target_size
                    )
                    try:
                        ffmpeg_proc.stdin.write(grid.tobytes())
                    except Exception as e:
                        self.error.emit(f"FFmpeg 파이프 쓰기 오류: {str(e)}")
                        return
                    processed_frames += 1
                    if total_output_frames > 0:
                        pct = int((processed_frames / total_output_frames) * 100)
                        if pct != last_emitted_pct:
                            last_emitted_pct = pct
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
        finally:
            if privacy_executor is not None:
                privacy_executor.shutdown(wait=False)
