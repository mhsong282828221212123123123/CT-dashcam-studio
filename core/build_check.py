"""Offline checks for the portable distribution, invoked with --build-check."""
import json
import subprocess
import traceback
import time
from datetime import datetime
from pathlib import Path


def run(app, report_path):
    import cv2
    import imageio_ffmpeg
    from PyQt6.QtCore import QT_VERSION_STR
    from PyQt6.QtGui import QFontInfo
    from core.themes import PALETTES, apply_theme
    from core.utils import resource_path
    from network.updater import UpdateCheckWorker
    from ui.main_window import CTDashcamStudio
    report_path = Path(report_path).resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report = {"ok": False, "qt": QT_VERSION_STR}
    window = None
    try:
        original_start = UpdateCheckWorker.start
        UpdateCheckWorker.start = lambda self: None
        try:
            window = CTDashcamStudio()
        finally:
            UpdateCheckWorker.start = original_start
        app.processEvents()
        families = {name: QFontInfo(getattr(window, name).font()).family()
                    for name in ("tree_explorer", "btn_export", "lbl_export_res", "combo_lang")}
        assert set(families.values()) == {"Pretendard"}, families
        report["fonts"] = families
        for theme in PALETTES:
            apply_theme(theme, persist=False)
            window._retranslate_tree()
            app.processEvents()
        report["themes"] = list(PALETTES)
        resources = ["icon.ico", "assets/fonts/OFL.txt", "assets/fonts/Pretendard-Regular.otf",
                     "assets/fonts/Pretendard-SemiBold.otf", "assets/fonts/Pretendard-Bold.otf",
                     "models/steering_hud.png", "models/face_detection_yunet.onnx", "models/yolov8n.onnx"]
        for relative in resources:
            assert Path(resource_path(relative)).is_file(), relative
        report["resources"] = resources
        face = cv2.FaceDetectorYN.create(resource_path("models/face_detection_yunet.onnx"), "", (320, 320))
        plate = cv2.dnn.readNetFromONNX(resource_path("models/yolov8n.onnx"))
        assert face is not None and not plate.empty()
        report["models_loaded"] = True
        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        video_path = report_path.with_suffix(".mp4")
        result = subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "lavfi",
                                 "-i", "color=c=black:s=320x180:r=12", "-frames:v", "2",
                                 "-c:v", "libx264", "-y", str(video_path)],
                                capture_output=True, timeout=30,
                                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        assert result.returncode == 0, result.stderr.decode(errors="replace")
        cap = cv2.VideoCapture(str(video_path))
        try:
            assert cap.isOpened() and int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 2
        finally:
            cap.release()
        report["ffmpeg_encode_and_opencv_decode"] = True
        from ui.loader_worker import ClipLoaderWorker
        from core.exporter import ExportWorker
        clip = {"prefix": "2026-10-30_12-00-00", "cams": {"front": str(video_path)},
                "dirpath": str(video_path.parent)}
        loader = ClipLoaderWorker([clip])
        loaded = []
        loader.finished.connect(lambda decoders, offsets, count, fps: loaded.append((count, fps)))
        loader.start()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and (loader.isRunning() or not loaded):
            app.processEvents()
            time.sleep(0.01)
        loader.stop()
        assert loader.wait(3000), "Clip loader thread did not stop"
        assert loaded and loaded[0][0] == 2, f"Clip loader signal missing: {loaded}"
        report["clip_loader_thread"] = True
        output = report_path.with_name(report_path.stem + "-export.mp4")
        options = window.get_current_options()
        options.update(map=False, blur_face=False, blur_plate=False, enhance_plate=False)
        exporter = ExportWorker([{"cams": clip["cams"], "base_time": datetime(2026, 10, 30, 12),
                                  "start_f": 0, "end_f": 1}], options, 12, 12, 1,
                                (960, 540), str(output))
        exported, errors = [], []
        exporter.finished.connect(exported.append)
        exporter.error.connect(errors.append)
        exporter.start()
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and (exporter.isRunning() or not (exported or errors)):
            app.processEvents()
            time.sleep(0.01)
        exporter.stop()
        assert exporter.wait(3000), "Export thread did not stop"
        assert exported and not errors, f"Export thread failed: {errors}"
        cap = cv2.VideoCapture(str(output))
        try:
            assert cap.isOpened() and int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 2
        finally:
            cap.release()
        report["export_thread_and_signal"] = True
        report["ok"] = True
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        if window is not None:
            window.close()
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0 if report["ok"] else 1
