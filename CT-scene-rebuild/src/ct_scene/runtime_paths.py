"""Locate a reusable interpreter without replacing this checkout's source."""
from pathlib import Path


def runtime_python(source_root):
    root=Path(source_root).resolve()
    candidates=[root/'.venv/Scripts/python.exe',root.parent/'CT-scene-rebuild/.venv/Scripts/python.exe',root.parent.parent/'CT-scene-rebuild/.venv/Scripts/python.exe']
    for p in candidates:
        if p.is_file():return p
    raise FileNotFoundError('주행 재현 분석 실행 환경을 찾지 못했습니다: '+str(root))


def scene_models_root(source_root):
    root=Path(source_root).resolve()
    candidates=[root.parent/'scene-models',root.parent.parent/'scene-models']
    return next((p for p in candidates if (p/'yolo11x.onnx').is_file() and (p/'model-review/yolopv2-roads.onnx').is_file()),candidates[0])
