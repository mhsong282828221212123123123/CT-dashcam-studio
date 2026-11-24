"""Host the scene engine's review UI inside Dashcam Studio.

Only the Qt review widget runs here. Analysis keeps its isolated interpreter.
"""
import sys
from pathlib import Path

from PyQt6.QtWidgets import QWidget, QVBoxLayout


class DrivingScenePanel(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName('drivingScenePanel')
        editor = Path(__file__).resolve().parents[1]
        candidates = (editor / 'CT-scene-rebuild', editor.parent / 'CT-scene-rebuild')
        root = next((p for p in candidates if (p / '.venv/Scripts/python.exe').is_file()), candidates[0])
        if not (root / '.venv/Scripts/python.exe').is_file():
            raise FileNotFoundError('주행 재현 엔진을 찾지 못했습니다: ' + str(root))
        source = str(root / 'src')
        if source not in sys.path:
            sys.path.insert(0, source)
        # The source-workspace installation already carries manifest validation
        # dependencies. Append (never prefer) it; existing editor Qt/numpy stay
        # loaded from the editor. GPU analysis still uses the engine executable.
        dependencies = str(root / '.venv/Lib/site-packages')
        if dependencies not in sys.path:
            sys.path.append(dependencies)
        from ct_scene.review_app import ReviewWindow
        self.review = ReviewWindow()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.review)

    def select_front(self, front):
        if front and self.review.process is None:
            self.review.source.setText(str(front))
