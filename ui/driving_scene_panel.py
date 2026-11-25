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
        # Source follows this Git checkout; only the interpreter may be shared.
        root = next((p for p in candidates if (p / 'src/ct_scene/review_app.py').is_file()), candidates[0])
        source = str(root / 'src')
        if source not in sys.path:
            sys.path.insert(0, source)
        from ct_scene.runtime_paths import runtime_python
        python = runtime_python(root)
        dependencies = str(python.parents[1] / 'Lib/site-packages')
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
