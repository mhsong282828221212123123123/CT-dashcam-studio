import sys
import os

# PyInstaller / Nuitka 및 로컬 실행 시 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import cv2
from PyQt6.QtWidgets import QApplication, QStyleFactory

# 하위 호환성을 위한 Re-export
from core.constants import APP_VERSION, GITHUB_REPO, RESOLUTIONS, SENSITIVITY_LEVELS
from core.utils import parse_version_tuple, is_newer_version, resource_path, get_app_dir, get_local_ip
from core.decoder import RealTeslaSEIDecoder
from core.map_loader import OpenStreetMapTileLoader
from core.renderer import OverlayRenderer
from core.scanner import LightMotionScanner
from core.exporter import ExportWorker
from network.server import LocalVideoShareServer, ThreadingHTTPServer, VideoShareHTTPHandler
from network.updater import UpdateCheckWorker
from ui.widgets import ClickableLabel, HighlightSlider
from ui.dialogs import QRShareDialog
from ui.loader_worker import ClipLoaderWorker
from ui.main_window import CTDashcamStudio, TeslaStudioPro

# OpenCL 하드웨어 가속 활성화
cv2.ocl.setUseOpenCL(True)


def main():
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create("Fusion"))

    window = CTDashcamStudio()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
