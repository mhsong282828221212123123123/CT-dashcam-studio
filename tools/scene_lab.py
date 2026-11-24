"""Launch the experimental Scene Lab independently of the main window."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.typography import configure_font_backend
configure_font_backend()
from PyQt6.QtWidgets import QApplication
from core.themes import apply_theme
from ui.scene_dialog import SceneAnalysisDialog

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result',type=Path)
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--review',type=Path)
    args=parser.parse_args()
    app=QApplication(sys.argv[:1]); apply_theme('graphite',persist=False)
    if args.review:
        from ui.scene_review_dialog import SceneReviewDialog
        dialog=SceneReviewDialog(args.review)
    else:
        dialog=SceneAnalysisDialog()
        if args.result: dialog.load_result(args.result)
        if args.demo or not args.result: dialog.view_mode.setCurrentIndex(1)
    dialog.show()
    sys.exit(app.exec())
