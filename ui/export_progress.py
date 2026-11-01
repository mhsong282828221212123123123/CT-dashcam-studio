from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar, QPushButton
from core.i18n import tr
from core.themes import set_theme_style


class ExportProgressDialog(QDialog):
    """Nonblocking window-modal progress; Escape/close request cancellation once."""
    cancel_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.Dialog | Qt.WindowType.FramelessWindowHint)
        self.setWindowModality(Qt.WindowModality.WindowModal)
        self.setWindowTitle(tr("export_progress_title"))
        self.setMinimumWidth(380)
        self._cancel_pending = False
        self._finished = False
        self.setObjectName("ExportProgressDialog")
        set_theme_style(self, """
            QDialog#ExportProgressDialog {
                background:@ct-dialog-bg; color:@ct-text-main;
                border:1px solid @ct-control-border; border-radius:10px;
            }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(18)
        self.title_label = QLabel(tr("export_progress_title"))
        set_theme_style(self.title_label, "color:@ct-text-main; font-size:@font-title; font-weight:600;")
        header = QHBoxLayout()
        header.addWidget(self.title_label)
        header.addStretch()
        self.percent_label = QLabel("0%")
        set_theme_style(self.percent_label, "color:@ct-text-main; font-size:@font-body; font-weight:600;")
        header.addWidget(self.percent_label)
        layout.addLayout(header)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setFixedHeight(24)
        layout.addWidget(self.progress_bar)
        self.cancel_button = QPushButton(tr("btn_cancel"))
        self.cancel_button.setMinimumHeight(36)
        self.cancel_button.setAutoDefault(False)
        self.cancel_button.clicked.connect(self.request_cancel)
        layout.addWidget(self.cancel_button)

    def set_progress(self, percent):
        value = max(0, min(100, int(percent)))
        self.progress_bar.setValue(value)
        self.percent_label.setText(f"{value}%")

    def request_cancel(self):
        if self._finished or self._cancel_pending:
            return
        self._cancel_pending = True
        self.cancel_button.setEnabled(False)
        self.cancel_button.setText(tr("btn_export_cancelling"))
        self.cancel_requested.emit()

    def reject(self):
        # Keep progress visible while the worker closes FFmpeg and releases files.
        self.request_cancel()

    def closeEvent(self, event):
        if self._finished:
            event.accept()
        else:
            self.request_cancel()
            event.ignore()

    def finish(self):
        self._finished = True
        self.done(QDialog.DialogCode.Accepted)

    def center_on_parent(self):
        parent = self.parentWidget()
        if parent is not None:
            self.adjustSize()
            self.move(parent.frameGeometry().center() - self.rect().center())
