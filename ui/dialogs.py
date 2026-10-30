from core.themes import set_theme_style
import os
import subprocess
import qrcode
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, QApplication, QMessageBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
from network.server import LocalVideoShareServer
from core.i18n import tr


class QRShareDialog(QDialog):
    def __init__(self, video_path, parent=None):
        super().__init__(parent)
        self.video_path = video_path
        self.video_filename = os.path.basename(video_path)
        self.server = LocalVideoShareServer(video_path)

        self.setWindowTitle(f"{tr('qr_dialog_title')}")
        self.setMinimumSize(460, 600)
        self.resize(480, 620)
        set_theme_style(self, """
            QDialog {
                background-color:@ct-bg-app;
                color:@ct-text-main;
            }
            QLabel {
                color:@ct-text-main;
            }
            QPushButton {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                border-radius:6px;
                padding:6px 14px;
                font-size:@font-body;
                font-weight:400;
                min-height:32px;
            }
            QPushButton:hover {
                background-color:@ct-control-bg;
                border-color:@ct-border-active;
            }
            QPushButton:pressed {
                background-color:@ct-control-bg;
            }
            QPushButton#btnPrimary {
                background:@ct-selected-bg;
                border:none;
            }
            QPushButton#btnPrimary:hover {
                background:@ct-selected-bg;
            }
            QPushButton#btnCopy {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                padding:6px 12px;
            }
            QPushButton#btnCopy:hover {
                background-color:@ct-control-bg;
                border-color:@ct-border-active;
            }
        """)

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 18, 20, 18)
        main_layout.setSpacing(12)

        # 1. Header Title & File info
        title_lbl = QLabel(tr("qr_dialog_header"))
        set_theme_style(title_lbl, "font-size:@font-title; font-weight:600; color:@ct-text-main;")
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title_lbl)

        file_size_mb = os.path.getsize(self.video_path) / (1024 * 1024) if os.path.exists(self.video_path) else 0
        info_lbl = QLabel(tr("qr_file_info", filename=self.video_filename, size=f"{file_size_mb:.1f}", ip=self.server.ip))
        set_theme_style(info_lbl, "font-size:@font-body; color:@ct-text-muted;")
        info_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info_lbl.setWordWrap(True)
        main_layout.addWidget(info_lbl)

        # 2. QR Code Box
        qr_container = QFrame()
        qr_container.setFixedSize(210, 210)
        set_theme_style(qr_container, "background-color:@ct-control-bg; border-radius:12px;")
        qr_layout = QVBoxLayout(qr_container)
        qr_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        qr_layout.setContentsMargins(10, 10, 10, 10)

        self.lbl_qr = QLabel()
        self.lbl_qr.setFixedSize(190, 190)
        self.lbl_qr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.render_qr_code()
        qr_layout.addWidget(self.lbl_qr)

        main_layout.addWidget(qr_container, alignment=Qt.AlignmentFlag.AlignCenter)

        # 3. Share URL & Copy button
        url_layout = QHBoxLayout()
        url_layout.setSpacing(8)

        self.url_lbl = QLabel(self.server.share_url)
        set_theme_style(self.url_lbl, "font-size:@font-body; font-weight:400; color:@ct-text-main; background-color:@ct-control-bg; border:1px solid @ct-control-border; border-radius:6px; padding:6px 12px;")
        self.url_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.url_lbl.setMinimumHeight(36)
        url_layout.addWidget(self.url_lbl, stretch=1)

        btn_copy = QPushButton(tr("qr_copy_btn"))
        btn_copy.setObjectName("btnCopy")
        btn_copy.setMinimumWidth(80)
        btn_copy.setMinimumHeight(36)
        btn_copy.clicked.connect(self.copy_url)
        url_layout.addWidget(btn_copy)
        main_layout.addLayout(url_layout)

        # 4. Guide text
        guide_box = QFrame()
        set_theme_style(guide_box, "background-color:@ct-control-bg; border:1px solid @ct-control-border; border-radius:8px; padding:10px;")
        g_layout = QVBoxLayout(guide_box)
        g_layout.setContentsMargins(12, 10, 12, 10)
        g_layout.setSpacing(5)

        g_title = QLabel(tr("qr_guide_title"))
        set_theme_style(g_title, "font-size:@font-body; font-weight:400; color:@ct-warning-text;")
        g_layout.addWidget(g_title)

        g_desc = QLabel(tr("qr_guide_desc"))
        set_theme_style(g_desc, "font-size:@font-body; color:@ct-text-main; line-height:1.45;")
        g_desc.setWordWrap(True)
        g_layout.addWidget(g_desc)
        main_layout.addWidget(guide_box)

        # 5. Bottom action buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        btn_open_folder = QPushButton(tr("qr_open_folder"))
        btn_open_folder.setMinimumHeight(38)
        btn_open_folder.clicked.connect(self.open_containing_folder)
        btn_layout.addWidget(btn_open_folder, stretch=1)

        btn_close = QPushButton(tr("btn_cancel"))
        btn_close.setObjectName("btnPrimary")
        btn_close.setMinimumHeight(38)
        btn_close.clicked.connect(self.accept)
        btn_layout.addWidget(btn_close, stretch=1)

        main_layout.addLayout(btn_layout)

    def render_qr_code(self):
        try:
            qr = qrcode.QRCode(version=1, box_size=5, border=1)
            qr.add_data(self.server.share_url)
            qr.make(fit=True)
            img = qr.make_image(fill_color="#000000", back_color="#FFFFFF").convert("RGB")
            data = img.tobytes("raw", "RGB")
            qimg = QImage(data, img.width, img.height, img.width * 3, QImage.Format.Format_RGB888)
            pix = QPixmap.fromImage(qimg)
            self.lbl_qr.setPixmap(pix.scaled(185, 185, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        except Exception as e:
            self.lbl_qr.setText(tr("qr_failed", err=str(e)))

    def copy_url(self):
        cb = QApplication.clipboard()
        cb.setText(self.server.share_url)
        QMessageBox.information(self, tr("qr_copied_title"), tr("qr_copied_msg", url=self.server.share_url))

    def open_containing_folder(self):
        if os.path.exists(self.video_path):
            subprocess.Popen(["explorer", f'/select,{os.path.abspath(self.video_path)}'])

    def closeEvent(self, event):
        self.server.stop()
        super().closeEvent(event)


class HotkeyGuideDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"{tr('hotkeys_dialog_title')}")
        self.setMinimumSize(640, 680)
        self.resize(660, 700)
        set_theme_style(self, """
            QDialog {
                background-color:@ct-bg-app;
            }
            QLabel {

                background:transparent;
                border:none;
                padding:0px;
            }
            QLabel#sectionTitle {
                color:@ct-text-main;
                font-size:@font-body;
                font-weight:600;
                padding-left:2px;
            }
            QFrame#shortcutsCard {
                background-color:@ct-control-bg;
                border:1px solid @ct-control-border;
                border-radius:12px;
            }
            QLabel#keyBadge {
                background-color:@ct-control-bg;
                color:@ct-text-main;

                font-size:@font-body;
                font-weight:400;
                border:1px solid @ct-control-border;
                border-radius:5px;
                padding:4px 10px;
                min-width:150px;
                max-width:200px;
                min-height:24px;
            }
            QLabel#mouseBadge {
                background-color:@ct-control-bg;
                color:@ct-warning-text;

                font-size:@font-body;
                font-weight:400;
                border:1px solid @ct-control-border;
                border-radius:5px;
                padding:4px 10px;
                min-width:150px;
                max-width:200px;
                min-height:24px;
            }
            QLabel#descLabel {
                color:@ct-text-main;
                font-size:@font-body;
                font-weight:400;
            }
            QPushButton#btnClose {
                background:@ct-selected-bg;
                color:@ct-text-main;
                border:none;
                border-radius:6px;
                padding:6px 28px;
                font-size:@font-body;
                font-weight:400;
                min-height:34px;
            }
            QPushButton#btnClose:hover {
                background:@ct-selected-bg;
            }
            QPushButton#btnClose:pressed {
                background-color:@ct-selected-bg;
            }
        """)
        self.init_ui()

    def init_ui(self):
        from PyQt6.QtWidgets import QGridLayout

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(24, 20, 24, 20)
        main_layout.setSpacing(10)

        # 1. Header Title
        title_lbl = QLabel(f"{tr('hotkeys_dialog_title')}")
        set_theme_style(title_lbl, "font-size:@font-title; font-weight:600; color:@ct-text-main;")
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title_lbl)

        sub_lbl = QLabel(tr("hk_subtitle"))
        set_theme_style(sub_lbl, "font-size:@font-small; color:@ct-text-muted; margin-bottom:2px;")
        sub_lbl.setWordWrap(True)
        sub_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(sub_lbl)

        # 2. Keyboard Shortcuts Card
        kb_sec_lbl = QLabel(tr("hk_section_kb"))
        kb_sec_lbl.setObjectName("sectionTitle")
        main_layout.addWidget(kb_sec_lbl)

        kb_card = QFrame()
        kb_card.setObjectName("shortcutsCard")
        kb_layout = QGridLayout(kb_card)
        kb_layout.setContentsMargins(16, 12, 16, 12)
        kb_layout.setHorizontalSpacing(14)
        kb_layout.setVerticalSpacing(8)

        shortcuts = [
            ("Space", tr("hk_space")),
            ("←  /  →", tr("hk_arrow_1s")),
            ("Shift + ←  /  →", tr("hk_arrow_5s")),
            ("I  /  O", tr("hk_io")),
            ("[  /  ]", tr("hk_brackets")),
            ("Home", tr("hk_home")),
            ("F1  /  ?", tr("hk_f1")),
        ]

        for row_idx, (key_text, desc_text) in enumerate(shortcuts):
            key_badge = QLabel(key_text)
            key_badge.setObjectName("keyBadge")
            key_badge.setWordWrap(True)
            key_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            kb_layout.addWidget(key_badge, row_idx, 0)

            desc_lbl = QLabel(desc_text)
            desc_lbl.setWordWrap(True)
            desc_lbl.setObjectName("descLabel")
            desc_lbl.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
            kb_layout.addWidget(desc_lbl, row_idx, 1)

        main_layout.addWidget(kb_card)

        # 3. Mouse & UI Interaction Card
        mouse_sec_lbl = QLabel(tr("hk_section_mouse"))
        mouse_sec_lbl.setObjectName("sectionTitle")
        main_layout.addWidget(mouse_sec_lbl)

        mouse_card = QFrame()
        mouse_card.setObjectName("shortcutsCard")
        mouse_layout = QGridLayout(mouse_card)
        mouse_layout.setContentsMargins(16, 12, 16, 12)
        mouse_layout.setHorizontalSpacing(14)
        mouse_layout.setVerticalSpacing(8)

        mouse_tips = [
            (tr("hk_mouse_doubleclick"), tr("hk_mouse_doubleclick_desc")),
            (tr("hk_mouse_drag"), tr("hk_mouse_drag_desc")),
            (tr("hk_mouse_tele_reset"), tr("hk_mouse_tele_reset_desc")),
            (tr("hk_mouse_slider_reset"), tr("hk_mouse_slider_reset_desc")),
        ]

        for row_idx, (act_text, desc_text) in enumerate(mouse_tips):
            act_badge = QLabel(act_text)
            act_badge.setObjectName("mouseBadge")
            act_badge.setWordWrap(True)
            act_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            mouse_layout.addWidget(act_badge, row_idx, 0)

            desc_lbl = QLabel(desc_text)
            desc_lbl.setWordWrap(True)
            desc_lbl.setObjectName("descLabel")
            desc_lbl.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
            mouse_layout.addWidget(desc_lbl, row_idx, 1)

        main_layout.addWidget(mouse_card)

        # 4. Bottom Close Button
        btn_close = QPushButton(tr("btn_confirm"))
        btn_close.setObjectName("btnClose")
        btn_close.clicked.connect(self.accept)
        main_layout.addWidget(btn_close, alignment=Qt.AlignmentFlag.AlignCenter)

