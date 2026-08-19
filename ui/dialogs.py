import os
import subprocess
import qrcode
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, QApplication, QMessageBox
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QImage, QPixmap
from network.server import LocalVideoShareServer


class QRShareDialog(QDialog):
    def __init__(self, video_path, parent=None):
        super().__init__(parent)
        self.video_path = video_path
        self.video_filename = os.path.basename(video_path)
        self.server = LocalVideoShareServer(video_path)
        
        self.setWindowTitle("📱 스마트폰 무선 전송 (QR 공유)")
        self.setMinimumSize(460, 600)
        self.resize(480, 620)
        self.setStyleSheet("""
            QDialog {
                background-color: #121418;
                color: #FFFFFF;
            }
            QLabel {
                color: #E0E0E0;
            }
            QPushButton {
                background-color: #22262E;
                color: #FFFFFF;
                border: 1px solid #333842;
                border-radius: 6px;
                padding: 6px 14px;
                font-size: 12px;
                font-weight: bold;
                min-height: 32px;
            }
            QPushButton:hover {
                background-color: #2C323D;
                border-color: #0078D7;
            }
            QPushButton:pressed {
                background-color: #1A1D23;
            }
            QPushButton#btnPrimary {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0078D7, stop:1 #00B4D8);
                border: none;
            }
            QPushButton#btnPrimary:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0086F0, stop:1 #00C6EC);
            }
            QPushButton#btnCopy {
                background-color: #2A303C;
                color: #00E6FF;
                border: 1px solid #3A4252;
                padding: 6px 12px;
            }
            QPushButton#btnCopy:hover {
                background-color: #353D4D;
                border-color: #00E6FF;
            }
        """)
        
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 18, 20, 18)
        main_layout.setSpacing(12)

        # 1. Header Title & File info
        title_lbl = QLabel("📱 스마트폰으로 비디오 무선 전송")
        title_lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #FFFFFF;")
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title_lbl)

        file_size_mb = os.path.getsize(self.video_path) / (1024 * 1024) if os.path.exists(self.video_path) else 0
        info_lbl = QLabel(f"파일명: {self.video_filename}\n용량: {file_size_mb:.1f} MB  |  IP: {self.server.ip}")
        info_lbl.setStyleSheet("font-size: 12px; color: #8E95A5;")
        info_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info_lbl.setWordWrap(True)
        main_layout.addWidget(info_lbl)

        # 2. QR Code Box
        qr_container = QFrame()
        qr_container.setFixedSize(210, 210)
        qr_container.setStyleSheet("background-color: #FFFFFF; border-radius: 12px;")
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
        self.url_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #00E6FF; background-color: #1A1D24; border: 1px solid #2A2E39; border-radius: 6px; padding: 6px 12px;")
        self.url_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.url_lbl.setMinimumHeight(36)
        url_layout.addWidget(self.url_lbl, stretch=1)

        btn_copy = QPushButton("📋 복사")
        btn_copy.setObjectName("btnCopy")
        btn_copy.setMinimumWidth(80)
        btn_copy.setMinimumHeight(36)
        btn_copy.clicked.connect(self.copy_url)
        url_layout.addWidget(btn_copy)
        main_layout.addLayout(url_layout)

        # 4. Guide text
        guide_box = QFrame()
        guide_box.setStyleSheet("background-color: #161920; border: 1px solid #232731; border-radius: 8px; padding: 10px;")
        g_layout = QVBoxLayout(guide_box)
        g_layout.setContentsMargins(12, 10, 12, 10)
        g_layout.setSpacing(5)

        g_title = QLabel("💡 스마트폰 전송 방법")
        g_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #FFA726;")
        g_layout.addWidget(g_title)

        g_desc = QLabel(
            "1. 스마트폰과 PC가 <b>동일한 Wi-Fi(공유기)</b>에 연결되어 있어야 합니다.<br>"
            "2. 스마트폰 기본 <b>카메라 앱</b>을 켜고 위 QR 코드를 비추세요.<br>"
            "3. 화면에 뜨는 링크를 누르면 <b>갤러리에 즉시 저장</b>됩니다."
        )
        g_desc.setStyleSheet("font-size: 11px; color: #A0A6B5; line-height: 1.45;")
        g_desc.setWordWrap(True)
        g_layout.addWidget(g_desc)
        main_layout.addWidget(guide_box)

        # 5. Bottom action buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        btn_open_folder = QPushButton("📁 저장 폴더 열기")
        btn_open_folder.setMinimumHeight(38)
        btn_open_folder.clicked.connect(self.open_containing_folder)
        btn_layout.addWidget(btn_open_folder, stretch=1)

        btn_close = QPushButton("닫기")
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
            self.lbl_qr.setText(f"QR 코드 생성 실패:\n{e}")

    def copy_url(self):
        cb = QApplication.clipboard()
        cb.setText(self.server.share_url)
        QMessageBox.information(self, "복사 완료", f"공유 링크가 클립보드에 복사되었습니다.\n\n{self.server.share_url}")

    def open_containing_folder(self):
        if os.path.exists(self.video_path):
            subprocess.Popen(["explorer", f'/select,{os.path.abspath(self.video_path)}'])

    def closeEvent(self, event):
        self.server.stop()
        super().closeEvent(event)


class HotkeyGuideDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("⌨ 키보드 단축키 안내")
        self.setFixedSize(500, 480)
        self.setStyleSheet("""
            QDialog {
                background-color: #101216;
            }
            QLabel {
                font-family: 'Segoe UI', 'Malgun Gothic', sans-serif;
                background: transparent;
                border: none;
                padding: 0px;
            }
            QFrame#shortcutsCard {
                background-color: #161920;
                border: 1px solid #262B36;
                border-radius: 12px;
            }
            QLabel#keyBadge {
                background-color: #222733;
                color: #00E6FF;
                font-family: 'Consolas', 'Segoe UI', monospace;
                font-size: 11px;
                font-weight: bold;
                border: 1px solid #363E50;
                border-radius: 5px;
                padding: 4px 10px;
                min-width: 120px;
                max-height: 24px;
            }
            QLabel#descLabel {
                color: #FFFFFF;
                font-size: 12px;
                font-weight: 500;
            }
            QPushButton#btnClose {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0078D7, stop:1 #00B4D8);
                color: #FFFFFF;
                border: none;
                border-radius: 6px;
                padding: 6px 28px;
                font-size: 13px;
                font-weight: bold;
                min-height: 34px;
            }
            QPushButton#btnClose:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0086F0, stop:1 #00C6EC);
            }
            QPushButton#btnClose:pressed {
                background-color: #0E5EAA;
            }
        """)
        self.init_ui()

    def init_ui(self):
        from PyQt6.QtWidgets import QGridLayout
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(24, 20, 24, 20)
        main_layout.setSpacing(14)

        # 1. Header Title
        title_lbl = QLabel("⌨ 키보드 단축키 가이드")
        title_lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #00E6FF;")
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title_lbl)

        sub_lbl = QLabel("마우스 없이도 영상을 빠르고 편리하게 탐색하고 편집할 수 있습니다.")
        sub_lbl.setStyleSheet("font-size: 11px; color: #8E95A5; margin-bottom: 4px;")
        sub_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(sub_lbl)

        # 2. Shortcuts Card Box
        card = QFrame()
        card.setObjectName("shortcutsCard")
        card_layout = QGridLayout(card)
        card_layout.setContentsMargins(18, 14, 18, 14)
        card_layout.setHorizontalSpacing(16)
        card_layout.setVerticalSpacing(10)

        shortcuts = [
            ("Space", "재생 / 일시정지 토글"),
            ("←  /  →", "1초 뒤로 / 앞으로 이동"),
            ("Shift + ←  /  →", "5초 고속 뒤로 / 앞으로 이동"),
            ("I", "내보내기 구간 시작점(In) 설정"),
            ("O", "내보내기 구간 종료점(Out) 설정"),
            ("[  /  ]", "이전 클립 / 다음 클립 전환"),
            ("Home", "영상 첫 프레임(0)으로 이동"),
            ("F1  또는  ?", "단축키 안내 창 열기")
        ]

        for row_idx, (key_text, desc_text) in enumerate(shortcuts):
            key_badge = QLabel(key_text)
            key_badge.setObjectName("keyBadge")
            key_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
            card_layout.addWidget(key_badge, row_idx, 0)

            desc_lbl = QLabel(desc_text)
            desc_lbl.setObjectName("descLabel")
            desc_lbl.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
            card_layout.addWidget(desc_lbl, row_idx, 1)

        main_layout.addWidget(card)

        # 3. Bottom Close Button
        btn_close = QPushButton("확인")
        btn_close.setObjectName("btnClose")
        btn_close.clicked.connect(self.accept)
        main_layout.addWidget(btn_close, alignment=Qt.AlignmentFlag.AlignCenter)

