from PyQt6.QtWidgets import (
    QLabel, QSlider, QStyleOptionSlider, QStyle, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal, QRect, QPoint
from PyQt6.QtGui import QPainter, QColor


class ClickableLabel(QLabel):
    clicked = pyqtSignal()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
            event.accept()
            return
        super().mousePressEvent(event)


class HighlightSlider(QSlider):
    def __init__(self, orientation=Qt.Orientation.Horizontal, parent=None):
        super().__init__(orientation, parent)
        self.event_blocks = []
        self.in_frame = None
        self.out_frame = None

    def set_event_blocks(self, blocks):
        self.event_blocks = blocks if blocks else []
        self.update()

    def set_range_points(self, in_f, out_f):
        self.in_frame = in_f
        self.out_frame = out_f
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            opt = QStyleOptionSlider()
            self.initStyleOption(opt)
            sr = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, opt, QStyle.SubControl.SC_SliderGroove, self)
            hr = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, opt, QStyle.SubControl.SC_SliderHandle, self)
            
            click_x = event.pos().x()
            half_h = hr.width() // 2
            groove_x = sr.x() + half_h
            groove_w = sr.width() - hr.width()
            
            if groove_w > 0:
                val_pct = (click_x - groove_x) / float(groove_w)
                val_pct = max(0.0, min(1.0, val_pct))
                new_val = int(self.minimum() + val_pct * (self.maximum() - self.minimum()))
                self.setValue(new_val)
                event.accept()
                return
        super().mousePressEvent(event)

    def paintEvent(self, event):
        min_val = self.minimum()
        max_val = self.maximum()
        val_range = max_val - min_val
        if val_range <= 0:
            super().paintEvent(event)
            return

        opt = QStyleOptionSlider()
        self.initStyleOption(opt)
        sr = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, opt, QStyle.SubControl.SC_SliderGroove, self)
        hr = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, opt, QStyle.SubControl.SC_SliderHandle, self)

        half_h = hr.width() // 2
        gx = sr.x() + half_h
        gw = max(1, sr.width() - hr.width())
        track_y = self.height() // 2

        # ── 1단계: 슬라이더 기본 렌더링 (그루브 + 핸들) ──
        super().paintEvent(event)

        # ── 2단계: 오버레이 (그루브 위에 그려지고 핸들도 덮임) ──
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 모션 감지 이벤트 블록 (빨간색 바)
        if self.event_blocks:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(255, 60, 60, 200))
            for start_f, end_f in self.event_blocks:
                x1 = gx + int(((start_f - min_val) / val_range) * gw)
                x2 = gx + int(((end_f - min_val) / val_range) * gw)
                block_w = max(4, x2 - x1)
                painter.drawRoundedRect(QRect(x1, track_y - 3, block_w, 6), 2, 2)

        # In/Out 구간 하이라이트 (오렌지/앰버)
        if self.in_frame is not None or self.out_frame is not None:
            in_pos  = int(((self.in_frame  - min_val) / val_range) * gw) if self.in_frame  is not None else 0
            out_pos = int(((self.out_frame - min_val) / val_range) * gw) if self.out_frame is not None else gw
            rect_x = gx + min(in_pos, out_pos)
            rect_w = abs(out_pos - in_pos)
            if rect_w > 0:
                painter.fillRect(QRect(rect_x, track_y - 7, max(2, rect_w), 14), QColor(255, 152, 0, 130))
                painter.setPen(QColor(255, 183, 77, 220))
                painter.drawRect(QRect(rect_x, track_y - 7, max(2, rect_w), 14))

        painter.end()

        # ── 3단계: 핸들만 다시 최상위에 렌더 (오버레이가 핸들을 덮지 않도록) ──
        painter2 = QPainter(self)
        opt2 = QStyleOptionSlider()
        self.initStyleOption(opt2)
        opt2.subControls = QStyle.SubControl.SC_SliderHandle
        self.style().drawComplexControl(QStyle.ComplexControl.CC_Slider, opt2, painter2, self)
        painter2.end()


class SingleCameraPopup(QWidget):
    """
    단일 카메라 모드에서 6개 카메라 중 전체화면으로 볼 카메라를 선택하는 플로팅 팝업.
    - 바깥 클릭 시 자동으로 닫힘 (Qt.WindowType.Popup)
    - 6개 카메라 모두 표시 (전방, 후방, 좌측 리피터, 우측 리피터, 좌측 필러, 우측 필러)
    - 클립에 없는 영상(특히 필러 카메라)은 회색으로 비활성화 (disabled)
    - 현재 선택된 카메라는 시안(Cyan) 하이라이트 스타일 적용
    """
    cam_selected = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.Popup | Qt.WindowType.FramelessWindowHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, False)
        self.setObjectName("SingleCameraPopup")

        # 6개 카메라 설정: (표시명, 카메라 키, 그리드 행, 그리드 열)
        self.cam_configs = [
            ("📷 전방 (FRONT)", "front", 0, 0),
            ("🚗 후방 (REAR)", "back", 0, 1),
            ("◀ 좌측 리피터 (LEFT)", "left_repeater", 1, 0),
            ("우측 리피터 (RIGHT) ▶", "right_repeater", 1, 1),
            ("◀ 좌측 필러 (L-PILLAR)", "left_pillar", 2, 0),
            ("우측 필러 (R-PILLAR) ▶", "right_pillar", 2, 1),
        ]
        self.cam_buttons = {}
        self.current_cam = "front"

        self._init_ui()

    def _init_ui(self):
        self.setStyleSheet("""
            QWidget#SingleCameraPopup {
                background-color: #14171E;
                border: 1.5px solid #00E6FF;
                border-radius: 8px;
            }
        """)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(14, 12, 14, 14)
        main_layout.setSpacing(10)

        # 상단 헤더
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)

        lbl_title = QLabel("📷 단일 카메라 뷰 선택")
        lbl_title.setStyleSheet("color: #00E6FF; font-size: 12px; font-weight: bold;")
        header_layout.addWidget(lbl_title)

        header_layout.addStretch()
        main_layout.addLayout(header_layout)

        # 구분선
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        line.setStyleSheet("background-color: #262E3B; border: none; max-height: 1px;")
        main_layout.addWidget(line)

        # 2열 3행 카메라 버튼 그리드
        grid = QGridLayout()
        grid.setSpacing(6)

        for label, key, row, col in self.cam_configs:
            btn = QPushButton(label)
            btn.setFixedHeight(34)
            btn.setMinimumWidth(138)
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            btn.clicked.connect(lambda checked, k=key: self._on_btn_clicked(k))
            self.cam_buttons[key] = btn
            grid.addWidget(btn, row, col)

        main_layout.addLayout(grid)

    def _on_btn_clicked(self, cam_key):
        self.current_cam = cam_key
        self.cam_selected.emit(cam_key)
        self.close()

    def update_states(self, available_cams, active_cam):
        """
        available_cams: 현재 클립에 존재하는 카메라 키 집합 (set)
        active_cam: 현재 선택된 카메라 키 (str)
        """
        self.current_cam = active_cam

        active_style = """
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0066AA, stop:1 #008CE3);
                color: #FFFFFF;
                border: 1.5px solid #00E6FF;
                border-radius: 5px;
                font-size: 11px;
                font-weight: bold;
                padding: 4px 6px;
            }
        """
        enabled_style = """
            QPushButton {
                background-color: #1A1F29;
                color: #D2D9E5;
                border: 1px solid #333D4F;
                border-radius: 5px;
                font-size: 11px;
                font-weight: bold;
                padding: 4px 6px;
            }
            QPushButton:hover {
                background-color: #262E3E;
                border: 1px solid #00B4D8;
                color: #FFFFFF;
            }
            QPushButton:pressed {
                background-color: #005F9E;
            }
        """
        disabled_style = """
            QPushButton {
                background-color: #121419;
                color: #4E5666;
                border: 1px solid #1D212A;
                border-radius: 5px;
                font-size: 11px;
                padding: 4px 6px;
            }
        """

        for _, key, _, _ in self.cam_configs:
            btn = self.cam_buttons.get(key)
            if not btn:
                continue

            # 필러쪽 카메라 및 일반 카메라 가용성 체크
            is_available = key in available_cams
            btn.setEnabled(is_available)

            if not is_available:
                btn.setStyleSheet(disabled_style)
                btn.setToolTip("현재 클립에 이 카메라 영상이 없습니다.")
            elif key == active_cam:
                btn.setStyleSheet(active_style)
                btn.setToolTip("현재 전체화면으로 표시 중인 카메라입니다.")
            else:
                btn.setStyleSheet(enabled_style)
                btn.setToolTip("클릭하여 이 카메라 화면을 전체 화면으로 표시")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.close()
            event.accept()
            return
        super().keyPressEvent(event)



