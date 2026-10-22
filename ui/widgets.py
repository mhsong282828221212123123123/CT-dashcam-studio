from datetime import timedelta
from PyQt6.QtWidgets import (
    QLabel, QSlider, QStyleOptionSlider, QStyle, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QFrame, QToolTip,
    QGraphicsOpacityEffect
)
from PyQt6.QtCore import Qt, pyqtSignal, QRect, QRectF, QPoint, QPointF, QTimer, QPropertyAnimation, QEasingCurve
from PyQt6.QtGui import QPainter, QColor, QPen, QLinearGradient, QPolygonF


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
        self.fps = 36.0
        self._hover_x = None
        self._is_dragging = False
        self.setFixedHeight(32)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def set_fps(self, fps):
        if fps and fps > 0:
            self.fps = float(fps)

    def set_event_blocks(self, blocks):
        self.event_blocks = blocks if blocks else []
        self.update()

    def set_range_points(self, in_f, out_f):
        self.in_frame = in_f
        self.out_frame = out_f
        self.update()

    def _track_geometry(self):
        pad_x = 12
        gx = pad_x
        gw = max(1, self.width() - pad_x * 2)
        cy = 19
        th = 8
        return gx, gw, cy, th

    def _x_to_value(self, x_pos):
        gx, gw, _, _ = self._track_geometry()
        val_pct = max(0.0, min(1.0, (x_pos - gx) / float(gw)))
        return int(round(self.minimum() + val_pct * (self.maximum() - self.minimum())))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.maximum() > self.minimum():
            self._is_dragging = True
            pos_x = event.position().x() if hasattr(event, "position") else event.pos().x()
            new_val = self._x_to_value(pos_x)
            self.setSliderDown(True)
            self.setValue(new_val)
            self.sliderMoved.emit(new_val)
            event.accept()
            self.update()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        pos = event.position().toPoint() if hasattr(event, "position") else event.pos()
        self._hover_x = pos.x()
        val_range = self.maximum() - self.minimum()
        if val_range > 0:
            hover_f = self._x_to_value(pos.x())
            if self._is_dragging and (event.buttons() & Qt.MouseButton.LeftButton):
                self.setValue(hover_f)
                self.sliderMoved.emit(hover_f)
            sec = int(max(0.0, hover_f / max(1.0, self.fps)))
            t_str = str(timedelta(seconds=sec))
            QToolTip.showText(self.mapToGlobal(QPoint(pos.x(), -18)), f"⏱ {t_str}", self)
        self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._is_dragging and event.button() == Qt.MouseButton.LeftButton:
            self._is_dragging = False
            self.setSliderDown(False)
            self.update()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event):
        self._hover_x = None
        self.update()
        super().leaveEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        gx, gw, cy, th = self._track_geometry()
        t_top = cy - th // 2
        t_bot = t_top + th

        min_val = self.minimum()
        max_val = self.maximum()
        val_range = max_val - min_val

        # 1. 상단 타임라인 눈금(Ruler Ticks)
        for i in range(0, 41):
            tx = gx + (gw * i) / 40.0
            is_major = (i % 5 == 0)
            painter.setPen(QPen(QColor(160, 175, 200, 55 if is_major else 24), 1.0))
            tick_top = 5.0 if is_major else 7.5
            painter.drawLine(QPointF(tx, tick_top), QPointF(tx, 10.5))

        # 2. 베이스 트랙 그루브 (다크 슬레이트 캡슐)
        track_rect = QRectF(gx, t_top, gw, th)
        painter.setPen(QPen(QColor("#283040"), 1.0))
        painter.setBrush(QColor("#0F131B"))
        painter.drawRoundedRect(track_rect, 4.0, 4.0)

        if val_range <= 0:
            painter.end()
            return

        # 3. In / Out 선택 구간 배경 틴트 (진행 바/모션 블록의 채도를 해치지 않도록 하단 레이어에 먼저 렌더)
        has_range = (self.in_frame is not None or self.out_frame is not None)
        rx1 = rx2 = r_top = r_bot = 0.0
        if has_range:
            in_pos = ((self.in_frame - min_val) / float(val_range)) * gw if self.in_frame is not None else 0.0
            out_pos = ((self.out_frame - min_val) / float(val_range)) * gw if self.out_frame is not None else float(gw)
            rx1 = gx + max(0.0, min(float(gw), min(in_pos, out_pos)))
            rx2 = gx + max(0.0, min(float(gw), max(in_pos, out_pos)))
            rw = max(2.0, rx2 - rx1)
            r_top = t_top - 3.0
            r_bot = t_bot + 3.0
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(255, 183, 0, 38))
            painter.drawRoundedRect(QRectF(rx1, r_top, rw, r_bot - r_top), 2.0, 2.0)

        # 4. 현재 재생 진행 바 (시안-블루 그라데이션)
        curr_pct = max(0.0, min(1.0, (self.value() - min_val) / float(val_range)))
        curr_x = gx + curr_pct * gw
        prog_w = max(0.0, curr_x - gx)
        if prog_w > 1.0:
            grad = QLinearGradient(gx, 0, curr_x, 0)
            grad.setColorAt(0.0, QColor("#0062AD"))
            grad.setColorAt(1.0, QColor("#00D8FF"))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(grad)
            painter.drawRoundedRect(QRectF(gx, t_top + 1, prog_w, th - 2), 3.0, 3.0)

        # 5. 센트리 모션 감지 이벤트 블록 (코랄 레드 하이라이트 필)
        if self.event_blocks:
            painter.setPen(QPen(QColor(255, 140, 155, 225), 0.8))
            painter.setBrush(QColor(255, 52, 84, 235))
            for start_f, end_f in self.event_blocks:
                x1 = gx + ((start_f - min_val) / float(val_range)) * gw
                x2 = gx + ((end_f - min_val) / float(val_range)) * gw
                block_w = max(5.0, x2 - x1)
                painter.drawRoundedRect(QRectF(x1, t_top + 0.5, block_w, th - 1.0), 3.0, 3.0)

        # 6. In / Out 선택 구간 하단 골드 라인 & 좌우 브래킷 핸들 [ ]
        if has_range:
            painter.setPen(QPen(QColor(255, 193, 7, 220), 1.8))
            painter.drawLine(QPointF(rx1, r_bot), QPointF(rx2, r_bot))

            bracket_pen = QPen(QColor("#FFC107"), 2.4)
            bracket_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(bracket_pen)
            arm = 5.0
            # Left [
            painter.drawLine(QPointF(rx1, r_top - 1.0), QPointF(rx1, r_bot + 1.0))
            painter.drawLine(QPointF(rx1, r_top - 1.0), QPointF(rx1 + arm, r_top - 1.0))
            painter.drawLine(QPointF(rx1, r_bot + 1.0), QPointF(rx1 + arm, r_bot + 1.0))
            # Right ]
            painter.drawLine(QPointF(rx2, r_top - 1.0), QPointF(rx2, r_bot + 1.0))
            painter.drawLine(QPointF(rx2 - arm, r_top - 1.0), QPointF(rx2, r_top - 1.0))
            painter.drawLine(QPointF(rx2 - arm, r_bot + 1.0), QPointF(rx2, r_bot + 1.0))

        # 6. 마우스 호버 가이드 라인
        if self._hover_x is not None and gx <= self._hover_x <= gx + gw:
            painter.setPen(QPen(QColor(0, 230, 255, 100), 1.0, Qt.PenStyle.DashLine))
            painter.drawLine(QPointF(self._hover_x, 4.0), QPointF(self._hover_x, t_bot + 4.0))

        # 7. 플레이헤드 (Playhead Handle: 룰러 영역 오각형 캡 + 수직 니들)
        is_active = self._is_dragging or (self._hover_x is not None and abs(self._hover_x - curr_x) <= 10)
        head_color = QColor("#FFFFFF") if is_active else QColor("#00E6FF")
        border_color = QColor("#00E6FF") if is_active else QColor("#0B0E14")

        # 플레이헤드 수직 니들
        painter.setPen(QPen(head_color, 2.2))
        painter.drawLine(QPointF(curr_x, 11.0), QPointF(curr_x, t_bot + 4.5))

        # 상단 플레이헤드 캡 (오각형 마커)
        cap_w = 11.0 if is_active else 10.0
        cap_top = 1.5
        cap_mid = 8.0
        cap_tip = 13.0
        poly = QPolygonF([
            QPointF(curr_x - cap_w / 2.0, cap_top),
            QPointF(curr_x + cap_w / 2.0, cap_top),
            QPointF(curr_x + cap_w / 2.0, cap_mid),
            QPointF(curr_x, cap_tip),
            QPointF(curr_x - cap_w / 2.0, cap_mid),
        ])
        painter.setPen(QPen(border_color, 1.2))
        painter.setBrush(head_color)
        painter.drawPolygon(poly)

        painter.end()


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


class ToastNotification(QFrame):
    """
    모던 다크 테마 플로팅 토스트 알림 위젯
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("ToastNotificationFrame")
        self.setFixedWidth(540)
        self.setStyleSheet("""
            QFrame#ToastNotificationFrame {
                background-color: rgba(24, 28, 36, 0.96);
                border: 1.5px solid #F59E0B;
                border-radius: 8px;
            }
            QLabel#ToastIcon {
                font-size: 20px;
                padding-left: 2px;
            }
            QLabel#ToastMsg {
                color: #F3F4F6;
                font-size: 12px;
                font-weight: 500;
                line-height: 1.4;
            }
            QPushButton#ToastClose {
                background: transparent;
                color: #9CA3AF;
                border: none;
                font-size: 15px;
                font-weight: bold;
                padding: 0px 4px;
            }
            QPushButton#ToastClose:hover {
                color: #FFFFFF;
            }
        """)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(14, 10, 12, 10)
        layout.setSpacing(12)

        self.icon_label = QLabel("🔒", self)
        self.icon_label.setObjectName("ToastIcon")
        layout.addWidget(self.icon_label)

        self.msg_label = QLabel(self)
        self.msg_label.setObjectName("ToastMsg")
        self.msg_label.setWordWrap(True)
        layout.addWidget(self.msg_label, stretch=1)

        self.btn_close = QPushButton("✕", self)
        self.btn_close.setObjectName("ToastClose")
        self.btn_close.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_close.clicked.connect(self.hide_toast)
        layout.addWidget(self.btn_close)

        self.opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self.opacity_effect)
        self.opacity_anim = QPropertyAnimation(self.opacity_effect, b"opacity")

        self.dismiss_timer = QTimer(self)
        self.dismiss_timer.setSingleShot(True)
        self.dismiss_timer.timeout.connect(self.hide_toast)

        self.hide()

    def show_message(self, message: str, duration_ms: int = 5500, icon: str = "🔒"):
        self.msg_label.setText(message)
        self.icon_label.setText(icon)
        self.adjustSize()
        self.reposition()
        self.show()
        self.raise_()

        self.opacity_anim.stop()
        try:
            self.opacity_anim.finished.disconnect(self._on_hide_finished)
        except (TypeError, RuntimeError):
            pass

        self.opacity_anim.setDuration(220)
        self.opacity_anim.setStartValue(0.0)
        self.opacity_anim.setEndValue(1.0)
        self.opacity_anim.setEasingCurve(QEasingCurve.Type.OutCubic)
        self.opacity_anim.start()

        self.dismiss_timer.stop()
        self.dismiss_timer.start(duration_ms)

    def reposition(self):
        if self.parent():
            parent_rect = self.parent().rect()
            x = (parent_rect.width() - self.width()) // 2
            y = 20
            self.move(max(10, x), y)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.hide_toast()
            event.accept()
            return
        super().mousePressEvent(event)

    def hide_toast(self):
        self.dismiss_timer.stop()
        self.opacity_anim.stop()
        try:
            self.opacity_anim.finished.disconnect(self._on_hide_finished)
        except (TypeError, RuntimeError):
            pass

        self.opacity_anim.setDuration(220)
        self.opacity_anim.setStartValue(self.opacity_effect.opacity())
        self.opacity_anim.setEndValue(0.0)
        self.opacity_anim.setEasingCurve(QEasingCurve.Type.InCubic)
        self.opacity_anim.finished.connect(self._on_hide_finished)
        self.opacity_anim.start()

    def _on_hide_finished(self):
        try:
            self.opacity_anim.finished.disconnect(self._on_hide_finished)
        except (TypeError, RuntimeError):
            pass
        self.hide()




