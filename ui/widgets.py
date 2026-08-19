from PyQt6.QtWidgets import QLabel, QSlider, QStyleOptionSlider, QStyle
from PyQt6.QtCore import Qt, pyqtSignal, QRect
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


