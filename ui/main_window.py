import sys
import os
import re
import math
import time
import subprocess
import webbrowser
from datetime import datetime, timedelta

import cv2
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QCheckBox, QFileDialog, QGroupBox,
    QProgressBar, QMessageBox, QComboBox, QTreeWidget, QTreeWidgetItem,
    QSplitter, QStyleFactory, QDialog, QProgressDialog, QApplication,
    QSizePolicy, QSlider
)
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QImage, QPixmap, QColor, QBrush, QIcon, QKeySequence, QShortcut

from core.constants import APP_VERSION, RESOLUTIONS, SENSITIVITY_LEVELS
from core.utils import resource_path, get_app_dir
from core.decoder import RealTeslaSEIDecoder
from core.renderer import OverlayRenderer
from core.scanner import LightMotionScanner
from core.exporter import ExportWorker
from network.updater import UpdateCheckWorker
from ui.widgets import ClickableLabel, HighlightSlider
from ui.dialogs import QRShareDialog, HotkeyGuideDialog
from ui.loader_worker import ClipLoaderWorker


class CTDashcamStudio(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CT Dashcam Studio")
        self.resize(1660, 880)
        self.setMinimumSize(1150, 680)

        icon_path = resource_path("icon.ico")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))

        self.cams = {}
        self.caps = {}
        self.decoder = None
        self.active_decoders = []
        self.total_frames = 0
        self.fps = 36.0
        self.base_time = datetime.now()
        self.is_current_sentry = False

        self.active_clip_list = []
        self.current_active_clip_idx = -1
        self.clip_frame_offsets = []

        self.is_exporting = False
        self.last_exported_video_path = None
        self.last_grid_image = None
        self.last_valid_preview_frames = {}

        self.detected_event_blocks = []
        self.scanner = None
        self.motion_cache = {}

        self.start_point = None
        self.end_point = None

        self.play_start_wall_time = None
        self.play_start_frame = 0

        self.init_ui()
        self.setup_shortcuts()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.play_next_frame_smooth)

        self.checker = UpdateCheckWorker(self)
        self.checker.update_available.connect(self.on_update_found)
        self.checker.start()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        root_layout = QHBoxLayout(main_widget)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        root_layout.addWidget(splitter)

        # =========================================================================
        # 1. 좌측 사이드바 패널 (폴더 선택 + 탐색기 + 오버레이 옵션 + 내보내기 설정)
        # =========================================================================
        left_panel = QWidget()
        left_panel.setMinimumWidth(370)
        left_panel.setMaximumWidth(460)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)

        self.btn_load = QPushButton("📁 TeslaCam 폴더 지정")
        self.btn_load.setFixedHeight(34)
        self.btn_load.setStyleSheet("background-color: #0078D7; color: #FFFFFF; font-weight: bold; font-size: 13px;")
        self.btn_load.clicked.connect(self.load_directory_dialog)
        left_layout.addWidget(self.btn_load)

        self.tree_explorer = QTreeWidget()
        self.tree_explorer.setHeaderLabels(["클립 탐색기", "유형 / 채널"])
        self.tree_explorer.setColumnWidth(0, 240)
        self.tree_explorer.setColumnWidth(1, 110)
        self.tree_explorer.header().setStretchLastSection(True)
        self.tree_explorer.itemClicked.connect(self.on_tree_item_clicked)
        left_layout.addWidget(self.tree_explorer, stretch=1)

        # 1) 오버레이 표시 옵션 (컴팩트 2열 그리드)
        opt_group = QGroupBox("오버레이 요소")
        opt_layout = QGridLayout(opt_group)
        opt_layout.setContentsMargins(8, 6, 8, 6)
        opt_layout.setHorizontalSpacing(8)
        opt_layout.setVerticalSpacing(3)

        self.chks = {
            "timestamp": QCheckBox("실제 시간 (Timestamp)"),
            "speed": QCheckBox("실제 속도 (km/h)"),
            "fsd": QCheckBox("FSD / 자율주행 모드"),
            "turn_signal": QCheckBox("방향지시등 (점멸 깜빡이)"),
            "steering": QCheckBox("스티어링 핸들 각도"),
            "pedal": QCheckBox("가속 / 브레이크 페달"),
            "pillar_pip": QCheckBox("필러 카메라 (PIP)"),
            "map": QCheckBox("GPS 미니맵 (OSM)"),
        }

        chk_positions = [
            ("timestamp", 0, 0), ("steering", 0, 1),
            ("speed", 1, 0),     ("pedal", 1, 1),
            ("fsd", 2, 0),       ("pillar_pip", 2, 1),
            ("turn_signal", 3, 0), ("map", 3, 1),
        ]

        for k, r, c in chk_positions:
            chk = self.chks[k]
            chk.setChecked(True)
            chk.stateChanged.connect(self.sync_preview)
            opt_layout.addWidget(chk, r, c)

        left_layout.addWidget(opt_group)

        # 2) 내보내기 설정 (사이드바 하단 정렬)
        exp_group = QGroupBox("내보내기 설정")
        exp_layout = QGridLayout(exp_group)
        exp_layout.setContentsMargins(8, 6, 8, 6)
        exp_layout.setHorizontalSpacing(6)
        exp_layout.setVerticalSpacing(4)

        exp_layout.addWidget(QLabel("출력 해상도:"), 0, 0)
        self.combo_res = QComboBox()
        for res_name in RESOLUTIONS.keys():
            self.combo_res.addItem(res_name)
        self.combo_res.currentIndexChanged.connect(self.update_estimated_size)
        exp_layout.addWidget(self.combo_res, 0, 1, 1, 3)

        exp_layout.addWidget(QLabel("화면 레이아웃:"), 1, 0)
        layout_btn_box = QHBoxLayout()
        layout_btn_box.setSpacing(3)

        self.btn_layout_1to3 = QPushButton("🔲 1:3")
        self.btn_layout_1to3.setToolTip("기본 (전방 대형 + 3개 측후방 세로배치)")
        self.btn_layout_1to3.setFixedHeight(26)
        self.btn_layout_1to3.clicked.connect(lambda: self.set_layout_mode("기본 (1:3 세로배치)"))

        self.btn_layout_2x2 = QPushButton("⊞ 2x2")
        self.btn_layout_2x2.setToolTip("2x2 분할 (전후/좌우 4분할)")
        self.btn_layout_2x2.setFixedHeight(26)
        self.btn_layout_2x2.clicked.connect(lambda: self.set_layout_mode("2x2 분할 (전후/좌우)"))

        self.btn_layout_front = QPushButton("⏹ 전면")
        self.btn_layout_front.setToolTip("전면 단독 (전방 풀스크린)")
        self.btn_layout_front.setFixedHeight(26)
        self.btn_layout_front.clicked.connect(lambda: self.set_layout_mode("전면 단독 (전방 풀스크린)"))

        for b in [self.btn_layout_1to3, self.btn_layout_2x2, self.btn_layout_front]:
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            layout_btn_box.addWidget(b)

        exp_layout.addLayout(layout_btn_box, 1, 1, 1, 3)

        exp_layout.addWidget(QLabel("저장 배속:"), 2, 0)
        self.combo_export_speed = QComboBox()
        self.combo_export_speed.addItems(["0.5x (슬로우)", "1.0x (표준)", "1.5x (빠르게)", "2.0x (2배속)", "4.0x (4배속)", "5.0x (5배속)"])
        self.combo_export_speed.setCurrentIndex(1)  # 기본값: 1.0x (표준)
        self.combo_export_speed.currentIndexChanged.connect(self.on_export_speed_changed)
        exp_layout.addWidget(self.combo_export_speed, 2, 1)


        exp_layout.addWidget(QLabel("출력 FPS:"), 2, 2)
        self.combo_fps = QComboBox()
        self.combo_fps.currentIndexChanged.connect(self.update_estimated_size)
        exp_layout.addWidget(self.combo_fps, 2, 3)

        self.lbl_est_size = QLabel("예상 크기: 약 0 MB")
        self.lbl_est_size.setStyleSheet("color: #00E6FF; font-weight: bold; font-size: 13px;")
        exp_layout.addWidget(self.lbl_est_size, 3, 0, 1, 4)

        self.btn_export = QPushButton("선택 구간 내보내기 (MP4)")
        self.btn_export.setFixedHeight(34)
        self.btn_export.setStyleSheet("background-color: #0078D7; color: #FFFFFF; font-weight: bold; font-size: 13px;")
        self.btn_export.clicked.connect(self.on_click_export_button)
        exp_layout.addWidget(self.btn_export, 4, 0, 1, 4)

        self.pbar = QProgressBar()
        self.pbar.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pbar.setFixedHeight(12)
        exp_layout.addWidget(self.pbar, 5, 0, 1, 4)

        self.btn_qr_share = QPushButton("📱 스마트폰 무선 전송 (QR)")
        self.btn_qr_share.setFixedHeight(28)
        self.btn_qr_share.setEnabled(False)
        self.btn_qr_share.clicked.connect(self.on_click_qr_share)
        exp_layout.addWidget(self.btn_qr_share, 6, 0, 1, 4)

        left_layout.addWidget(exp_group)
        splitter.addWidget(left_panel)

        # =========================================================================
        # 2. 우측 메인 패널 (대형 비디오 뷰 + 날렵한 1줄 타임라인 컨트롤 바)
        # =========================================================================
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        # 비디오 프리뷰 화면 (화면의 대부분을 꽉 채우도록 Expanding 설정)
        self.preview_lbl = QLabel()
        self.preview_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_lbl.setStyleSheet("background-color: #0b0d10; border: 1px solid #1e222a; border-radius: 4px;")
        self.preview_lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.preview_lbl.setMinimumSize(640, 360)
        right_layout.addWidget(self.preview_lbl, stretch=10)

        # 하단 타임라인 컨트롤 그룹
        timeline_group = QGroupBox("타임라인 컨트롤")
        bottom_ctrl_layout = QVBoxLayout(timeline_group)
        bottom_ctrl_layout.setContentsMargins(8, 6, 8, 6)
        bottom_ctrl_layout.setSpacing(4)

        # 타임라인 슬라이더
        self.slider = HighlightSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 0)
        self.slider.sliderMoved.connect(self.on_slider_manual_seek)
        self.slider.valueChanged.connect(self.on_slider_manual_seek)
        bottom_ctrl_layout.addWidget(self.slider)

        # 슬라이더 상단/하단 정보 바 (시간, 모션 감지 상태, 선택 구간)
        info_bar = QHBoxLayout()
        info_bar.setContentsMargins(2, 0, 2, 0)
        
        self.lbl_curr_time = QLabel("00:00:00 / 00:00:00")
        self.lbl_curr_time.setStyleSheet("color: #00E6FF; font-weight: bold; font-size: 13px;")
        info_bar.addWidget(self.lbl_curr_time)

        info_bar.addSpacing(16)

        self.lbl_motion_status = QLabel("모션 분석: 대기 중")
        self.lbl_motion_status.setStyleSheet("color: #00E6FF; font-size: 12px; font-weight: bold;")
        info_bar.addWidget(self.lbl_motion_status)

        info_bar.addStretch()

        self.lbl_range_info = QLabel("선택 구간: 미지정 (전체)")
        self.lbl_range_info.setStyleSheet("color: #FFA726; font-size: 13px; font-weight: bold;")
        info_bar.addWidget(self.lbl_range_info)
        bottom_ctrl_layout.addLayout(info_bar)

        # 재생 & 조작 통합 툴바 (1줄에 직관적으로 정렬)
        playback_bar = QHBoxLayout()
        playback_bar.setSpacing(4)

        self.btn_prev = QPushButton("◀ 이전")
        self.btn_prev.setToolTip("이전 클립 (단축키: [)")
        self.btn_prev.setFixedHeight(28)
        self.btn_prev.setStyleSheet("padding: 2px 6px; font-weight: bold;")
        self.btn_prev.clicked.connect(self.play_prev_clip)
        playback_bar.addWidget(self.btn_prev)

        self.btn_play = QPushButton("▶ 재생 (Space)")
        self.btn_play.setToolTip("재생 / 일시정지 (단축키: Space)")
        self.btn_play.setFixedHeight(28)
        self.btn_play.setStyleSheet("font-weight: bold; background-color: #0078D7; color: #FFFFFF; min-width: 95px; padding: 2px 8px;")
        self.btn_play.clicked.connect(self.toggle_play)
        playback_bar.addWidget(self.btn_play)

        self.btn_next = QPushButton("다음 ▶")
        self.btn_next.setToolTip("다음 클립 (단축키: ])")
        self.btn_next.setFixedHeight(28)
        self.btn_next.setStyleSheet("padding: 2px 6px; font-weight: bold;")
        self.btn_next.clicked.connect(self.play_next_clip)
        playback_bar.addWidget(self.btn_next)

        playback_bar.addSpacing(4)

        self.btn_in = QPushButton("[ 시작점")
        self.btn_in.setToolTip("구간 시작점 설정 (단축키: I)")
        self.btn_in.setFixedHeight(28)
        self.btn_in.setStyleSheet("padding: 2px 6px;")
        self.btn_in.clicked.connect(self.set_in_point)
        playback_bar.addWidget(self.btn_in)

        self.btn_out = QPushButton("] 끝점")
        self.btn_out.setToolTip("구간 종료점 설정 (단축키: O)")
        self.btn_out.setFixedHeight(28)
        self.btn_out.setStyleSheet("padding: 2px 6px;")
        self.btn_out.clicked.connect(self.set_out_point)
        playback_bar.addWidget(self.btn_out)

        self.btn_reset_range = QPushButton("🔄 초기화")
        self.btn_reset_range.setToolTip("구간 선택 초기화")
        self.btn_reset_range.setFixedHeight(28)
        self.btn_reset_range.setStyleSheet("padding: 2px 6px;")
        self.btn_reset_range.clicked.connect(self.reset_range_points)
        playback_bar.addWidget(self.btn_reset_range)

        self.btn_hotkeys = QPushButton("⌨ 단축키")
        self.btn_hotkeys.setToolTip("키보드 단축키 안내 (F1 또는 ?)")
        self.btn_hotkeys.setFixedHeight(28)
        self.btn_hotkeys.setStyleSheet("padding: 2px 6px;")
        self.btn_hotkeys.clicked.connect(self.show_hotkey_guide)
        playback_bar.addWidget(self.btn_hotkeys)

        playback_bar.addSpacing(6)

        # 밝기 및 대비 슬라이더
        lbl_b = QLabel("☀️ 밝기:")
        lbl_b.setStyleSheet("font-size: 12px; color: #E0E0E0;")
        playback_bar.addWidget(lbl_b)
        self.slider_brightness = QSlider(Qt.Orientation.Horizontal)
        self.slider_brightness.setRange(-100, 100)
        self.slider_brightness.setValue(0)
        self.slider_brightness.setFixedWidth(65)
        self.slider_brightness.valueChanged.connect(self.sync_preview)
        playback_bar.addWidget(self.slider_brightness)

        lbl_c = QLabel("🌓 대비:")
        lbl_c.setStyleSheet("font-size: 12px; color: #E0E0E0;")
        playback_bar.addWidget(lbl_c)
        self.slider_contrast = QSlider(Qt.Orientation.Horizontal)
        self.slider_contrast.setRange(50, 200)
        self.slider_contrast.setValue(100)
        self.slider_contrast.setFixedWidth(65)
        self.slider_contrast.valueChanged.connect(self.sync_preview)
        playback_bar.addWidget(self.slider_contrast)

        playback_bar.addSpacing(4)

        # 모션 감지 및 감도 설정
        self.chk_motion = QCheckBox("모션 감지")
        self.chk_motion.setChecked(True)
        self.chk_motion.stateChanged.connect(self.on_motion_chk_changed)
        playback_bar.addWidget(self.chk_motion)

        self.lbl_sensitivity_tag = QLabel("감도:")
        self.lbl_sensitivity_tag.setStyleSheet("font-size: 12px; color: #A0A6B5;")
        playback_bar.addWidget(self.lbl_sensitivity_tag)

        self.combo_sensitivity = QComboBox()
        for s_name in SENSITIVITY_LEVELS.keys():
            self.combo_sensitivity.addItem(s_name)
        self.combo_sensitivity.setCurrentText("보통")
        self.combo_sensitivity.setFixedWidth(60)
        self.combo_sensitivity.currentIndexChanged.connect(self.on_sensitivity_changed)
        playback_bar.addWidget(self.combo_sensitivity)

        self.btn_prev_event = QPushButton("⚡ 이전")
        self.btn_prev_event.setFixedHeight(26)
        self.btn_prev_event.setVisible(False)
        self.btn_prev_event.clicked.connect(self.jump_to_prev_event)
        playback_bar.addWidget(self.btn_prev_event)

        self.btn_next_event = QPushButton("⚡ 다음")
        self.btn_next_event.setFixedHeight(26)
        self.btn_next_event.setVisible(False)
        self.btn_next_event.clicked.connect(self.jump_to_next_event)
        playback_bar.addWidget(self.btn_next_event)

        playback_bar.addSpacing(8)
        self.chk_auto_next = QCheckBox("다음 클립 자동재생")
        self.chk_auto_next.setChecked(False)  # 기본값: OFF
        self.chk_auto_next.setToolTip("재생이 끝나면 자동으로 다음 클립으로 이동합니다")
        playback_bar.addWidget(self.chk_auto_next)

        playback_bar.addSpacing(8)
        lbl_spd = QLabel("▶ 배속:")
        lbl_spd.setStyleSheet("font-size: 12px; color: #E0E0E0;")
        playback_bar.addWidget(lbl_spd)
        self.slider_preview_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_preview_speed.setRange(1, 10)  # 0.5x ~ 5.0x
        self.slider_preview_speed.setValue(2)      # 기본 1.0x
        self.slider_preview_speed.setFixedWidth(70)
        self.slider_preview_speed.setToolTip("미리보기 재생 배속 (0.5x ~ 5.0x)")
        self.slider_preview_speed.valueChanged.connect(self.on_preview_speed_slider_changed)
        playback_bar.addWidget(self.slider_preview_speed)
        self.lbl_preview_speed_val = QLabel("1.0x")
        self.lbl_preview_speed_val.setStyleSheet("font-size: 12px; color: #FFD700; font-weight: bold; min-width: 32px;")
        playback_bar.addWidget(self.lbl_preview_speed_val)

        playback_bar.addStretch()
        bottom_ctrl_layout.addLayout(playback_bar)

        right_layout.addWidget(timeline_group)

        # 우측 하단 푸터 (서명 및 버전 정보)
        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(0, 0, 4, 0)
        footer_layout.addStretch()
        self.lbl_footer = QLabel(f"♥ Companion Turret  ·  v{APP_VERSION}")
        self.lbl_footer.setStyleSheet("color: #454D5A; font-size: 11px; font-weight: normal;")
        footer_layout.addWidget(self.lbl_footer)
        right_layout.addLayout(footer_layout)

        splitter.addWidget(right_panel)

        # 좌측 380px, 우측 비디오 뷰 최대화
        splitter.setSizes([380, 1220])
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        self.current_layout_mode = "기본 (1:3 세로배치)"
        self.update_layout_buttons_style()

        # 모든 버튼들이 포커스를 뺏어 키보드 탐색을 방해하지 않도록 NoFocus 설정
        for btn in [self.btn_load, self.btn_prev, self.btn_play, self.btn_next, 
                    self.btn_in, self.btn_out, self.btn_reset_range, self.btn_hotkeys,
                    self.btn_prev_event, self.btn_next_event, self.btn_export, self.btn_qr_share]:
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.apply_dark_theme()

    def setup_shortcuts(self):
        """ 전역 단축키 등록 (어떤 위젯에 포커스가 있든 무조건 최우선 실행) """
        # Space: 재생/일시정지
        QShortcut(QKeySequence(Qt.Key.Key_Space), self, self.toggle_play)
        
        # 좌/우 방향키: 1초 프레임 이동 (일시정지 중이든 재생 중이든 즉시 탐색)
        QShortcut(QKeySequence(Qt.Key.Key_Left), self, lambda: self.seek_relative(-1.0))
        QShortcut(QKeySequence(Qt.Key.Key_Right), self, lambda: self.seek_relative(1.0))
        
        # Shift + 좌/우: 5초 고속 탐색
        QShortcut(QKeySequence("Shift+Left"), self, lambda: self.seek_relative(-5.0))
        QShortcut(QKeySequence("Shift+Right"), self, lambda: self.seek_relative(5.0))
        
        # I / O: 시작점 / 종료점 설정
        QShortcut(QKeySequence("I"), self, self.set_in_point)
        QShortcut(QKeySequence("O"), self, self.set_out_point)
        
        # [ / ]: 이전 / 다음 클립
        QShortcut(QKeySequence("["), self, self.play_prev_clip)
        QShortcut(QKeySequence("]"), self, self.play_next_clip)
        
        # Home: 첫 프레임으로 이동
        QShortcut(QKeySequence(Qt.Key.Key_Home), self, lambda: self.seek_absolute(0))

        # F1 또는 ?: 단축키 안내 팝업 열기
        QShortcut(QKeySequence(Qt.Key.Key_F1), self, self.show_hotkey_guide)
        QShortcut(QKeySequence("?"), self, self.show_hotkey_guide)

    def show_hotkey_guide(self):
        """ 단축키 안내 다이얼로그 표시 """
        dlg = HotkeyGuideDialog(self)
        dlg.exec()

    def seek_relative(self, delta_sec):
        """ 상대 시간(초) 단위로 프레임 즉시 탐색 및 렌더링 """
        if not self.active_clip_list or self.total_frames <= 0 or self.is_exporting:
            return

        step_frames = int(round(self.fps * delta_sec))
        if step_frames == 0:
            step_frames = 1 if delta_sec > 0 else -1

        new_idx = max(0, min(self.total_frames - 1, self.slider.value() + step_frames))
        self.slider.blockSignals(True)
        self.slider.setValue(new_idx)
        self.slider.blockSignals(False)

        if self.timer.isActive():
            self.play_start_wall_time = time.perf_counter()
            self.play_start_frame = new_idx

        self.update_current_time_display(new_idx)
        frames = self.read_frames_with_cache(new_idx, is_seeking=True)
        self.render_and_display(frames, new_idx)

    def seek_absolute(self, target_frame):
        """ 절대 프레임 번호로 즉시 이동 """
        if not self.active_clip_list or self.total_frames <= 0 or self.is_exporting:
            return
        new_idx = max(0, min(self.total_frames - 1, target_frame))
        self.slider.blockSignals(True)
        self.slider.setValue(new_idx)
        self.slider.blockSignals(False)

        if self.timer.isActive():
            self.play_start_wall_time = time.perf_counter()
            self.play_start_frame = new_idx

        self.update_current_time_display(new_idx)
        frames = self.read_frames_with_cache(new_idx, is_seeking=True)
        self.render_and_display(frames, new_idx)

    def apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #121418; }
            QWidget { color: #E0E0E0; font-family: 'Segoe UI', 'Malgun Gothic', sans-serif; font-size: 13px; }
            QGroupBox { border: 1px solid #232731; border-radius: 5px; margin-top: 5px; font-weight: bold; color: #00E6FF; padding: 5px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; font-size: 13px; }
            QPushButton { background-color: #1A1D24; border: 1px solid #2A2E39; border-radius: 4px; padding: 4px 10px; font-weight: bold; min-height: 24px; font-size: 13px; }
            QPushButton:hover { background-color: #252A36; border-color: #0078D7; }
            QPushButton:pressed { background-color: #0E1015; }
            QPushButton:disabled { background-color: #14161A; color: #555555; border-color: #1E2128; }
            QTreeWidget { background-color: #161920; border: 1px solid #232731; border-radius: 4px; color: #FFFFFF; font-size: 13px; }
            QTreeWidget::item:selected { background-color: #005A9E; color: #FFFFFF; }
            QTreeWidget::item:hover { background-color: #1F2430; }
            QComboBox { background-color: #1A1D24; border: 1px solid #2A2E39; border-radius: 3px; padding: 3px 8px; color: #FFFFFF; font-size: 13px; }
            QComboBox QAbstractItemView { background-color: #1A1D24; selection-background-color: #0078D7; color: #FFFFFF; font-size: 13px; }
            QCheckBox { spacing: 6px; color: #D0D4DC; font-size: 13px; }
            QProgressBar { border: 1px solid #2A2E39; border-radius: 3px; text-align: center; background-color: #14161B; color: #FFFFFF; font-weight: bold; font-size: 11px; }
            QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0078D7, stop:1 #00E6FF); border-radius: 2px; }
            QSlider::groove:horizontal { border: 1px solid #2A2E39; height: 4px; background: #1C2028; border-radius: 2px; }
            QSlider::sub-page:horizontal { background: #0078D7; border-radius: 2px; }
            QSlider::handle:horizontal { background: #00E6FF; border: 1px solid #FFFFFF; width: 14px; margin-top: -5px; margin-bottom: -5px; border-radius: 7px; }
            QSlider::handle:horizontal:hover { background: #FFFFFF; border-color: #00E6FF; }
        """)

    def load_directory_dialog(self):
        default_dir = get_app_dir()
        folder = QFileDialog.getExistingDirectory(self, "TeslaCam 폴더 선택", default_dir)
        if not folder:
            return

        self.motion_cache.clear()
        self.scan_and_populate_tree(folder)

    def scan_and_populate_tree(self, root_path):
        self.tree_explorer.clear()
        self.reset_range_points()
        self.slider.set_event_blocks([])
        
        cam_suffixes = {
            "front": ["-front.mp4"],
            "back": ["-back.mp4"],
            "left_repeater": ["-left_repeater.mp4"],
            "right_repeater": ["-right_repeater.mp4"],
            "left_pillar": ["-left_pillar.mp4", "-front_left.mp4", "-left_b_pillar.mp4"],
            "right_pillar": ["-right_pillar.mp4", "-front_right.mp4", "-right_b_pillar.mp4"]
        }

        tree_data = {}
        for dirpath, _, filenames in os.walk(root_path):
            front_files = [f for f in filenames if f.endswith("-front.mp4")]
            if not front_files:
                continue

            front_files.sort()
            rel = os.path.relpath(dirpath, root_path)
            parts = rel.split(os.sep) if rel != "." else [os.path.basename(dirpath)]

            category = parts[0] if len(parts) > 1 else "TeslaCam"
            event_name = parts[-1]

            if category not in tree_data:
                tree_data[category] = {}
            if event_name not in tree_data[category]:
                tree_data[category][event_name] = []

            dir_lower = dirpath.lower()
            is_sentry_folder = ("sentryclips" in dir_lower or "sentry" in dir_lower) and not ("savedclips" in dir_lower)
            is_recent_folder = "recentclips" in dir_lower or "recent" in dir_lower

            for ff in front_files:
                prefix = ff[:-10]
                front_path = os.path.join(dirpath, ff)
                cams = {"front": front_path}

                for c_key, s_list in cam_suffixes.items():
                    if c_key == "front":
                        continue
                    for s in s_list:
                        candidate = os.path.join(dirpath, prefix + s)
                        if os.path.exists(candidate):
                            cams[c_key] = candidate
                            break

                ts_display = prefix
                m = re.match(r'(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})', prefix)
                if m:
                    ts_display = f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}"

                tree_data[category][event_name].append({
                    "prefix": prefix,
                    "display": ts_display,
                    "cams": cams,
                    "dirpath": dirpath,
                    "is_sentry": is_sentry_folder,
                    "is_recent": is_recent_folder
                })

        if not tree_data:
            QMessageBox.warning(self, "경고", "선택한 폴더 내에서 테슬라 영상 파일(*-front.mp4)을 찾지 못했습니다.")
            return

        brush_white = QBrush(QColor("#FFFFFF"))
        brush_green = QBrush(QColor("#00FF66"))
        brush_red = QBrush(QColor("#FF3333"))

        for cat_name, events in tree_data.items():
            cat_item = QTreeWidgetItem(self.tree_explorer, [f"📁 {cat_name}", f"{len(events)}개 그룹"])
            cat_item.setExpanded(True)

            for ev_name, clips in events.items():
                is_sentry_group = any(c["is_sentry"] for c in clips)
                is_recent_group = any(c["is_recent"] for c in clips)

                if is_sentry_group and not is_recent_group:
                    tag = "[센트리]"
                    ev_item = QTreeWidgetItem(cat_item, [f"📂 {ev_name} (주차 세션)", f"{tag} {len(clips)}개 영상 통합"])
                    ev_item.setForeground(1, brush_red)
                    ev_item.setData(0, Qt.ItemDataRole.UserRole, {
                        "is_group": True,
                        "is_sentry": True,
                        "clip_list": clips
                    })
                else:
                    ev_item = QTreeWidgetItem(cat_item, [f"📂 {ev_name}", f"{len(clips)}개 클립"])
                    ev_item.setExpanded(True)

                    for clip in clips:
                        front_path = clip["cams"]["front"]
                        ch_count = len(clip["cams"])
                        
                        is_park = RealTeslaSEIDecoder.quick_check_is_parking(front_path)
                        tag = "[주차]" if is_park else "[주행]"
                        
                        clip_item = QTreeWidgetItem(ev_item, [f"🎬 {clip['display']}", f"{tag} {ch_count}채널"])
                        clip_item.setForeground(1, brush_green if is_park else brush_white)
                        clip_item.setData(0, Qt.ItemDataRole.UserRole, {
                            "is_group": False,
                            "is_sentry": is_park,
                            "clip_list": [clip]
                        })

    def get_current_clip_key(self):
        if not self.active_clip_list:
            return None
        return f"{self.active_clip_list[0]['prefix']}_{len(self.active_clip_list)}"

    def on_tree_item_clicked(self, item, column):
        item_data = item.data(0, Qt.ItemDataRole.UserRole)
        if not item_data or "clip_list" not in item_data:
            return

        self.stop_motion_scanner()

        if not getattr(self, 'was_playing_before_load', False):
            self.was_playing_before_load = self.timer.isActive()
        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText("▶ 재생 (Space)")


        self.current_item = item
        self.active_clip_list = item_data["clip_list"]
        if not self.active_clip_list:
            return

        if hasattr(self, 'loader_worker') and self.loader_worker and self.loader_worker.isRunning():
            self.loader_worker.stop()
            self.loader_worker.wait()

        self.is_load_canceled = False
        self.progress_dialog = QProgressDialog("비디오 정보 로딩 중...", "취소", 0, 100, self)
        self.progress_dialog.setWindowTitle("로딩 중")
        self.progress_dialog.setWindowModality(Qt.WindowModality.WindowModal)
        self.progress_dialog.setAutoClose(False)
        self.progress_dialog.setAutoReset(False)
        self.progress_dialog.setMinimumDuration(200)
        self.progress_dialog.setValue(0)

        def on_user_cancel():
            self.is_load_canceled = True
            if hasattr(self, 'loader_worker') and self.loader_worker:
                self.loader_worker.stop()

        self.progress_dialog.canceled.connect(on_user_cancel)

        self.loader_worker = ClipLoaderWorker(self.active_clip_list, self)
        self.loader_worker.progress.connect(self._on_clips_load_progress)
        self.loader_worker.finished.connect(lambda decs, offs, tot, fps, itm_d=item_data: self._on_clips_loaded_callback(decs, offs, tot, fps, itm_d))
        self.loader_worker.start()

    def _on_clips_load_progress(self, val, msg):
        if hasattr(self, 'progress_dialog') and self.progress_dialog and not getattr(self, 'is_load_canceled', False):
            self.progress_dialog.setValue(val)
            self.progress_dialog.setLabelText(msg)

    def _on_clips_loaded_callback(self, decoders, offsets, total_f, detected_fps, item_data):
        try:
            if hasattr(self, 'progress_dialog') and self.progress_dialog:
                try:
                    self.progress_dialog.canceled.disconnect()
                except Exception:
                    pass
                self.progress_dialog.reset()
                self.progress_dialog.hide()

            if getattr(self, 'is_load_canceled', False):
                return

            self.active_decoders = decoders
            self.clip_frame_offsets = offsets
            self.total_frames = total_f
            self.is_current_sentry = item_data.get("is_sentry", False)

            # 주차 영상에 대해서는 전면 프레임 모드 비활성화 및 이전 영상이 전면모드였을 경우 1:3으로 전환
            if self.is_current_sentry:
                self.btn_layout_front.setEnabled(False)
                self.btn_layout_front.setToolTip("주차/센트리 영상은 다채널 확인을 위해 전면 단독 모드를 지원하지 않습니다.")
                if self.current_layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]:
                    self.current_layout_mode = "기본 (1:3 세로배치)"
                    self.update_layout_buttons_style()
            else:
                self.btn_layout_front.setEnabled(True)
                self.btn_layout_front.setToolTip("전면 단독 (전방 풀스크린)")

            match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', self.active_clip_list[0]["prefix"])
            if match:
                self.base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S")

            has_pillars = ('left_pillar' in self.active_clip_list[0]["cams"]) or ('right_pillar' in self.active_clip_list[0]["cams"])
            self.chks["pillar_pip"].blockSignals(True)
            if self.current_layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]:
                self.chks["pillar_pip"].setChecked(False)
                self.chks["pillar_pip"].setEnabled(False)
            elif has_pillars:
                self.chks["pillar_pip"].setEnabled(True)
                self.chks["pillar_pip"].setChecked(True)
            else:
                self.chks["pillar_pip"].setChecked(False)
                self.chks["pillar_pip"].setEnabled(False)
            self.chks["pillar_pip"].blockSignals(False)

            self.last_valid_preview_frames.clear()
            self.current_active_clip_idx = -1

            if detected_fps is not None:
                self.fps = detected_fps
            else:
                self.fps = 24.0 if has_pillars else 36.0

            self.play_start_wall_time = None
            self.play_start_frame = 0

            self.slider.blockSignals(True)
            self.slider.setRange(0, max(0, self.total_frames - 1))
            self.slider.setValue(0)
            self.slider.set_event_blocks([])
            self.slider.blockSignals(False)
            
            self.update_fps_options()
            self.update_time_ticks()
            self.update_current_time_display(0)
            self.update_range_highlight()
            
            frames = self.read_frames_with_cache(0, is_seeking=True)
            self.render_and_display(frames, 0)

            if self.is_current_sentry:
                self.chk_motion.setVisible(True)
                self.lbl_sensitivity_tag.setVisible(True)
                self.combo_sensitivity.setVisible(True)
                
                if self.chk_motion.isChecked():
                    self.btn_prev_event.setVisible(True)
                    self.btn_next_event.setVisible(True)
                    
                    clip_key = self.get_current_clip_key()
                    if clip_key in self.motion_cache:
                        cached_blocks = self.motion_cache[clip_key]
                        self.detected_event_blocks = cached_blocks
                        self.slider.set_event_blocks(cached_blocks)
                        self.lbl_motion_status.setText(f"모션 분석 완료: {len(cached_blocks)}개 구간 감지")
                    else:
                        self.start_light_motion_scanner(self.active_clip_list)
                else:
                    self.btn_prev_event.setVisible(False)
                    self.btn_next_event.setVisible(False)
                    self.lbl_motion_status.setText("모션 분석: 대기 중")
            else:
                self.chk_motion.setVisible(False)
                self.lbl_sensitivity_tag.setVisible(False)
                self.combo_sensitivity.setVisible(False)
                self.btn_prev_event.setVisible(False)
                self.btn_next_event.setVisible(False)
                self.detected_event_blocks = []
                self.slider.set_event_blocks([])
                self.lbl_motion_status.setText("모션 분석: 비활성화")

            if getattr(self, 'was_playing_before_load', False):
                if not self.timer.isActive():
                    self.toggle_play()
                self.was_playing_before_load = False

        except Exception as e:
            import traceback
            print("Exception in _on_clips_loaded_callback:")
            traceback.print_exc()

    def switch_to_clip_index(self, clip_idx):
        if self.current_active_clip_idx == clip_idx:
            return

        for cap in self.caps.values():
            cap.release()

        clip_info = self.active_clip_list[clip_idx]
        self.cams = clip_info["cams"]
        self.caps = {k: cv2.VideoCapture(v) for k, v in self.cams.items() if os.path.exists(v)}
        self.decoder = self.active_decoders[clip_idx] if clip_idx < len(self.active_decoders) else None
        self.current_active_clip_idx = clip_idx

        # 현재 클립의 base_time으로 업데이트 (HUD 시간 동기화)
        match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', clip_info.get("prefix", ""))
        if match:
            self.base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S")

    def on_motion_chk_changed(self, state):
        if self.is_current_sentry:
            if state == Qt.CheckState.Checked.value or state == 2:
                self.btn_prev_event.setVisible(True)
                self.btn_next_event.setVisible(True)
                
                clip_key = self.get_current_clip_key()
                if clip_key in self.motion_cache:
                    cached_blocks = self.motion_cache[clip_key]
                    self.detected_event_blocks = cached_blocks
                    self.slider.set_event_blocks(cached_blocks)
                    self.lbl_motion_status.setText(f"모션 분석 완료: {len(cached_blocks)}개 구간 감지")
                else:
                    self.start_light_motion_scanner(self.active_clip_list)
            else:
                self.btn_prev_event.setVisible(False)
                self.btn_next_event.setVisible(False)
                self.detected_event_blocks = []
                self.slider.set_event_blocks([])
                self.stop_motion_scanner()
                self.lbl_motion_status.setText("모션 분석: 비활성화")

    def on_sensitivity_changed(self):
        if self.is_current_sentry and self.chk_motion.isChecked():
            clip_key = self.get_current_clip_key()
            if clip_key and clip_key in self.motion_cache:
                del self.motion_cache[clip_key]
            self.start_light_motion_scanner(self.active_clip_list)

    def start_light_motion_scanner(self, clip_list):
        if not self.is_current_sentry:
            return

        self.stop_motion_scanner()

        self.lbl_motion_status.setText("모션 분석 중...")
        self.detected_event_blocks = []

        sensitivity_preset = self.combo_sensitivity.currentText()
        self.scanner = LightMotionScanner(clip_list, fps=self.fps, sensitivity_preset=sensitivity_preset, target_scan_fps=6.0)
        self.scanner.events_found.connect(self.on_motion_events_found)
        self.scanner.start()

    def on_motion_events_found(self, event_blocks):
        if not self.is_current_sentry:
            return

        self.detected_event_blocks = event_blocks
        self.slider.set_event_blocks(event_blocks)

        clip_key = self.get_current_clip_key()
        if clip_key:
            self.motion_cache[clip_key] = event_blocks

        self.lbl_motion_status.setText(f"모션 분석 완료: {len(event_blocks)}개 구간 감지")

    def stop_motion_scanner(self):
        if self.scanner and self.scanner.isRunning():
            self.scanner.stop()
            self.scanner.wait()
            self.scanner = None

    def jump_to_prev_event(self):
        if not self.detected_event_blocks:
            return
        curr_f = self.slider.value()
        prev_f = None
        for start_f, _ in reversed(self.detected_event_blocks):
            if start_f < curr_f - 10:
                prev_f = start_f
                break
        if prev_f is None:
            prev_f = self.detected_event_blocks[-1][0]
        self.slider.setValue(prev_f)

    def jump_to_next_event(self):
        if not self.detected_event_blocks:
            return
        curr_f = self.slider.value()
        next_f = None
        for start_f, _ in self.detected_event_blocks:
            if start_f > curr_f + 10:
                next_f = start_f
                break
        if next_f is None:
            next_f = self.detected_event_blocks[0][0]
        self.slider.setValue(next_f)

    def get_sibling_clip_item(self, direction=1):
        curr_item = self.tree_explorer.currentItem()
        if not curr_item:
            return None

        parent_item = curr_item.parent()
        if not parent_item:
            return None

        idx = parent_item.indexOfChild(curr_item)
        target_idx = idx + direction

        if 0 <= target_idx < parent_item.childCount():
            return parent_item.child(target_idx)
        return None

    def play_prev_clip(self):
        target_item = self.get_sibling_clip_item(-1)
        if target_item:
            self.was_playing_before_load = self.timer.isActive()
            self.tree_explorer.setCurrentItem(target_item)
            self.on_tree_item_clicked(target_item, 0)

    def play_next_clip(self, auto=False):
        """다음 클립으로 이동. auto=True 이면 자동재생 체크 상태를 유지하며 로드 완료 후 재생 시작."""
        target_item = self.get_sibling_clip_item(1)
        if target_item:
            auto_next_was_checked = hasattr(self, 'chk_auto_next') and self.chk_auto_next.isChecked()
            self.was_playing_before_load = auto or self.timer.isActive()
            self.tree_explorer.setCurrentItem(target_item)
            self.on_tree_item_clicked(target_item, 0)
            if hasattr(self, 'chk_auto_next'):
                self.chk_auto_next.setChecked(auto_next_was_checked)
        else:
            if self.timer.isActive():
                self.toggle_play()

    def on_slider_manual_seek(self):
        if not self.active_clip_list or self.is_exporting:
            return
        idx = self.slider.value()
        if self.timer.isActive():
            self.play_start_wall_time = time.perf_counter()
            self.play_start_frame = idx
        self.update_current_time_display(idx)
        frames = self.read_frames_with_cache(idx, is_seeking=True)
        self.render_and_display(frames, idx)

    def sync_preview(self):
        if not self.active_clip_list or self.is_exporting:
            return
        idx = self.slider.value()
        frames = self.read_frames_with_cache(idx, is_seeking=True)
        self.render_and_display(frames, idx)

    def read_frames_with_cache(self, global_idx, is_seeking=False, skip_count=0):
        clip_idx = 0
        for i in range(len(self.clip_frame_offsets) - 1):
            if self.clip_frame_offsets[i] <= global_idx < self.clip_frame_offsets[i+1]:
                clip_idx = i
                break
        else:
            clip_idx = len(self.active_clip_list) - 1

        local_idx = global_idx - self.clip_frame_offsets[clip_idx]

        if self.current_active_clip_idx != clip_idx:
            self.switch_to_clip_index(clip_idx)
            is_seeking = True

        frames = {}
        for k in ['front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar']:
            if k in self.caps:
                cap = self.caps[k]
                if is_seeking:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, local_idx)
                elif skip_count > 0:
                    for _ in range(skip_count):
                        cap.grab()
                ret, img = cap.read()
                if ret and img is not None and img.size > 0:
                    if k != 'front' and not self.is_exporting:
                        img = cv2.resize(img, (480, 270), interpolation=cv2.INTER_NEAREST)
                    self.last_valid_preview_frames[k] = img
                    frames[k] = img
                elif k in self.last_valid_preview_frames:
                    frames[k] = self.last_valid_preview_frames[k]
                else:
                    frames[k] = None
        return frames

    def render_and_display(self, frames, global_idx):
        opts = self.get_current_options()
        clip_idx = self.current_active_clip_idx if self.current_active_clip_idx >= 0 else 0
        local_idx = global_idx - self.clip_frame_offsets[clip_idx]

        # 현재 활성 클립의 base_time을 직접 계산 (self.base_time 타이밍 의존 제거)
        clip_base_time = self.base_time  # 기본값
        if self.active_clip_list and clip_idx < len(self.active_clip_list):
            prefix = self.active_clip_list[clip_idx].get("prefix", "")
            m = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', prefix)
            if m:
                clip_base_time = datetime.strptime(m.group(1), "%Y-%m-%d_%H-%M-%S")

        decoder = self.active_decoders[clip_idx] if clip_idx < len(self.active_decoders) else self.decoder

        grid = OverlayRenderer.render(
            frames, local_idx, clip_base_time, opts, decoder, self.fps, target_size=(1920, 1080)
        )
        self.last_grid_image = grid.copy()
        self._display_grid_image(grid)


    def _display_grid_image(self, grid_mat):
        grid_rgb = cv2.cvtColor(grid_mat, cv2.COLOR_BGR2RGB)
        h, w, ch = grid_rgb.shape
        qimg = QImage(grid_rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(qimg)

        scaled_pixmap = pixmap.scaled(
            self.preview_lbl.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation
        )
        self.preview_lbl.setPixmap(scaled_pixmap)

    def toggle_play(self):
        if self.is_exporting or not self.active_clip_list: 
            return

        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText("▶ 재생 (Space)")
            self.play_start_wall_time = None
        else:
            if self.slider.value() >= self.total_frames - 1:
                self.slider.setValue(0)
            self.play_start_wall_time = time.perf_counter()
            self.play_start_frame = self.slider.value()
            self.timer.start(16)
            self.btn_play.setText("⏸ 일시정지 (Space)")

    def on_preview_speed_slider_changed(self, val):
        speed = val * 0.5  # 1→0.5x, 2→1.0x, ... 10→5.0x
        self.lbl_preview_speed_val.setText(f"{speed:.1f}x")
        # 재생 중이면 기준 시간 리셋 (배속 변경 시 튀는 현상 방지)
        if self.timer.isActive():
            self.play_start_frame = self.slider.value()
            self.play_start_wall_time = time.perf_counter()

    def play_next_frame_smooth(self):
        if not self.active_clip_list or self.play_start_wall_time is None:
            return

        speed_val = getattr(self, 'slider_preview_speed', None)
        speed = (speed_val.value() * 0.5) if speed_val else 1.0

        elapsed_sec = time.perf_counter() - self.play_start_wall_time
        target_frame = self.play_start_frame + int(elapsed_sec * self.fps * speed)

        if target_frame >= self.total_frames:
            self.slider.setValue(self.total_frames - 1)
            # 자동재생 체크박스가 ON일 때만 다음 클립으로 자동 이동
            if hasattr(self, 'chk_auto_next') and self.chk_auto_next.isChecked():
                self.play_next_clip(auto=True)
            else:
                self.toggle_play()  # 재생 정지
            return

        curr_slider_val = self.slider.value()
        if target_frame == curr_slider_val:
            return

        skip = max(0, target_frame - curr_slider_val - 1)
        # 배속이 빠를 때 키프레임 점프 오차 방지: 150프레임 이상 건너뛸 때만 seek
        is_seek = skip > 150
        frames = self.read_frames_with_cache(target_frame, is_seeking=is_seek, skip_count=skip if not is_seek else 0)

        self.slider.blockSignals(True)
        self.slider.setValue(target_frame)
        self.slider.blockSignals(False)

        self.update_current_time_display(target_frame)
        self.render_and_display(frames, target_frame)


    def set_layout_mode(self, mode):
        """ 화면 레이아웃 모드 전환 및 미리보기 즉시 갱신 """
        self.current_layout_mode = mode
        
        # 전면만 표시하는 프레임 선택 시 PIP OFF 및 체크박스 비활성화
        if mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]:
            self.chks["pillar_pip"].blockSignals(True)
            self.chks["pillar_pip"].setChecked(False)
            self.chks["pillar_pip"].setEnabled(False)
            self.chks["pillar_pip"].blockSignals(False)
        else:
            has_pillars = False
            if self.active_clip_list:
                has_pillars = ('left_pillar' in self.active_clip_list[0]["cams"]) or ('right_pillar' in self.active_clip_list[0]["cams"])
            self.chks["pillar_pip"].setEnabled(has_pillars)

        self.update_layout_buttons_style()
        self.sync_preview()
        self.update_estimated_size()

    def update_layout_buttons_style(self):
        """ 3개 레이아웃 버튼의 활성/비활성 테마 스타일 적용 """
        active_style = """
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0078D7, stop:1 #00B4D8);
            color: #FFFFFF;
            border: 1px solid #00E6FF;
            font-weight: bold;
            border-radius: 4px;
            font-size: 11px;
        """
        inactive_style = """
            background-color: #1A1D24;
            color: #A0A6B5;
            border: 1px solid #2A2E39;
            border-radius: 4px;
            font-size: 11px;
        """
        
        self.btn_layout_1to3.setStyleSheet(active_style if self.current_layout_mode == "기본 (1:3 세로배치)" else inactive_style)
        self.btn_layout_2x2.setStyleSheet(active_style if self.current_layout_mode == "2x2 분할 (전후/좌우)" else inactive_style)
        self.btn_layout_front.setStyleSheet(active_style if self.current_layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1"] else inactive_style)

    def update_current_time_display(self, frame_idx):
        if self.total_frames <= 0:
            return

        dur_sec = self.total_frames / self.fps
        curr_sec = frame_idx / self.fps

        curr_t = str(timedelta(seconds=int(curr_sec)))
        dur_t = str(timedelta(seconds=int(dur_sec)))

        # 불필요한 프레임 번호 제거하고 직관적인 시간 형식으로 표시
        self.lbl_curr_time.setText(f"{curr_t} / {dur_t}")

    def update_time_ticks(self):
        pass

    def update_fps_options(self):
        has_pillars = False
        if self.active_clip_list:
            has_pillars = ('left_pillar' in self.active_clip_list[0]["cams"]) or ('right_pillar' in self.active_clip_list[0]["cams"])
        base_fps = 24.0 if has_pillars else 36.0

        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0

        max_fps = min(120, int(round(base_fps * export_speed)))

        self.combo_fps.blockSignals(True)
        self.combo_fps.clear()

        fps_candidates = [
            (f"{max_fps} FPS (원본 최적)", max_fps),
            ("60 FPS", 60),
            ("30 FPS (표준)", 30),
            ("24 FPS (시네마)", 24)
        ]

        added = set()
        for label, val in fps_candidates:
            if val <= max_fps and val not in added:
                self.combo_fps.addItem(label, val)
                added.add(val)

        self.combo_fps.setCurrentIndex(0)
        self.combo_fps.blockSignals(False)
        self.update_estimated_size()

    def on_export_speed_changed(self):
        self.update_fps_options()

    def update_estimated_size(self):
        if not self.active_clip_list or self.total_frames <= 0:
            self.lbl_est_size.setText("예상 크기: 약 0 MB")
            return

        start_dt = self.start_point["dt"] if self.start_point else self.base_time
        end_dt = self.end_point["dt"] if self.end_point else (self.base_time + timedelta(seconds=self.total_frames / self.fps))

        dur_sec = (end_dt - start_dt).total_seconds()
        if dur_sec <= 0:
            self.lbl_est_size.setText("예상 크기: 약 0 MB")
            return

        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0

        out_duration_sec = dur_sec / export_speed

        res_key = self.combo_res.currentText()
        target_fps = self.combo_fps.currentData() or 30
        # ── FPS 보정: H.264 CRF 인코딩은 프레임 간 압축으로 인해 FPS 증가 시 비트레이트가 비선형(약 0.45승)으로 증가 ──
        fps_ratio = (target_fps / 30.0) ** 0.45

        layout_mode = getattr(self, 'current_layout_mode', "기본 (1:3 세로배치)")

        # ── 해상도 × 레이아웃 실측 MB/분 테이블 ──────────────────────────────
        # 288케이스 실측 인코딩 매트릭스 결과 (CRF26, All-ON, 1x배속, 30fps 기준)
        MEASURED_MBPM = {
            ("기본 (1:3 세로배치)",       "QHD (2560x1440) - 최고화질"):  77.6,
            ("기본 (1:3 세로배치)",       "FHD (1920x1080) - 권장 표준"): 50.2,
            ("기본 (1:3 세로배치)",       "HD (1280x720) - 용량 절감"):   27.1,
            ("기본 (1:3 세로배치)",       "Compact (960x540) - 모바일용"): 17.7,
            ("2x2 분할 (전후/좌우)",      "QHD (2560x1440) - 최고화질"):  90.8,
            ("2x2 분할 (전후/좌우)",      "FHD (1920x1080) - 권장 표준"): 61.9,
            ("2x2 분할 (전후/좌우)",      "HD (1280x720) - 용량 절감"):   32.0,
            ("2x2 분할 (전후/좌우)",      "Compact (960x540) - 모바일용"): 22.0,
            ("전면 단독 (전방 풀스크린)",  "QHD (2560x1440) - 최고화질"):  78.8,
            ("전면 단독 (전방 풀스크린)",  "FHD (1920x1080) - 권장 표준"): 42.5,
            ("전면 단독 (전방 풀스크린)",  "HD (1280x720) - 용량 절감"):   19.2,
            ("전면 단독 (전방 풀스크린)",  "Compact (960x540) - 모바일용"): 13.7,
        }
        base_mbpm = MEASURED_MBPM.get((layout_mode, res_key), 50.2)

        # ── 오버레이 보정: 맵/PIP 없을 때 -13% (실측: 43.7/50.2 = 0.870) ──
        has_map = self.chks.get("map") and self.chks["map"].isChecked()
        has_pip = self.chks.get("pillar_pip") and self.chks["pillar_pip"].isChecked()
        overlay_factor = 1.0 if (has_map or has_pip) else 0.87

        # ── 최종 예상 용량 계산 (실측 정밀 피팅 계수: 0.65) ──
        # out_duration_sec = 배속 적용 후 실제 출력 길이(초)
        est_mb = base_mbpm * fps_ratio * overlay_factor * (out_duration_sec / 60.0) * 0.65

        self.lbl_est_size.setText(f"예상 크기: 약 {est_mb:.1f} MB")





    def get_current_options(self):
        opts = {k: v.isChecked() for k, v in self.chks.items()}
        opts["layout"] = getattr(self, 'current_layout_mode', "기본 (1:3 세로배치)")
        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        opts["export_speed"] = float(m.group(1)) if m else 1.0
        
        opts["brightness"] = self.slider_brightness.value()
        opts["contrast"] = self.slider_contrast.value() / 100.0
        return opts

    def set_in_point(self):
        if self.is_exporting or not self.active_clip_list:
            return
        curr_f = self.slider.value()
        curr_dt = self.base_time + timedelta(seconds=curr_f / self.fps)

        if self.end_point:
            if curr_dt >= self.end_point["dt"]:
                QMessageBox.warning(self, "경고", "시작점이 끝점보다 뒤이거나 같습니다.\n시간 순서가 역순이므로 지정할 수 없습니다.")
                return
            dur_sec = (self.end_point["dt"] - curr_dt).total_seconds()
            if dur_sec > 1800.0:
                QMessageBox.warning(self, "경고", f"선택 구간이 {dur_sec / 60.0:.1f}분입니다.\n최대 내보내기 가능 길이는 30분(1,800초)입니다.")
                return

        self.start_point = {
            "item": getattr(self, 'current_item', None),
            "frame": curr_f,
            "dt": curr_dt,
            "base_time": self.base_time
        }
        self.update_range_highlight()

    def set_out_point(self):
        if self.is_exporting or not self.active_clip_list:
            return
        curr_f = self.slider.value()
        curr_dt = self.base_time + timedelta(seconds=curr_f / self.fps)

        if self.start_point:
            if curr_dt <= self.start_point["dt"]:
                QMessageBox.warning(self, "경고", "끝점이 시작점보다 앞서거나 같습니다.\n시간 순서가 역순이므로 지정할 수 없습니다.")
                return
            dur_sec = (curr_dt - self.start_point["dt"]).total_seconds()
            if dur_sec > 1800.0:
                QMessageBox.warning(self, "경고", f"선택 구간이 {dur_sec / 60.0:.1f}분입니다.\n최대 내보내기 가능 길이는 30분(1,800초)입니다.")
                return

        self.end_point = {
            "item": getattr(self, 'current_item', None),
            "frame": curr_f,
            "dt": curr_dt,
            "base_time": self.base_time
        }
        self.update_range_highlight()

    def reset_range_points(self):
        if self.is_exporting:
            return
        self.start_point = None
        self.end_point = None
        self.update_range_highlight()

    def _clear_tree_range_highlights(self):
        """트리 내 모든 클립 아이템의 범위 하이라이트(배경·텍스트색)를 초기 상태로 복원."""
        brush_white = QBrush(QColor("#FFFFFF"))
        brush_green = QBrush(QColor("#00FF66"))
        brush_red   = QBrush(QColor("#FF3333"))

        root = self.tree_explorer.invisibleRootItem()
        for ci in range(root.childCount()):         # 카테고리
            cat = root.child(ci)
            for ei in range(cat.childCount()):      # 이벤트 그룹
                ev = cat.child(ei)
                cdata = ev.data(0, Qt.ItemDataRole.UserRole) or {}
                if cdata.get("is_group", False):
                    # 센트리 그룹: 배경·텍스트 모두 초기화
                    ev.setBackground(0, QBrush())
                    ev.setBackground(1, QBrush())
                    ev.setForeground(0, brush_white)
                    ev.setForeground(1, brush_red if cdata.get("is_sentry") else brush_white)
                else:
                    for ki in range(ev.childCount()):  # 개별 클립
                        clip_item = ev.child(ki)
                        kdata = clip_item.data(0, Qt.ItemDataRole.UserRole) or {}
                        is_park = kdata.get("is_sentry", False)
                        clip_item.setBackground(0, QBrush())
                        clip_item.setBackground(1, QBrush())
                        clip_item.setForeground(0, brush_white)
                        clip_item.setForeground(1, brush_green if is_park else brush_white)


    def _apply_tree_range_highlights(self, s_dt, e_dt):
        """s_dt ~ e_dt 구간에 걸치는 클립 아이템을 노란색으로 하이라이트."""
        brush_yellow_bg = QBrush(QColor("#5C4A00"))   # 어두운 황금색 배경 (다크 테마용)
        brush_yellow_fg = QBrush(QColor("#FFD700"))   # 골드 텍스트
        clip_dur = timedelta(seconds=60)              # 테슬라 클립 기본 길이 1분

        root = self.tree_explorer.invisibleRootItem()
        for ci in range(root.childCount()):
            cat = root.child(ci)
            for ei in range(cat.childCount()):
                ev = cat.child(ei)
                cdata = ev.data(0, Qt.ItemDataRole.UserRole) or {}

                if cdata.get("is_group", False):
                    # 센트리 그룹: clip_list 첫/마지막 prefix로 범위 추정
                    clips = cdata.get("clip_list", [])
                    if not clips:
                        continue
                    try:
                        def parse_prefix(pfx):
                            m = re.match(r'(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})', pfx)
                            return datetime.strptime(f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}", "%Y-%m-%d %H:%M:%S") if m else None
                        clip_start = parse_prefix(clips[0]["prefix"])
                        clip_end   = parse_prefix(clips[-1]["prefix"])
                        if clip_end:
                            clip_end += clip_dur
                        if clip_start and clip_end and clip_start < e_dt and clip_end > s_dt:
                            ev.setBackground(0, brush_yellow_bg)
                            ev.setBackground(1, brush_yellow_bg)
                            ev.setForeground(0, brush_yellow_fg)
                    except Exception:
                        pass
                else:
                    for ki in range(ev.childCount()):
                        clip_item = ev.child(ki)
                        kdata = clip_item.data(0, Qt.ItemDataRole.UserRole) or {}
                        clips = kdata.get("clip_list", [])
                        if not clips:
                            continue
                        try:
                            m = re.match(r'(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})', clips[0]["prefix"])
                            if not m:
                                continue
                            clip_start = datetime.strptime(
                                f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}", "%Y-%m-%d %H:%M:%S")
                            clip_end = clip_start + clip_dur
                            # 겹치면 하이라이트 (clip_start < e_dt AND clip_end > s_dt)
                            if clip_start < e_dt and clip_end > s_dt:
                                clip_item.setBackground(0, brush_yellow_bg)
                                clip_item.setBackground(1, brush_yellow_bg)
                                clip_item.setForeground(0, brush_yellow_fg)
                            else:
                                clip_item.setBackground(0, QBrush())
                                clip_item.setBackground(1, QBrush())
                                # 원래 텍스트 색 복원 (주차=green, 주행=white)
                                is_park = kdata.get("is_sentry", False)
                                orig_color = QColor("#00FF66") if is_park else QColor("#FFFFFF")
                                clip_item.setForeground(0, QBrush(orig_color))
                        except Exception:
                            pass

    def update_range_highlight(self):
        if not self.active_clip_list or (not self.start_point and not self.end_point):
            self.slider.set_range_points(None, None)
            total_sec = self.total_frames / self.fps if self.total_frames > 0 else 0
            self.lbl_range_info.setText(f"선택 구간: 미지정 (전체 {total_sec:.1f}초)")
            self._clear_tree_range_highlights()
            self.update_estimated_size()
            return

        # 첫 클립의 시작 시간과 전체 클립 기준 끝 시간으로 계산
        item_start_dt = self.base_time
        if self.active_clip_list:
            last_clip = self.active_clip_list[-1]
            match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', last_clip.get("prefix", ""))
            if match:
                last_clip_start = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S")
                cap_tmp = cv2.VideoCapture(last_clip["cams"]["front"])
                last_fcount = int(cap_tmp.get(cv2.CAP_PROP_FRAME_COUNT)) or int(60 * self.fps)
                last_fps = cap_tmp.get(cv2.CAP_PROP_FPS) or self.fps
                cap_tmp.release()
                item_end_dt = last_clip_start + timedelta(seconds=last_fcount / last_fps)
            else:
                item_end_dt = self.base_time + timedelta(seconds=(self.total_frames / self.fps))
        else:
            item_end_dt = self.base_time + timedelta(seconds=(self.total_frames / self.fps))

        in_f = None
        out_f = None

        if self.start_point:
            s_dt = self.start_point["dt"]
            if s_dt >= item_end_dt:
                in_f = None
            elif s_dt <= item_start_dt:
                in_f = 0
            else:
                in_f = int((s_dt - item_start_dt).total_seconds() * self.fps)
        elif self.end_point:
            in_f = 0

        if self.end_point:
            e_dt = self.end_point["dt"]
            if e_dt <= item_start_dt:
                out_f = None
            elif e_dt >= item_end_dt:
                out_f = self.total_frames - 1
            else:
                out_f = int((e_dt - item_start_dt).total_seconds() * self.fps)
        elif self.start_point:
            out_f = self.total_frames - 1

        if in_f is not None and out_f is not None and in_f <= out_f:
            self.slider.set_range_points(in_f, out_f)
        else:
            self.slider.set_range_points(None, None)

        if self.start_point and self.end_point:
            dur = (self.end_point["dt"] - self.start_point["dt"]).total_seconds()
            t1 = self.start_point["dt"].strftime("%H:%M:%S")
            t2 = self.end_point["dt"].strftime("%H:%M:%S")
            self.lbl_range_info.setText(f"구간: {t1} ~ {t2} ({dur:.1f}초)")
        elif self.start_point:
            t1 = self.start_point["dt"].strftime("%H:%M:%S")
            self.lbl_range_info.setText(f"시작점: {t1} (끝점 선택 필요)")
        elif self.end_point:
            t2 = self.end_point["dt"].strftime("%H:%M:%S")
            self.lbl_range_info.setText(f"끝점: {t2} (시작점 선택 필요)")

        # ── 트리 하이라이트 ──────────────────────────────────────────────
        self._clear_tree_range_highlights()
        s_dt_hl = self.start_point["dt"] if self.start_point else (
            self.base_time)
        e_dt_hl = self.end_point["dt"] if self.end_point else (
            self.base_time + timedelta(seconds=self.total_frames / self.fps))
        self._apply_tree_range_highlights(s_dt_hl, e_dt_hl)

        self.update_estimated_size()


    def build_target_clip_chain(self):
        if not self.start_point or not self.end_point:
            # 구간 미지정 시 현재 로드된 클립 전체 내보내기
            target_clips = []
            for i, clip in enumerate(self.active_clip_list):
                match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', clip["prefix"])
                clip_base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time
                cap = cv2.VideoCapture(clip["cams"]["front"])
                fcount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or int(60 * self.fps)
                cap.release()
                target_clips.append({
                    "cams": clip["cams"],
                    "start_f": 0,
                    "end_f": fcount - 1,
                    "base_time": clip_base_time
                })
            return target_clips

        start_item = self.start_point.get("item")
        end_item = self.end_point.get("item")

        # 1) 동일한 아이템 내에서 선택된 경우
        if start_item == end_item and start_item is not None:
            cdata = start_item.data(0, Qt.ItemDataRole.UserRole)
            if cdata.get("is_group", False):
                s_dt = self.start_point["dt"]
                e_dt = self.end_point["dt"]
                target_clips = []
                for i, c_info in enumerate(self.active_clip_list):
                    match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
                    b_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time
                    cap = cv2.VideoCapture(c_info["cams"]["front"])
                    fcount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or int(60 * self.fps)
                    cap.release()
                    c_end_time = b_time + timedelta(seconds=fcount / self.fps)

                    if c_end_time <= s_dt or b_time >= e_dt:
                        continue

                    local_s = max(0, int((s_dt - b_time).total_seconds() * self.fps)) if s_dt > b_time else 0
                    local_e = min(fcount - 1, int((e_dt - b_time).total_seconds() * self.fps)) if e_dt < c_end_time else (fcount - 1)

                    target_clips.append({
                        "cams": c_info["cams"],
                        "base_time": b_time,
                        "start_f": local_s,
                        "end_f": local_e
                    })
                return target_clips
            else:
                c_info = cdata["clip_list"][0]
                match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
                b_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time
                return [{
                    "cams": c_info["cams"],
                    "base_time": b_time,
                    "start_f": self.start_point["frame"],
                    "end_f": self.end_point["frame"]
                }]

        # 2) 서로 다른 개별 클립 간에 걸쳐 선택된 경우
        if start_item and end_item:
            parent_item = start_item.parent()
            if parent_item and parent_item == end_item.parent():
                start_idx = parent_item.indexOfChild(start_item)
                end_idx = parent_item.indexOfChild(end_item)

                if start_idx > end_idx:
                    QMessageBox.warning(self, "경고", "시간 순서가 올바르지 않습니다.")
                    return []

                target_clips = []
                for i in range(start_idx, end_idx + 1):
                    child_item = parent_item.child(i)
                    cdata = child_item.data(0, Qt.ItemDataRole.UserRole)
                    if not cdata or not cdata.get("clip_list"):
                        continue

                    c_info = cdata["clip_list"][0]
                    cap = cv2.VideoCapture(c_info["cams"]["front"])
                    fcount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or int(60 * self.fps)
                    cap.release()

                    # 전역 슬라이더 프레임 → 각 클립의 로컬 프레임으로 변환
                    # clip_frame_offsets 인덱스 불일치 위험이 있으므로 dt 기반으로 안전하게 계산
                    match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
                    b_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time

                    if i == start_idx:
                        s_sec = (self.start_point["dt"] - b_time).total_seconds()
                        s_frame = max(0, int(s_sec * self.fps))
                    else:
                        s_frame = 0

                    if i == end_idx:
                        e_sec = (self.end_point["dt"] - b_time).total_seconds()
                        e_frame = min(fcount - 1, int(e_sec * self.fps))
                    else:
                        e_frame = fcount - 1

                    s_frame = max(0, min(s_frame, fcount - 1))
                    e_frame = max(0, min(e_frame, fcount - 1))

                    target_clips.append({
                        "cams": c_info["cams"],
                        "base_time": b_time,
                        "start_f": s_frame,
                        "end_f": e_frame
                    })
                return target_clips

        return []

    def set_controls_enabled(self, enabled):
        self.btn_load.setEnabled(enabled)
        self.combo_res.setEnabled(enabled)
        self.btn_layout_1to3.setEnabled(enabled)
        self.btn_layout_2x2.setEnabled(enabled)
        self.btn_layout_front.setEnabled(enabled)
        self.combo_export_speed.setEnabled(enabled)
        self.combo_fps.setEnabled(enabled)

        self.btn_prev.setEnabled(enabled)
        self.btn_play.setEnabled(enabled)
        self.btn_next.setEnabled(enabled)
        self.btn_in.setEnabled(enabled)
        self.btn_out.setEnabled(enabled)
        self.btn_reset_range.setEnabled(enabled)
        if hasattr(self, 'slider_preview_speed'):
            self.slider_preview_speed.setEnabled(enabled)

    def on_click_export_button(self):
        if not self.active_clip_list:
            QMessageBox.warning(self, "경고", "내보낼 비디오 클립을 먼저 선택해주세요.")
            return

        if self.is_exporting:
            if hasattr(self, 'worker') and self.worker:
                self.worker.stop()
            return

        default_dir = get_app_dir()
        default_path = os.path.join(default_dir, "CT_output.mp4")
        out_path, _ = QFileDialog.getSaveFileName(self, "내보낼 MP4 파일 저장 위치 선택", default_path, "MP4 (*.mp4)")
        if not out_path:
            return

        target_clips = self.build_target_clip_chain()
        if not target_clips:
            QMessageBox.warning(self, "경고", "내보낼 유효한 프레임 구간이 없습니다.")
            return

        if self.timer.isActive():
            self.toggle_play()

        self.is_exporting = True
        self.set_controls_enabled(False)
        self.btn_export.setText("⏹ 내보내기 취소")
        self.btn_export.setStyleSheet("background-color: #8A1C1C; color: #FFFFFF; font-weight: bold;")
        self.on_export_progress(0)

        res_key = self.combo_res.currentText()
        target_size = RESOLUTIONS.get(res_key, RESOLUTIONS["QHD (2560x1440) - 최고화질"])["size"]

        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        export_speed = float(m.group(1)) if m else 1.0

        source_fps = self.fps or 36.0
        target_fps = self.combo_fps.currentData() or min(120, int(round(source_fps * export_speed)))

        opts = self.get_current_options()

        # target_clips의 각 클립 front 경로와 active_clip_list를 매칭해서
        # 정확한 decoder를 1:1로 추출 (인덱스 불일치 방지)
        clip_front_to_decoder = {}
        for idx, c in enumerate(self.active_clip_list):
            front_path = c.get("cams", {}).get("front", "")
            if idx < len(self.active_decoders):
                clip_front_to_decoder[front_path] = self.active_decoders[idx]

        matched_decoders = []
        for tc in target_clips:
            front_path = tc.get("cams", {}).get("front", "")
            matched_decoders.append(clip_front_to_decoder.get(front_path, None))

        self.worker = ExportWorker(
            target_clips, opts, source_fps, target_fps, export_speed, target_size, out_path, active_decoders=matched_decoders
        )
        self.worker.progress.connect(self.on_export_progress)
        self.worker.finished.connect(self.on_export_finished)
        self.worker.error.connect(self.on_export_error)
        self.worker.cancelled.connect(self.on_export_cancelled)
        self.worker.start()

    def on_export_progress(self, percent):
        self.pbar.setValue(percent)
        if self.last_grid_image is not None and percent < 100:
            overlaid = OverlayRenderer.apply_rendering_overlay(self.last_grid_image, percent)
            self._display_grid_image(overlaid)

    def on_export_finished(self, output_path):
        self.cleanup_export_ui()
        self.last_exported_video_path = output_path
        self.btn_qr_share.setEnabled(True)
        self.btn_qr_share.setStyleSheet("background-color: #0078D7; color: #FFFFFF; font-weight: bold;")

        # 작업표시줄 알림
        QApplication.alert(self, 0)

        # 완료 다이얼로그 (폴더 열기 / QR 전송 / 확인 선택 가능)
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("내보내기 완료")
        msg_box.setText(f"비디오 렌더링이 성공적으로 완료되었습니다!\n\n저장 경로:\n{output_path}")
        msg_box.setIcon(QMessageBox.Icon.Information)

        btn_open = msg_box.addButton("📁 저장 폴더 열기", QMessageBox.ButtonRole.ActionRole)
        btn_qr = msg_box.addButton("📱 스마트폰 QR 전송", QMessageBox.ButtonRole.ActionRole)
        btn_qr.setStyleSheet("background-color: #0078D7; color: white; font-weight: bold; padding: 4px 10px;")
        btn_ok = msg_box.addButton("확인", QMessageBox.ButtonRole.AcceptRole)

        msg_box.exec()

        clicked = msg_box.clickedButton()
        if clicked == btn_qr:
            self.open_qr_share_dialog(output_path)
        elif clicked == btn_open:
            if os.path.exists(output_path):
                norm_path = os.path.normpath(os.path.abspath(output_path))
                try:
                    subprocess.Popen(f'explorer /select,"{norm_path}"')
                except Exception:
                    try:
                        os.startfile(os.path.dirname(norm_path))
                    except Exception:
                        pass

    def on_export_error(self, err_msg):
        self.cleanup_export_ui()
        QMessageBox.critical(self, "내보내기 오류", f"영상 저장 중 오류가 발생했습니다:\n{err_msg}")

    def on_export_cancelled(self):
        self.cleanup_export_ui()
        QMessageBox.information(self, "내보내기 취소", "영상 내보내기 작업이 사용자에 의해 취소되었습니다.")

    def cleanup_export_ui(self):
        self.is_exporting = False
        self.set_controls_enabled(True)
        self.btn_export.setText("선택 구간 내보내기 (MP4)")
        self.btn_export.setStyleSheet("")
        self.pbar.setValue(0)

        idx = self.slider.value()
        frames = self.read_frames_with_cache(idx, is_seeking=True)
        self.render_and_display(frames, idx)

    def on_click_qr_share(self):
        if self.last_exported_video_path and os.path.exists(self.last_exported_video_path):
            self.open_qr_share_dialog(self.last_exported_video_path)
        else:
            QMessageBox.warning(self, "경고", "내보내기가 완료된 비디오 파일을 찾을 수 없습니다.")

    def open_qr_share_dialog(self, video_path):
        dlg = QRShareDialog(video_path, self)
        dlg.exec()

    def on_update_found(self, tag_name, url):
        reply = QMessageBox.question(
            self, "새 버전 업데이트 알림",
            f"새로운 버전({tag_name})이 릴리즈되었습니다!\n지금 다운로드 페이지로 이동하시겠습니까?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            webbrowser.open(url)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.last_grid_image is not None and not self.is_exporting:
            self._display_grid_image(self.last_grid_image)

    def closeEvent(self, event):
        self.stop_motion_scanner()
        if hasattr(self, 'loader_worker') and self.loader_worker and self.loader_worker.isRunning():
            self.loader_worker.stop()
            self.loader_worker.wait()
        if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait()
        for cap in self.caps.values():
            cap.release()
        super().closeEvent(event)


# 하위 호환용 알리아스
TeslaStudioPro = CTDashcamStudio
