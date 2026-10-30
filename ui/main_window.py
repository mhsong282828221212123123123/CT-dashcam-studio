import sys
import os
import re
import math
import time
import subprocess
import webbrowser
from datetime import datetime, timedelta
from concurrent.futures import ThreadPoolExecutor

import cv2
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QCheckBox, QFileDialog, QGroupBox,
    QProgressBar, QMessageBox, QComboBox, QTreeWidget, QTreeWidgetItem, QTreeWidgetItemIterator,
    QSplitter, QStyleFactory, QDialog, QProgressDialog, QApplication,
    QSizePolicy, QSlider, QFrame
)
from PyQt6.QtCore import Qt, QTimer, QRect, QPoint, QEvent, QSize
from PyQt6.QtGui import QImage, QPixmap, QColor, QBrush, QIcon, QKeySequence, QShortcut

from core.clip_groups import group_recent_clips, clip_datetime, group_title, clip_identity
from core.themes import color, set_theme_style, apply_theme, current_theme, theme_names
from core.constants import APP_VERSION, RESOLUTIONS, SENSITIVITY_LEVELS
from core.utils import resource_path, get_app_dir
from core.i18n import (
    tr, set_language, get_current_language, LANGUAGES,
    RESOLUTION_CHOICES, EXPORT_SPEED_CHOICES, SENSITIVITY_CHOICES
)
from core.decoder import RealTeslaSEIDecoder
from core.map_loader import OpenStreetMapTileLoader
from core.renderer import OverlayRenderer
from core.scanner import LightMotionScanner
from core.exporter import ExportWorker
from core.privacy_filter import PrivacyFilter
from network.updater import UpdateCheckWorker
from ui.widgets import ClickableLabel, HighlightSlider, SingleCameraPopup, ToastNotification
from ui.dialogs import QRShareDialog, HotkeyGuideDialog
from ui.loader_worker import ClipLoaderWorker


class CTDashcamStudio(QMainWindow):
    def __init__(self):
        super().__init__()
        apply_theme(current_theme(), persist=False)
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
        self.last_raw_preview_frames = {}
        self.last_preview_slot_size = {}

        self.detected_event_blocks = []
        self.scanner = None
        self.motion_cache = {}

        self.start_point = None
        self.end_point = None

        self.play_start_wall_time = None
        self.play_start_frame = 0
        self._preview_pool = ThreadPoolExecutor(max_workers=6)

        self.current_single_cam = "front"
        self.single_cam_popup = None
        self.toast_widget = None
        self.telemetry_order = list(OverlayRenderer.DEFAULT_TELEMETRY_ORDER)
        self._dragging_telemetry_key = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.play_next_frame_smooth)

        self.setAcceptDrops(True)
        self.init_ui()
        self.setup_shortcuts()

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
        self.left_panel = left_panel
        left_panel.setMinimumWidth(370)
        left_panel.setMaximumWidth(460)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)

        self.btn_load = QPushButton(tr("btn_load"))
        self.btn_load.setFixedHeight(34)
        set_theme_style(self.btn_load, "background-color:@ct-primary-bg; color:@ct-primary-text; font-weight:600; font-size:@font-body;")
        self.btn_load.clicked.connect(self.load_directory_dialog)
        left_layout.addWidget(self.btn_load)

        self.tree_explorer = QTreeWidget()
        self.tree_explorer.setHeaderLabels([tr("tree_col_explorer"), tr("tree_col_type")])
        self.tree_explorer.setIndentation(14)
        self.tree_explorer.setUniformRowHeights(True)
        self.tree_explorer.setColumnWidth(0, 245)
        self.tree_explorer.setColumnWidth(1, 110)
        self.tree_explorer.header().setStretchLastSection(True)
        self.tree_explorer.itemClicked.connect(self.on_tree_item_clicked)
        left_layout.addWidget(self.tree_explorer, stretch=1)

        # 1) 오버레이 표시 옵션 (컴팩트 2열 그리드)
        self.opt_group = QGroupBox(tr("grp_overlays"))
        opt_layout = QGridLayout(self.opt_group)
        opt_layout.setContentsMargins(8, 6, 8, 6)
        opt_layout.setHorizontalSpacing(8)
        opt_layout.setVerticalSpacing(3)

        self.chks = {
            "timestamp": QCheckBox(tr("chk_timestamp")),
            "speed": QCheckBox(tr("chk_speed")),
            "fsd": QCheckBox(tr("chk_fsd")),
            "turn_signal": QCheckBox(tr("chk_turn_signal")),
            "steering": QCheckBox(tr("chk_steering_wheel")),
            "pedal": QCheckBox(tr("chk_pedals")),
            "pillar_pip": QCheckBox(tr("chk_pillar_pip")),
            "map": QCheckBox(tr("chk_osm_minimap")),
            "blur_face": QCheckBox(tr("chk_blur_faces")),
            "blur_plate": QCheckBox(tr("chk_blur_plates")),
            "enhance_plate": QCheckBox(tr("chk_enhance_plates")),
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

        left_layout.addWidget(self.opt_group)

        # 1-2) 개인정보 보호 및 번호판 개선 그룹박스
        self.privacy_group = QGroupBox(tr("grp_privacy"))
        privacy_layout = QGridLayout(self.privacy_group)
        privacy_layout.setContentsMargins(8, 6, 8, 6)
        privacy_layout.setHorizontalSpacing(8)
        privacy_layout.setVerticalSpacing(3)

        for idx, k in enumerate(["blur_face"]):
            chk = self.chks[k]
            chk.setChecked(False)  # 기본 OFF
            chk.stateChanged.connect(self.sync_preview)
            privacy_layout.addWidget(chk, 0, idx)

        chk_blur = self.chks["blur_plate"]
        chk_blur.setChecked(False)
        chk_blur.stateChanged.connect(self.on_blur_plate_toggled)
        privacy_layout.addWidget(chk_blur, 0, 1)

        # 번호판 선명화 (번호판 모자이크와 상호 배타)
        chk_enh = self.chks["enhance_plate"]
        chk_enh.setChecked(False)
        chk_enh.setToolTip(tr("tt_enhance_plate"))
        chk_enh.stateChanged.connect(self.on_enhance_plate_toggled)
        privacy_layout.addWidget(chk_enh, 1, 0, 1, 2)

        left_layout.addWidget(self.privacy_group)

        # 2) 내보내기 설정 (사이드바 하단 정렬)
        self.exp_group = QGroupBox(tr("grp_export"))
        exp_layout = QGridLayout(self.exp_group)
        exp_layout.setContentsMargins(8, 6, 8, 6)
        exp_layout.setHorizontalSpacing(6)
        exp_layout.setVerticalSpacing(4)

        self.lbl_export_res = QLabel(tr("lbl_res"))
        exp_layout.addWidget(self.lbl_export_res, 0, 0)
        self.combo_res = QComboBox()
        for res_id, tr_k, _, _ in RESOLUTION_CHOICES:
            self.combo_res.addItem(tr(tr_k), res_id)
        self.combo_res.currentIndexChanged.connect(self.update_estimated_size)
        exp_layout.addWidget(self.combo_res, 0, 1, 1, 3)

        self.lbl_export_layout = QLabel(tr("lbl_layout"))
        exp_layout.addWidget(self.lbl_export_layout, 1, 0)
        layout_btn_box = QHBoxLayout()
        layout_btn_box.setSpacing(3)

        self.btn_layout_1to3 = QPushButton(f"{tr('layout_1to3')}")
        self.btn_layout_1to3.setToolTip(tr("layout_1to3_tooltip"))
        self.btn_layout_1to3.setFixedHeight(26)
        self.btn_layout_1to3.clicked.connect(lambda: self.set_layout_mode("기본 (1:3 세로배치)"))

        self.btn_layout_2x2 = QPushButton(tr("layout_2x2"))
        self.btn_layout_2x2.setToolTip(tr("layout_2x2_tooltip"))
        self.btn_layout_2x2.setFixedHeight(26)
        self.btn_layout_2x2.clicked.connect(lambda: self.set_layout_mode("2x2 분할 (전후/좌우)"))

        self.btn_layout_single = QPushButton(f"{tr('layout_single')}")
        self.btn_layout_single.setToolTip(tr("layout_single_tooltip"))
        self.btn_layout_single.setFixedHeight(26)
        self.btn_layout_single.clicked.connect(self.on_click_layout_single)
        self.btn_layout_front = self.btn_layout_single  # 하위 호환성 유지

        for b in [self.btn_layout_1to3, self.btn_layout_2x2, self.btn_layout_single]:
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            layout_btn_box.addWidget(b)

        exp_layout.addLayout(layout_btn_box, 1, 1, 1, 3)

        self.lbl_export_speed = QLabel(tr("lbl_speed"))
        exp_layout.addWidget(self.lbl_export_speed, 2, 0)
        self.combo_export_speed = QComboBox()
        self.combo_export_speed.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        for spd_val, tr_k in EXPORT_SPEED_CHOICES:
            self.combo_export_speed.addItem(tr(tr_k), spd_val)
        self.combo_export_speed.setCurrentIndex(1)  # 기본값: 1.0x (표준)
        self.combo_export_speed.currentIndexChanged.connect(self.on_export_speed_changed)
        exp_layout.addWidget(self.combo_export_speed, 2, 1)

        self.lbl_export_fps = QLabel(tr("lbl_fps"))
        exp_layout.addWidget(self.lbl_export_fps, 2, 2)
        self.combo_fps = QComboBox()
        self.combo_fps.currentIndexChanged.connect(self.update_estimated_size)
        exp_layout.addWidget(self.combo_fps, 2, 3)

        self.lbl_est_size = QLabel(tr("lbl_est_size", size="0"))
        set_theme_style(self.lbl_est_size, "color:@ct-text-main; font-weight:400; font-size:@font-body;")
        exp_layout.addWidget(self.lbl_est_size, 3, 0, 1, 4)

        self.btn_export = QPushButton(tr("btn_export"))
        self.btn_export.setFixedHeight(34)
        set_theme_style(self.btn_export, "background-color:@ct-primary-bg; color:@ct-primary-text; font-weight:600; font-size:@font-body;")
        self.btn_export.clicked.connect(self.on_click_export_button)
        exp_layout.addWidget(self.btn_export, 4, 0, 1, 4)

        self.pbar = QProgressBar()
        self.pbar.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pbar.setFixedHeight(12)
        exp_layout.addWidget(self.pbar, 5, 0, 1, 4)

        self.btn_qr_share = QPushButton(f"{tr('chk_qr_share')}")
        self.btn_qr_share.setFixedHeight(28)
        self.btn_qr_share.setEnabled(False)
        self.btn_qr_share.clicked.connect(self.on_click_qr_share)
        exp_layout.addWidget(self.btn_qr_share, 6, 0, 1, 4)

        left_layout.addWidget(self.exp_group)
        splitter.addWidget(left_panel)

        # =========================================================================
        # 2. 우측 메인 패널 (대형 비디오 뷰 + 날렵한 1줄 타임라인 컨트롤 바)
        # =========================================================================
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        # 비디오 프리뷰 화면 (화면의 대부분을 꽉 채우도록 Expanding 설정)
        self.preview_lbl = QLabel(tr("preview_empty"))
        self.preview_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        set_theme_style(self.preview_lbl, "background-color:@ct-bg-app; border:1px solid @ct-control-border; border-radius:4px;")
        self.preview_lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.preview_lbl.setMinimumSize(640, 360)
        self.preview_lbl.setMouseTracking(True)
        self.preview_lbl.setToolTip(tr("tt_preview_screen"))
        self.preview_lbl.installEventFilter(self)
        right_layout.addWidget(self.preview_lbl, stretch=10)

        # 하단 타임라인 플로팅 컨트롤 덱 (모던 다크 슬레이트 카드형 패널)
        timeline_group = QFrame()
        timeline_group.setObjectName("timelineDeck")
        set_theme_style(timeline_group, """
            QFrame#timelineDeck {
                background:@ct-bg-sidebar;
                border:1px solid @ct-control-border;
                border-radius:10px;
            }
        """)
        bottom_ctrl_layout = QVBoxLayout(timeline_group)
        bottom_ctrl_layout.setContentsMargins(12, 8, 12, 9)
        bottom_ctrl_layout.setSpacing(6)

        def _make_v_divider():
            div = QFrame()
            div.setFrameShape(QFrame.Shape.VLine)
            div.setFrameShadow(QFrame.Shadow.Plain)
            set_theme_style(div, "color:@ct-text-muted; background-color:@ct-control-bg; border:none;")
            div.setFixedSize(1, 18)
            return div

        range_btn_style = """
            QPushButton {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                border-radius:5px;
                padding:2px 9px;
                font-size:@font-body;
                font-weight:400;
            }
            QPushButton:hover {
                background-color:@ct-control-bg;
                border-color:@ct-border-active;
                color:@ct-warning-text;
            }
            QPushButton:pressed {
                background-color:@ct-bg-app;
            }
        """

        transport_sub_btn_style = """
            QPushButton {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                border-radius:6px;
                padding:3px 10px;
                font-size:@font-body;
                font-weight:400;
            }
            QPushButton:hover {
                background-color:@ct-control-bg;
                border-color:@ct-border-active;
                color:@ct-text-main;
            }
            QPushButton:pressed {
                background-color:@ct-control-bg;
            }
        """

        self._play_btn_idle_style = """
            QPushButton {
                background:@ct-selected-bg;
                color:@ct-text-main;
                border:1px solid @ct-border-active;
                border-radius:6px;
                padding:3px 14px;
                font-size:@font-body;
                font-weight:400;
            }
            QPushButton:hover {
                background:@ct-selected-bg;
                border-color:@ct-border-active;
            }
            QPushButton:pressed {
                background-color:@ct-selected-bg;
            }
        """

        self._play_btn_active_style = """
            QPushButton {
                background:@ct-selected-bg;
                color:@ct-text-main;
                border:1px solid @ct-border-active;
                border-radius:6px;
                padding:3px 14px;
                font-size:@font-body;
                font-weight:400;
            }
            QPushButton:hover {
                background:@ct-selected-bg;
            }
        """

        # ── [1단] 커스텀 NLE 타임라인 스크러버 ──
        self.slider = HighlightSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 0)
        self.slider.sliderMoved.connect(self.on_slider_manual_seek)
        self.slider.valueChanged.connect(self.on_slider_manual_seek)
        bottom_ctrl_layout.addWidget(self.slider)

        # ── [2단] 타임코드 모니터 · 센트리 모션 클러스터 · 구간 편집 바 ──
        info_bar = QHBoxLayout()
        info_bar.setContentsMargins(2, 0, 2, 0)
        info_bar.setSpacing(6)

        # (좌) 디지털 타임코드 모니터 배지
        self.lbl_curr_time = QLabel("0:00:00 / 0:00:00")
        set_theme_style(self.lbl_curr_time, """
            QLabel {
                background-color:@ct-bg-app;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                border-radius:5px;
                padding:3px 10px;

                font-weight:400;
                font-size:@font-body;
            }
        """)
        info_bar.addWidget(self.lbl_curr_time)

        info_bar.addWidget(_make_v_divider())

        # (중) 클립 모드 & 센트리 모션 감지 클러스터
        self.lbl_motion_status = QLabel(tr("motion_ready"))
        set_theme_style(self.lbl_motion_status, """
            QLabel {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                border-radius:5px;
                padding:2px 9px;
                font-size:@font-body;
                font-weight:400;
            }
        """)
        info_bar.addWidget(self.lbl_motion_status)

        self.chk_motion = QCheckBox(tr("chk_motion"))
        self.chk_motion.setChecked(True)
        self.chk_motion.stateChanged.connect(self.on_motion_chk_changed)
        info_bar.addWidget(self.chk_motion)

        info_bar.addSpacing(8)

        self.lbl_sensitivity_tag = QLabel(tr("lbl_sensitivity"))
        set_theme_style(self.lbl_sensitivity_tag, "font-size:@font-body; color:@ct-text-main; font-weight:400;")
        info_bar.addWidget(self.lbl_sensitivity_tag)

        self.combo_sensitivity = QComboBox()
        for raw_k, tr_k in SENSITIVITY_CHOICES:
            self.combo_sensitivity.addItem(tr(tr_k), raw_k)
        self.combo_sensitivity.setCurrentIndex(1)
        self.combo_sensitivity.currentIndexChanged.connect(self.on_sensitivity_changed)
        info_bar.addWidget(self.combo_sensitivity)

        self.btn_prev_event = QPushButton(tr("btn_prev_event"))
        self.btn_prev_event.setToolTip(tr("tt_motion_prev"))
        self.btn_prev_event.setFixedHeight(26)
        set_theme_style(self.btn_prev_event, range_btn_style)
        self.btn_prev_event.setVisible(False)
        self.btn_prev_event.clicked.connect(self.jump_to_prev_event)
        info_bar.addWidget(self.btn_prev_event)

        self.btn_next_event = QPushButton(tr("btn_next_event"))
        self.btn_next_event.setToolTip(tr("tt_motion_next"))
        self.btn_next_event.setFixedHeight(26)
        set_theme_style(self.btn_next_event, range_btn_style)
        self.btn_next_event.setVisible(False)
        self.btn_next_event.clicked.connect(self.jump_to_next_event)
        info_bar.addWidget(self.btn_next_event)

        info_bar.addStretch(1)

        bottom_ctrl_layout.addLayout(info_bar)
        range_bar = QHBoxLayout()
        range_bar.setSpacing(6)
        # (우) 선택 구간 배지 & NLE 구간 트림 버튼 클러스터
        self.lbl_range_info = QLabel(tr("range_none", total=tr("time_sec", sec="0.0")))
        set_theme_style(self.lbl_range_info, """
            QLabel {
                background-color:@ct-selected-bg;
                color:@ct-warning-text;
                border:1px solid @ct-selected-bg;
                border-radius:5px;
                padding:2px 9px;
                font-size:@font-body;
                font-weight:400;
            }
        """)
        self.lbl_range_info.setWordWrap(True)
        range_bar.addWidget(self.lbl_range_info, stretch=1)

        range_bar.addSpacing(2)

        self.btn_in = QPushButton(tr("btn_in"))
        self.btn_in.setToolTip(tr("btn_in_tooltip"))
        self.btn_in.setFixedHeight(26)
        set_theme_style(self.btn_in, range_btn_style)
        self.btn_in.clicked.connect(self.set_in_point)
        range_bar.addWidget(self.btn_in)

        self.btn_out = QPushButton(tr("btn_out"))
        self.btn_out.setToolTip(tr("btn_out_tooltip"))
        self.btn_out.setFixedHeight(26)
        set_theme_style(self.btn_out, range_btn_style)
        self.btn_out.clicked.connect(self.set_out_point)
        range_bar.addWidget(self.btn_out)

        self.btn_reset_range = QPushButton(tr("btn_reset_range"))
        self.btn_reset_range.setToolTip(tr("btn_reset_tooltip"))
        self.btn_reset_range.setFixedHeight(26)
        set_theme_style(self.btn_reset_range, range_btn_style)
        self.btn_reset_range.clicked.connect(self.reset_range_points)
        range_bar.addWidget(self.btn_reset_range)

        bottom_ctrl_layout.addLayout(range_bar)

        # 1단과 2단 사이 미세 수평 구분선
        h_sep = QFrame()
        h_sep.setFrameShape(QFrame.Shape.HLine)
        h_sep.setFixedHeight(1)
        set_theme_style(h_sep, "background-color:@ct-control-bg; border:none;")
        bottom_ctrl_layout.addWidget(h_sep)

        # ── [3단] 트랜스포트 재생 · 배속 · 영상 보정 · 단축키 바 ──
        playback_bar = QHBoxLayout()
        playback_bar.setContentsMargins(2, 1, 2, 0)
        playback_bar.setSpacing(6)

        # (좌) 메인 트랜스포트 클러스터
        self.btn_prev = QPushButton(f"|◀ {tr('btn_prev')}")
        self.btn_prev.setToolTip(tr("tt_btn_prev"))
        self.btn_prev.setFixedHeight(28)
        set_theme_style(self.btn_prev, transport_sub_btn_style)
        self.btn_prev.clicked.connect(self.play_prev_clip)
        playback_bar.addWidget(self.btn_prev)

        self.btn_play = QPushButton(tr("btn_play"))
        self.btn_play.setToolTip(tr("tt_btn_play"))
        self.btn_play.setFixedHeight(28)
        set_theme_style(self.btn_play, self._play_btn_idle_style)
        self.btn_play.clicked.connect(self.toggle_play)
        playback_bar.addWidget(self.btn_play)

        self.btn_next = QPushButton(f"{tr('btn_next')} ▶|")
        self.btn_next.setToolTip(tr("tt_btn_next"))
        self.btn_next.setFixedHeight(28)
        set_theme_style(self.btn_next, transport_sub_btn_style)
        self.btn_next.clicked.connect(self.play_next_clip)
        playback_bar.addWidget(self.btn_next)

        playback_bar.addStretch(1)
        playback_bar.addWidget(_make_v_divider())
        playback_bar.addStretch(1)

        # (중) 배속 & 자동재생 클러스터
        self.lbl_spd_tag = QLabel(tr("lbl_speed_tag"))
        set_theme_style(self.lbl_spd_tag, "font-size:@font-body; color:@ct-text-main; font-weight:400;")
        self.lbl_spd_tag.setToolTip(tr("tt_speed_reset"))
        self.lbl_spd_tag.setCursor(Qt.CursorShape.PointingHandCursor)
        self.lbl_spd_tag.installEventFilter(self)
        playback_bar.addWidget(self.lbl_spd_tag)

        self.slider_preview_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_preview_speed.setRange(1, 10)  # 0.5x ~ 5.0x
        self.slider_preview_speed.setValue(2)      # 기본 1.0x
        self.slider_preview_speed.setMinimumWidth(58)
        self.slider_preview_speed.setMaximumWidth(96)
        self.slider_preview_speed.setCursor(Qt.CursorShape.PointingHandCursor)
        self.slider_preview_speed.setToolTip(tr("tt_speed_slider"))
        self.slider_preview_speed.valueChanged.connect(self.on_preview_speed_slider_changed)
        self.slider_preview_speed.installEventFilter(self)
        playback_bar.addWidget(self.slider_preview_speed)

        self.lbl_preview_speed_val = QLabel("1.0x")
        set_theme_style(self.lbl_preview_speed_val, """
            QLabel {
                background-color:@ct-control-bg;
                color:@ct-warning-text;
                border:1px solid @ct-control-border;
                border-radius:4px;
                padding:1px 6px;

                font-size:@font-body;
                font-weight:400;
            }
        """)
        self.lbl_preview_speed_val.setToolTip(tr("tt_speed_reset"))
        self.lbl_preview_speed_val.setCursor(Qt.CursorShape.PointingHandCursor)
        self.lbl_preview_speed_val.installEventFilter(self)
        playback_bar.addWidget(self.lbl_preview_speed_val)

        playback_bar.addSpacing(6)

        self.chk_auto_next = QCheckBox(tr("chk_auto_next"))
        self.chk_auto_next.setChecked(False)  # 기본값: OFF
        self.chk_auto_next.setToolTip(tr("chk_auto_next_tooltip"))
        playback_bar.addWidget(self.chk_auto_next)

        playback_bar.addStretch(1)
        playback_bar.addWidget(_make_v_divider())
        playback_bar.addStretch(1)

        # (우) 언어 선택 드롭다운 · 화면 밝기/대비 보정 · 단축키 클러스터
        self.combo_lang = QComboBox()
        self.combo_lang.setFixedHeight(28)
        self.combo_lang.setFixedWidth(112)
        self.combo_lang.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.combo_lang.setToolTip("Language / 언어 선택")
        set_theme_style(self.combo_lang, """
            QComboBox {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                border:1px solid @ct-control-border;
                border-radius:4px;
                padding:1px 5px;
                font-size:@font-body;
                font-weight:400;
            }
            QComboBox:hover {
                background-color:@ct-control-bg;
                border-color:@ct-border-active;
                color:@ct-text-main;
            }
            QComboBox::drop-down {
                border:none;
                width:14px;
            }
            QComboBox QAbstractItemView {
                background-color:@ct-control-bg;
                color:@ct-text-main;
                selection-background-color:@ct-selected-bg;
                selection-color:@ct-control-bg;
                border:1px solid @ct-control-border;
                outline:none;
                padding:2px;
            }
        """)
        for code, name in LANGUAGES.items():
            self.combo_lang.addItem(name, code)
        curr_lang = get_current_language()
        idx = self.combo_lang.findData(curr_lang)
        if idx >= 0:
            self.combo_lang.setCurrentIndex(idx)
        self.combo_lang.currentIndexChanged.connect(self.on_language_changed)
        preferences = QHBoxLayout()
        preferences.setSpacing(6)
        self.lbl_language = QLabel(tr("lbl_language"))
        preferences.addWidget(self.lbl_language)
        preferences.addWidget(self.combo_lang)
        self.lbl_theme = QLabel(tr("lbl_theme"))
        preferences.addWidget(self.lbl_theme)
        self.combo_theme = QComboBox()
        for key, label in theme_names(get_current_language()).items():
            self.combo_theme.addItem(label, key)
        self.combo_theme.setCurrentIndex(self.combo_theme.findData(current_theme()))
        self.combo_theme.setMinimumWidth(110)
        self.combo_theme.currentIndexChanged.connect(self.on_theme_changed)
        preferences.addWidget(self.combo_theme, stretch=1)
        left_layout.addLayout(preferences)

        bottom_ctrl_layout.addLayout(playback_bar)
        adjustment_bar = QHBoxLayout()
        adjustment_bar.setSpacing(6)
        adjustment_bar.addStretch()

        self.lbl_brightness_tag = QLabel(tr("lbl_brightness_tag"))
        set_theme_style(self.lbl_brightness_tag, "font-size:@font-body; color:@ct-text-main;")
        self.lbl_brightness_tag.setToolTip(tr("tt_brightness_reset"))
        self.lbl_brightness_tag.setCursor(Qt.CursorShape.PointingHandCursor)
        self.lbl_brightness_tag.installEventFilter(self)
        adjustment_bar.addWidget(self.lbl_brightness_tag)

        self.slider_brightness = QSlider(Qt.Orientation.Horizontal)
        self.slider_brightness.setRange(-100, 100)
        self.slider_brightness.setValue(0)
        self.slider_brightness.setMinimumWidth(55)
        self.slider_brightness.setMaximumWidth(88)
        self.slider_brightness.setCursor(Qt.CursorShape.PointingHandCursor)
        self.slider_brightness.setToolTip(tr("slider_brightness_tooltip"))
        self.slider_brightness.valueChanged.connect(self.sync_preview)
        self.slider_brightness.installEventFilter(self)
        adjustment_bar.addWidget(self.slider_brightness)

        adjustment_bar.addSpacing(4)

        self.lbl_contrast_tag = QLabel(tr("lbl_contrast_tag"))
        set_theme_style(self.lbl_contrast_tag, "font-size:@font-body; color:@ct-text-main;")
        self.lbl_contrast_tag.setToolTip(tr("tt_contrast_reset"))
        self.lbl_contrast_tag.setCursor(Qt.CursorShape.PointingHandCursor)
        self.lbl_contrast_tag.installEventFilter(self)
        adjustment_bar.addWidget(self.lbl_contrast_tag)

        self.slider_contrast = QSlider(Qt.Orientation.Horizontal)
        self.slider_contrast.setRange(50, 200)
        self.slider_contrast.setValue(100)
        self.slider_contrast.setMinimumWidth(55)
        self.slider_contrast.setMaximumWidth(88)
        self.slider_contrast.setCursor(Qt.CursorShape.PointingHandCursor)
        self.slider_contrast.setToolTip(tr("slider_contrast_tooltip"))
        self.slider_contrast.valueChanged.connect(self.sync_preview)
        self.slider_contrast.installEventFilter(self)
        adjustment_bar.addWidget(self.slider_contrast)

        adjustment_bar.addSpacing(4)

        self.btn_hotkeys = QPushButton(tr("btn_hotkeys"))
        self.btn_hotkeys.setToolTip(tr("tt_hotkeys"))
        self.btn_hotkeys.setFixedHeight(28)
        set_theme_style(self.btn_hotkeys, transport_sub_btn_style)
        self.btn_hotkeys.clicked.connect(self.show_hotkey_guide)
        adjustment_bar.addWidget(self.btn_hotkeys)

        bottom_ctrl_layout.addLayout(adjustment_bar)

        right_layout.addWidget(timeline_group)

        # 우측 하단 푸터 (서명 및 버전 정보)
        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(0, 0, 4, 0)
        footer_layout.addStretch()
        self.lbl_footer = QLabel(f"♥ Companion Turret  ·  v{APP_VERSION}")
        set_theme_style(self.lbl_footer, "color:@ct-text-muted; font-size:@font-small; font-weight:normal;")
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
        self.retranslate_ui()

    def _preview_pos_to_canvas(self, pos):
        pm = self.preview_lbl.pixmap()
        if pm is not None and not pm.isNull() and pm.width() > 0 and pm.height() > 0:
            off_x = (self.preview_lbl.width() - pm.width()) / 2.0
            off_y = (self.preview_lbl.height() - pm.height()) / 2.0
            rx = (pos.x() - off_x) / float(pm.width())
            ry = (pos.y() - off_y) / float(pm.height())
        else:
            if self.preview_lbl.width() <= 0 or self.preview_lbl.height() <= 0:
                return None, None
            rx = pos.x() / float(self.preview_lbl.width())
            ry = pos.y() / float(self.preview_lbl.height())

        if rx < 0.0 or rx > 1.0 or ry < 0.0 or ry > 1.0:
            return None, None
        return rx * 1920.0, ry * 1080.0

    def _hit_test_telemetry_col(self, cx, cy):
        if cx is None or cy is None:
            return None
        opts = self.get_current_options()
        col_box_map, _, offset_x, _, front_h, bottom_h, _, _ = OverlayRenderer.compute_telemetry_layout(opts, 1920, 1080)
        if bottom_h <= 0 or cy < front_h:
            return None
        pad = 10.0
        for k, (x1, w) in col_box_map.items():
            abs_x1 = offset_x + x1 - pad
            abs_x2 = offset_x + x1 + w + pad
            if abs_x1 <= cx <= abs_x2:
                return k
        return None

    def _update_telemetry_drag(self, cx):
        drag_k = getattr(self, "_dragging_telemetry_key", None)
        if not drag_k or cx is None:
            return False

        opts = self.get_current_options()
        col_box_map, _, offset_x, _, _, bottom_h, _, _ = OverlayRenderer.compute_telemetry_layout(opts, 1920, 1080)
        if bottom_h <= 0 or drag_k not in col_box_map:
            return False

        active_keys = [k for k in self.telemetry_order if k in col_box_map]
        if drag_k not in active_keys or len(active_keys) < 2:
            return False

        cur_idx = active_keys.index(drag_k)
        target_k = None
        if cur_idx > 0:
            left_k = active_keys[cur_idx - 1]
            lx1, lw = col_box_map[left_k]
            if cx < (offset_x + lx1 + lw * 0.55):
                target_k = left_k
        if target_k is None and cur_idx < len(active_keys) - 1:
            right_k = active_keys[cur_idx + 1]
            rx1, rw = col_box_map[right_k]
            if cx > (offset_x + rx1 + rw * 0.45):
                target_k = right_k

        if target_k is not None:
            i1 = self.telemetry_order.index(drag_k)
            i2 = self.telemetry_order.index(target_k)
            self.telemetry_order[i1], self.telemetry_order[i2] = self.telemetry_order[i2], self.telemetry_order[i1]
            if not (hasattr(self, "timer") and self.timer.isActive()):
                self.sync_preview()
            return True
        return False

    def eventFilter(self, obj, event):
        etype = event.type()
        if obj is getattr(self, "preview_lbl", None) and not self.is_exporting:
            if etype == QEvent.Type.MouseButtonPress and event.button() == Qt.MouseButton.LeftButton:
                pos = event.position().toPoint() if hasattr(event, "position") else event.pos()
                cx, cy = self._preview_pos_to_canvas(pos)
                hit_k = self._hit_test_telemetry_col(cx, cy)
                if hit_k:
                    self._dragging_telemetry_key = hit_k
                    self.preview_lbl.setCursor(Qt.CursorShape.ClosedHandCursor)
                    if not (hasattr(self, "timer") and self.timer.isActive()):
                        self.sync_preview()
                    return True

            elif etype == QEvent.Type.MouseMove:
                pos = event.position().toPoint() if hasattr(event, "position") else event.pos()
                cx, cy = self._preview_pos_to_canvas(pos)
                if getattr(self, "_dragging_telemetry_key", None):
                    self._update_telemetry_drag(cx)
                    return True
                else:
                    hit_k = self._hit_test_telemetry_col(cx, cy)
                    self.preview_lbl.setCursor(
                        Qt.CursorShape.OpenHandCursor if hit_k else Qt.CursorShape.ArrowCursor
                    )

            elif etype == QEvent.Type.MouseButtonRelease and event.button() == Qt.MouseButton.LeftButton:
                if getattr(self, "_dragging_telemetry_key", None):
                    self._dragging_telemetry_key = None
                    pos = event.position().toPoint() if hasattr(event, "position") else event.pos()
                    cx, cy = self._preview_pos_to_canvas(pos)
                    hit_k = self._hit_test_telemetry_col(cx, cy)
                    self.preview_lbl.setCursor(
                        Qt.CursorShape.OpenHandCursor if hit_k else Qt.CursorShape.ArrowCursor
                    )
                    if not (hasattr(self, "timer") and self.timer.isActive()):
                        self.sync_preview()
                    return True

        if etype == QEvent.Type.MouseButtonDblClick:
            if self.is_exporting:
                return False
            if obj is getattr(self, "preview_lbl", None):
                self._dragging_telemetry_key = None
                pos = event.position().toPoint() if hasattr(event, "position") else event.pos()
                self._handle_preview_double_click(pos)
                return True
            if obj in (getattr(self, "slider_brightness", None), getattr(self, "lbl_brightness_tag", None)):
                self.slider_brightness.setValue(0)
                return True
            if obj in (getattr(self, "slider_contrast", None), getattr(self, "lbl_contrast_tag", None)):
                self.slider_contrast.setValue(100)
                return True
            if obj in (getattr(self, "slider_preview_speed", None), getattr(self, "lbl_spd_tag", None), getattr(self, "lbl_preview_speed_val", None)):
                self.slider_preview_speed.setValue(2)
                return True
        return super().eventFilter(obj, event)

    def _handle_preview_double_click(self, pos):
        """미리보기 화면 더블클릭 시 해당 슬롯 카메라 단독 뷰 전환 또는 이전 분할 뷰로 복귀."""
        if self.is_exporting:
            return

        cx, cy = self._preview_pos_to_canvas(pos)
        if cx is None or cy is None:
            return

        opts = self.get_current_options()
        has_bottom_hud = any(
            opts.get(k) for k in ("map", "timestamp", "speed", "fsd", "turn_signal", "steering", "pedal")
        )
        front_h = 1080.0 - (180.0 if has_bottom_hud else 0.0)
        if cy >= front_h:
            if self.telemetry_order != list(OverlayRenderer.DEFAULT_TELEMETRY_ORDER):
                self.telemetry_order = list(OverlayRenderer.DEFAULT_TELEMETRY_ORDER)
                self.sync_preview()
            return

        is_single = self.current_layout_mode in [
            "단일", "단일 카메라", "단일 카메라 (풀스크린)", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"
        ]
        if is_single:
            prev_mode = getattr(self, "_prev_multi_layout_mode", "기본 (1:3 세로배치)")
            prev_pip = getattr(self, "_prev_pillar_pip_checked", None)
            self.set_layout_mode(prev_mode)
            if prev_pip is not None and self.chks["pillar_pip"].isEnabled():
                self.chks["pillar_pip"].setChecked(prev_pip)
            return

        target_cam = None
        is_2x2 = self.current_layout_mode in ["2x2 분할 (전후/좌우)", "2x2"]
        if opts.get("pillar_pip", False):
            pw = 236.0 if is_2x2 else 320.0
            ph = 132.0 if is_2x2 else 180.0
            lp_x, lp_y = 8.0, front_h - ph - 8.0
            rp_x = (1920.0 if is_2x2 else 1440.0) - pw - 8.0
            rp_y = front_h - ph - 8.0
            if lp_x <= cx <= lp_x + pw and lp_y <= cy <= lp_y + ph:
                target_cam = "left_pillar"
            elif rp_x <= cx <= rp_x + pw and rp_y <= cy <= rp_y + ph:
                target_cam = "right_pillar"

        if target_cam is None:
            if is_2x2:
                if cx < 960.0:
                    target_cam = "front" if cy < (front_h / 2.0) else "back"
                else:
                    target_cam = "left_repeater" if cy < (front_h / 2.0) else "right_repeater"
            else:
                if cx < 1440.0:
                    target_cam = "front"
                else:
                    slot_idx = min(2, max(0, int(cy / (front_h / 3.0))))
                    target_cam = ["back", "left_repeater", "right_repeater"][slot_idx]

        if self.active_clip_list:
            avail_cams = self.active_clip_list[0].get("cams", {})
            if target_cam not in avail_cams:
                return

        self._prev_multi_layout_mode = self.current_layout_mode
        self._prev_pillar_pip_checked = self.chks["pillar_pip"].isChecked()
        self.current_single_cam = target_cam
        self.set_layout_mode("단일")

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
        # Compatibility entry point; all palettes share the application geometry.
        self.setProperty("ctStyleTemplate", None)
        self.setStyleSheet("")
        apply_theme(current_theme(), persist=False)

    def on_theme_changed(self, index):
        apply_theme(self.combo_theme.currentData())
        self._retranslate_tree()
        self.update_range_highlight()
        self.slider.update()

    def _refresh_theme_names(self):
        self.lbl_theme.setText(tr("lbl_theme"))
        self.combo_theme.setToolTip(tr("lbl_theme"))
        for index, (key, label) in enumerate(theme_names(get_current_language()).items()):
            self.combo_theme.setItemText(index, label)

    def show_toast(self, message: str, duration_ms: int = 5500, icon: str = ""):
        """화면 상단 중앙에 플로팅 토스트 알림을 띄웁니다."""
        if self.toast_widget is None:
            self.toast_widget = ToastNotification(self)
        self.toast_widget.show_message(message, duration_ms=duration_ms, icon=icon)

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dropEvent(self, event):
        if event.mimeData().hasUrls():
            urls = event.mimeData().urls()
            paths = [u.toLocalFile() for u in urls if u.isLocalFile()]
            if paths:
                self.handle_dropped_paths(paths)
            event.acceptProposedAction()
        else:
            super().dropEvent(event)

    def handle_dropped_paths(self, paths):
        """드래그 앤 드롭된 파일/폴더 경로 처리"""
        has_encrypted = any("encryptedclips" in p.lower() for p in paths)
        if has_encrypted:
            self.show_toast(tr("encrypted_clips_toast"))

        valid_paths = [p for p in paths if "encryptedclips" not in p.lower()]
        if not valid_paths:
            return

        self.motion_cache.clear()

        target_dir = None
        for p in valid_paths:
            if os.path.isdir(p):
                target_dir = p
                break

        if not target_dir:
            first_file = valid_paths[0]
            parent = os.path.dirname(first_file)
            grandparent = os.path.dirname(parent)
            if grandparent and any(k in os.path.basename(parent).lower() for k in ["saved", "sentry", "recent", "clips"]):
                target_dir = grandparent
            elif parent:
                target_dir = parent

        if target_dir and os.path.exists(target_dir):
            self.scan_and_populate_tree(target_dir)

    def on_language_changed(self, index):
        lang_code = self.combo_lang.currentData()
        if lang_code:
            set_language(lang_code)
            self.retranslate_ui()

    def _create_group_header_widget(self, title, count_text="", group_type="recent"):
        widget = QWidget()
        widget.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        set_theme_style(widget, "background:transparent;")
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(4, 2, 8, 2)
        layout.setSpacing(8)

        marker = QFrame()
        marker.setFixedSize(3, 16)
        set_theme_style(marker, f"background: @ct-clip-{group_type}; border: none; border-radius: 1px;")
        layout.addWidget(marker)
        lbl_title = QLabel(title)
        set_theme_style(lbl_title, f"color: @ct-clip-{group_type};  font-size:@font-body; font-weight:600; background:transparent;")
        layout.addWidget(lbl_title)

        if count_text:
            lbl_count = QLabel(count_text)
            set_theme_style(lbl_count, "color:@ct-text-muted; font-size:@font-small; font-weight:normal; background:transparent;")
            layout.addWidget(lbl_count)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Plain)
        set_theme_style(line, "background-color:@ct-control-bg; border:none; min-height:1px; max-height:1px;")
        layout.addWidget(line, stretch=1)

        return widget

    def _retranslate_tree(self):
        """트리 탐색기에 이미 로드된 대그룹 헤더 및 하위 항목들을 현재 언어로 갱신"""
        root = self.tree_explorer.invisibleRootItem()
        for i in range(root.childCount()):
            item = root.child(i)
            cdata = item.data(0, Qt.ItemDataRole.UserRole) or {}
            if cdata.get("is_big_group"):
                gtype = cdata.get("group_type")
                counts = cdata.get("counts", (0, 0))
                if gtype == "recent":
                    title = tr("tree_recent_title")
                    count_text = tr("tree_recent_summary", chunks=counts[0], clips=counts[1])
                elif gtype == "saved":
                    title = tr("tree_saved_title")
                    count_text = tr("tree_saved_summary", events=counts[0], clips=counts[1])
                elif gtype == "sentry":
                    title = tr("tree_sentry_title")
                    count_text = tr("tree_sentry_summary", sessions=counts[0], clips=counts[1])
                else:
                    continue
                self.tree_explorer.setItemWidget(item, 0, self._create_group_header_widget(title, count_text, gtype))

                for j in range(item.childCount()):
                    sub = item.child(j)
                    sdata = sub.data(0, Qt.ItemDataRole.UserRole) or {}
                    if sdata.get("is_virtual_dir"):
                        c_list = sdata.get("clip_list", [])
                        sub.setText(1, tr("tree_n_clips", clips=len(c_list)))
                        sub.setToolTip(0, tr("clip_start_range") + "\n" + group_title(c_list))
                        for k in range(sub.childCount()):
                            clip_item = sub.child(k)
                            c_info = clip_item.data(0, Qt.ItemDataRole.UserRole) or {}
                            ch_cnt = len(c_info.get("clip_list", [{}])[0].get("cams", {}))
                            clip_item.setText(1, tr("tree_recent_tag", channels=ch_cnt))
                    elif sdata.get("is_group"):
                        c_list = sdata.get("clip_list", [])
                        sub.setText(1, tr("tree_n_merged", clips=len(c_list)))
                        sub.setToolTip(0, tr("clip_start_range") + "\n" + c_list[0].get("dirpath", ""))

    def retranslate_ui(self):
        """다국어 변경 시 프로그램 전체 UI 텍스트를 실시간 일괄 갱신"""
        self.setWindowTitle(tr("app_title"))
        self._refresh_theme_names()
        self.lbl_language.setText(tr("lbl_language"))
        if not self.active_clip_list:
            self.preview_lbl.setText(tr("preview_empty"))
        self.btn_load.setText(tr("btn_load"))
        self.tree_explorer.setHeaderLabels([tr("tree_col_explorer"), tr("tree_col_type")])

        if hasattr(self, 'opt_group'):
            self.opt_group.setTitle(tr("grp_overlays"))
        chk_trans_map = {
            "timestamp": "chk_timestamp",
            "speed": "chk_speed",
            "fsd": "chk_fsd",
            "turn_signal": "chk_turn_signal",
            "steering": "chk_steering_wheel",
            "pedal": "chk_pedals",
            "pillar_pip": "chk_pillar_pip",
            "map": "chk_osm_minimap",
            "blur_face": "chk_blur_faces",
            "blur_plate": "chk_blur_plates",
            "enhance_plate": "chk_enhance_plates"
        }
        for k, tr_k in chk_trans_map.items():
            if k in self.chks:
                self.chks[k].setText(tr(tr_k))

        if hasattr(self, 'privacy_group'):
            self.privacy_group.setTitle(tr("grp_privacy"))

        if hasattr(self, 'exp_group'):
            self.exp_group.setTitle(tr("grp_export"))
        if hasattr(self, 'lbl_export_res'):
            self.lbl_export_res.setText(tr("lbl_res"))
        if hasattr(self, 'combo_res'):
            curr_res_id = self.combo_res.currentData() or "qhd"
            self.combo_res.blockSignals(True)
            self.combo_res.clear()
            for res_id, tr_k, _, _ in RESOLUTION_CHOICES:
                self.combo_res.addItem(tr(tr_k), res_id)
            idx = self.combo_res.findData(curr_res_id)
            if idx >= 0:
                self.combo_res.setCurrentIndex(idx)
            self.combo_res.blockSignals(False)

        if hasattr(self, 'lbl_export_layout'):
            self.lbl_export_layout.setText(tr("lbl_layout"))
        if hasattr(self, 'lbl_export_speed'):
            self.lbl_export_speed.setText(tr("lbl_speed"))
        if hasattr(self, 'combo_export_speed'):
            curr_spd_val = self.combo_export_speed.currentData()
            if curr_spd_val is None:
                curr_spd_val = 1.0
            self.combo_export_speed.blockSignals(True)
            self.combo_export_speed.clear()
            for spd_val, tr_k in EXPORT_SPEED_CHOICES:
                self.combo_export_speed.addItem(tr(tr_k), spd_val)
            idx = self.combo_export_speed.findData(curr_spd_val)
            if idx >= 0:
                self.combo_export_speed.setCurrentIndex(idx)
            self.combo_export_speed.blockSignals(False)

        if hasattr(self, 'lbl_export_fps'):
            self.lbl_export_fps.setText(tr("lbl_fps"))
        if not getattr(self, 'is_exporting', False):
            self.btn_export.setText(tr("btn_export"))
        self.btn_qr_share.setText(f"{tr('chk_qr_share')}")

        self.btn_layout_1to3.setText(f"{tr('layout_1to3')}")
        self.btn_layout_1to3.setToolTip(tr("layout_1to3_tooltip"))
        self.btn_layout_2x2.setText(tr("layout_2x2"))
        self.btn_layout_2x2.setToolTip(tr("layout_2x2_tooltip"))
        self.btn_layout_single.setText(f"{tr('layout_single')}")
        self.btn_layout_single.setToolTip(tr("layout_single_tooltip"))

        self.btn_prev.setText(f"|◀ {tr('btn_prev')}")
        self.btn_prev.setToolTip(tr("tt_btn_prev"))
        self.btn_play.setText(tr("btn_pause") if getattr(self, 'timer', None) and self.timer.isActive() else tr("btn_play"))
        self.btn_play.setToolTip(tr("tt_btn_play"))
        self.btn_next.setText(f"{tr('btn_next')} ▶|")
        self.btn_next.setToolTip(tr("tt_btn_next"))
        self.chk_motion.setText(tr("chk_motion"))
        if hasattr(self, 'lbl_sensitivity_tag'):
            self.lbl_sensitivity_tag.setText(tr("lbl_sensitivity"))

        if hasattr(self, 'combo_sensitivity'):
            curr_sens = self.combo_sensitivity.currentData() or "보통"
            self.combo_sensitivity.blockSignals(True)
            self.combo_sensitivity.clear()
            for raw_k, tr_k in SENSITIVITY_CHOICES:
                self.combo_sensitivity.addItem(tr(tr_k), raw_k)
            idx = self.combo_sensitivity.findData(curr_sens)
            if idx >= 0:
                self.combo_sensitivity.setCurrentIndex(idx)
            self.combo_sensitivity.blockSignals(False)

        self.btn_in.setText(tr("btn_in"))
        self.btn_in.setToolTip(tr("btn_in_tooltip"))
        self.btn_out.setText(tr("btn_out"))
        self.btn_out.setToolTip(tr("btn_out_tooltip"))
        self.btn_reset_range.setText(tr("btn_reset_range"))
        self.btn_reset_range.setToolTip(tr("btn_reset_tooltip"))

        self.lbl_spd_tag.setText(tr("lbl_speed_tag"))
        self.lbl_spd_tag.setToolTip(tr("tt_speed_reset"))
        if hasattr(self, 'slider_preview_speed'):
            self.slider_preview_speed.setToolTip(tr("tt_speed_slider"))
        if hasattr(self, 'lbl_preview_speed_val'):
            self.lbl_preview_speed_val.setToolTip(tr("tt_speed_reset"))

        self.chk_auto_next.setText(tr("chk_auto_next"))
        self.chk_auto_next.setToolTip(tr("chk_auto_next_tooltip"))
        self.lbl_brightness_tag.setText(tr("lbl_brightness_tag"))
        self.lbl_brightness_tag.setToolTip(tr("tt_brightness_reset"))
        self.slider_brightness.setToolTip(tr("slider_brightness_tooltip"))
        self.lbl_contrast_tag.setText(tr("lbl_contrast_tag"))
        self.lbl_contrast_tag.setToolTip(tr("tt_contrast_reset"))
        self.slider_contrast.setToolTip(tr("slider_contrast_tooltip"))
        self.btn_hotkeys.setText(tr("btn_hotkeys"))
        self.btn_hotkeys.setToolTip(tr("tt_hotkeys"))

        if hasattr(self, 'preview_lbl'):
            self.preview_lbl.setToolTip(tr("tt_preview_screen"))
        if "enhance_plate" in self.chks:
            self.chks["enhance_plate"].setToolTip(tr("tt_enhance_plate"))

        if hasattr(self, 'btn_prev_event'):
            self.btn_prev_event.setText(tr("btn_prev_event"))
            self.btn_prev_event.setToolTip(tr("tt_motion_prev"))
        if hasattr(self, 'btn_next_event'):
            self.btn_next_event.setText(tr("btn_next_event"))
            self.btn_next_event.setToolTip(tr("tt_motion_next"))
        if hasattr(self, 'single_cam_popup') and self.single_cam_popup:
            self.single_cam_popup.retranslate_ui()
        if not getattr(self, 'active_clip_list', None) and hasattr(self, 'lbl_motion_status'):
            self.lbl_motion_status.setText(tr("motion_ready"))

        for label in (self.lbl_export_res, self.lbl_export_layout, self.lbl_export_speed, self.lbl_export_fps):
            label.ensurePolished()
            label.setMinimumWidth(label.fontMetrics().horizontalAdvance(label.text()) + 4)
        self.combo_export_speed.ensurePolished()
        speed_width = max(self.combo_export_speed.fontMetrics().horizontalAdvance(self.combo_export_speed.itemText(i))
                          for i in range(self.combo_export_speed.count()))
        self.combo_export_speed.setMinimumWidth(speed_width + 32)
        self.left_panel.setMinimumWidth(max(370, self.exp_group.minimumSizeHint().width()))
        self.update_range_highlight()
        self.update_estimated_size()
        self._retranslate_tree()

    def load_directory_dialog(self):
        default_dir = get_app_dir()
        folder = QFileDialog.getExistingDirectory(self, tr("btn_load"), default_dir)
        if not folder:
            return

        self.motion_cache.clear()
        self.scan_and_populate_tree(folder)

    def scan_and_populate_tree(self, root_path):
        self.tree_explorer.setUpdatesEnabled(False)
        try:
            self._scan_and_populate_tree(root_path)
        finally:
            self.tree_explorer.setUpdatesEnabled(True)

    def _scan_and_populate_tree(self, root_path):
        self.tree_explorer.clear()
        self.reset_range_points()
        self.slider.set_event_blocks([])

        has_encrypted_detected = False

        # 1. 루트 경로 자체에 EncryptedClips가 포함되어 있는지 확인
        norm_root = os.path.normpath(str(root_path)).lower()
        if "encryptedclips" in norm_root:
            has_encrypted_detected = True
            self.show_toast(tr("encrypted_clips_toast"))

        cam_suffixes = {
            "front": ["-front.mp4"],
            "back": ["-back.mp4"],
            "left_repeater": ["-left_repeater.mp4"],
            "right_repeater": ["-right_repeater.mp4"],
            "left_pillar": ["-left_pillar.mp4", "-front_left.mp4", "-left_b_pillar.mp4"],
            "right_pillar": ["-right_pillar.mp4", "-front_right.mp4", "-right_b_pillar.mp4"]
        }

        tree_data = {}

        # 루트 경로 자체가 EncryptedClips인 경우 내부 파일은 모두 암호화되어 있으므로 스캔 제외
        if not ("encryptedclips" in norm_root):
            for dirpath, dirs, filenames in os.walk(root_path):
                # 하위 디렉토리 중 EncryptedClips 발견 시 즉시 제외 및 알림
                enc_dirs = [d for d in dirs if "encryptedclips" in d.lower()]
                if enc_dirs:
                    if not has_encrypted_detected:
                        has_encrypted_detected = True
                        self.show_toast(tr("encrypted_clips_toast"))
                    for d in enc_dirs:
                        dirs.remove(d)

                # 현재 탐색 디렉토리 경로에 EncryptedClips가 포함되어 있으면 제외
                if "encryptedclips" in dirpath.lower():
                    if not has_encrypted_detected:
                        has_encrypted_detected = True
                        self.show_toast(tr("encrypted_clips_toast"))
                    continue

                # 파일 목록 중 EncryptedClips 포함 파일 감지 및 제외
                enc_files = [f for f in filenames if "encryptedclips" in f.lower()]
                if enc_files and not has_encrypted_detected:
                    has_encrypted_detected = True
                    self.show_toast(tr("encrypted_clips_toast"))

                front_files = [
                    f for f in filenames
                    if f.endswith("-front.mp4") and "encryptedclips" not in f.lower()
                ]
                if not front_files:
                    continue

                front_files.sort()
                rel = os.path.relpath(dirpath, root_path)
                parts = rel.split(os.sep) if rel != "." else [os.path.basename(dirpath)]

                category = parts[0] if (parts and parts[0] != ".") else "TeslaCam"
                event_name = os.path.normpath(dirpath)

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
            if not has_encrypted_detected:
                QMessageBox.warning(self, tr("warning_title"), tr("no_clips_warning"))
            return

        brush_white = QBrush(QColor(color("text-main")))

        def create_big_group_item(title, count_text, group_type, counts=(0, 0)):
            # QTreeWidget의 기본 텍스트와 setItemWidget 텍스트가 겹치지 않도록 빈 문자열 전달
            group_item = QTreeWidgetItem(self.tree_explorer, ["", ""])
            group_item.setFirstColumnSpanned(True)
            group_item.setSizeHint(0, QSize(100, 30))
            group_item.setData(0, Qt.ItemDataRole.UserRole, {
                "is_big_group": True,
                "group_type": group_type,
                "counts": counts
            })
            group_item.setExpanded(True)
            self.tree_explorer.setItemWidget(group_item, 0, self._create_group_header_widget(title, count_text, group_type))
            return group_item

        # 전체 이벤트를 최근클립, 저장된클립, 센트리 3대 대그룹으로 분류
        all_recent_events = []
        all_saved_events = []
        all_sentry_events = []

        for cat_name, events in tree_data.items():
            for ev_name, clips in events.items():
                is_sentry = any(c["is_sentry"] for c in clips)
                is_recent = any(c["is_recent"] for c in clips)

                if is_sentry and not is_recent:
                    all_sentry_events.append((ev_name, clips))
                elif is_recent:
                    all_recent_events.append((ev_name, clips))
                else:
                    all_saved_events.append((ev_name, clips))

        # 최근 클립: 180초 세션 간격, 10개 단위 가상 그룹
        if all_recent_events:
            # 전체 최근 클립 청크 분할
            all_recent_clips = []
            for ev_name, clips in all_recent_events:
                all_recent_clips.extend(clips)
            all_recent_clips.sort(key=clip_datetime)

            recent_chunks = group_recent_clips(all_recent_clips)

            recent_header = create_big_group_item(
                tr("tree_recent_title"),
                tr("tree_recent_summary", chunks=len(recent_chunks), clips=len(all_recent_clips)),
                "recent",
                (len(recent_chunks), len(all_recent_clips))
            )

            for chunk in recent_chunks:
                vdir_title = group_title(chunk)

                vdir_item = QTreeWidgetItem(recent_header, [vdir_title, tr("tree_n_clips", clips=len(chunk))])
                vdir_item.setForeground(0, brush_white)
                vdir_item.setForeground(1, brush_white)
                vdir_item.setToolTip(0, tr("clip_start_range") + "\n" + vdir_title)
                vdir_item.setExpanded(False)
                vdir_item.setData(0, Qt.ItemDataRole.UserRole, {
                    "is_virtual_dir": True,
                    "clip_type": "recent",
                    "clip_list": chunk
                })

                for clip in chunk:
                    ch_count = len(clip["cams"])
                    clip_item = QTreeWidgetItem(vdir_item, [clip["display"].split(" ")[-1], tr("tree_recent_tag", channels=ch_count)])
                    clip_item.setToolTip(0, clip["cams"]["front"])
                    clip_item.setForeground(0, brush_white)
                    clip_item.setForeground(1, brush_white)
                    clip_item.setData(0, Qt.ItemDataRole.UserRole, {
                        "is_group": False,
                        "is_sentry": False,
                        "is_recent": True,
                        "clip_type": "recent",
                        "clip_list": [clip]
                    })

        # 저장된 클립: 폴더는 탐색용, 영상은 개별 클립으로 로드
        if all_saved_events:
            total_saved_clips = sum(len(clips) for _, clips in all_saved_events)
            saved_header = create_big_group_item(
                tr("tree_saved_title"),
                tr("tree_saved_summary", events=len(all_saved_events), clips=total_saved_clips),
                "saved",
                (len(all_saved_events), total_saved_clips)
            )

            for ev_name, clips in sorted(all_saved_events, key=lambda event: clip_datetime(event[1][0])):
                clips = sorted(clips, key=clip_datetime)
                ev_item = QTreeWidgetItem(saved_header, [group_title(clips), tr("tree_n_clips", clips=len(clips))])
                ev_item.setToolTip(0, tr("clip_start_range") + "\n" + ev_name)
                ev_item.setForeground(0, brush_white)
                ev_item.setForeground(1, brush_white)
                ev_item.setData(0, Qt.ItemDataRole.UserRole, {
                    "is_group": False,
                    "is_virtual_dir": True,
                    "is_sentry": False,
                    "is_recent": False,
                    "is_saved": True,
                    "clip_type": "saved",
                    "clip_list": clips
                })

                ev_item.setExpanded(False)
                for clip in clips:
                    clip_item = QTreeWidgetItem(ev_item, [
                        clip["display"].split(" ")[-1],
                        tr("tree_recent_tag", channels=len(clip["cams"])),
                    ])
                    clip_item.setToolTip(0, clip["cams"]["front"])
                    clip_item.setData(0, Qt.ItemDataRole.UserRole, {
                        "is_group": False,
                        "is_sentry": False,
                        "is_recent": False,
                        "is_saved": True,
                        "clip_type": "saved",
                        "clip_list": [clip],
                    })

        # 센트리 이벤트의 통합 재생 유지
        if all_sentry_events:
            total_sentry_clips = sum(len(clips) for _, clips in all_sentry_events)
            sentry_header = create_big_group_item(
                tr("tree_sentry_title"),
                tr("tree_sentry_summary", sessions=len(all_sentry_events), clips=total_sentry_clips),
                "sentry",
                (len(all_sentry_events), total_sentry_clips)
            )

            for ev_name, clips in sorted(all_sentry_events, key=lambda event: clip_datetime(event[1][0])):
                clips = sorted(clips, key=clip_datetime)
                ev_item = QTreeWidgetItem(sentry_header, [group_title(clips), tr("tree_n_merged", clips=len(clips))])
                ev_item.setToolTip(0, tr("clip_start_range") + "\n" + ev_name)
                ev_item.setForeground(0, brush_white)
                ev_item.setForeground(1, brush_white)
                ev_item.setData(0, Qt.ItemDataRole.UserRole, {
                    "is_group": True,
                    "is_sentry": True,
                    "clip_type": "sentry",
                    "clip_list": clips
                })

        self._clear_tree_range_highlights()

    def _refresh_playing_item(self):
        """Playback marker is independent of focus selection and export range."""
        iterator = QTreeWidgetItemIterator(self.tree_explorer)
        while iterator.value():
            row = iterator.value()
            data = row.data(0, Qt.ItemDataRole.UserRole) or {}
            if data.get("clip_list") and not data.get("is_virtual_dir"):
                title = row.data(0, Qt.ItemDataRole.UserRole + 1)
                if title is None:
                    title = row.text(0)
                    row.setData(0, Qt.ItemDataRole.UserRole + 1, title)
                playing = row is getattr(self, "current_item", None)
                row.setText(0, ("▶ " if playing else "") + title)
                font = self.tree_explorer.font()
                font.setBold(playing)
                row.setFont(0, font)
            iterator += 1

    def get_current_clip_key(self):
        if not self.active_clip_list:
            return None
        return tuple(clip_identity(clip) for clip in self.active_clip_list)

    def on_tree_item_clicked(self, item, column):
        if self.is_exporting:
            return
        item_data = item.data(0, Qt.ItemDataRole.UserRole)
        # 하위 클립을 가진 폴더(최근 클립 가상 디렉토리 또는 이벤트 폴더 등) 클릭 시: 단순 트리 접기/펼치기만 수행
        if not item_data or "clip_list" not in item_data or item_data.get("is_virtual_dir", False):
            if item.childCount() > 0:
                item.setExpanded(not item.isExpanded())
            return

        self.stop_motion_scanner()

        if not getattr(self, 'was_playing_before_load', False):
            self.was_playing_before_load = self.timer.isActive()
        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText(tr("btn_play"))
            set_theme_style(self.btn_play, self._play_btn_idle_style)


        self.current_item = item
        self._refresh_playing_item()
        self.active_clip_list = item_data["clip_list"]
        if not self.active_clip_list:
            return

        if hasattr(self, 'loader_worker') and self.loader_worker and self.loader_worker.isRunning():
            self.loader_worker.stop()
            self.loader_worker.wait()

        self.is_load_canceled = False
        self.progress_dialog = QProgressDialog(tr("loading_dialog_msg"), tr("btn_cancel"), 0, 100, self)
        self.progress_dialog.setWindowTitle(tr("loading_dialog_title"))
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

    def _set_motion_badge_style(self, mode="idle"):
        if mode == "detected":
            set_theme_style(self.lbl_motion_status, """
                QLabel {
                    background-color:@ct-selected-bg;
                    color:@ct-danger-text;
                    border:1px solid @ct-selected-bg;
                    border-radius:5px;
                    padding:2px 9px;
                    font-size:@font-body;
                    font-weight:400;
                }
            """)
        elif mode == "scanning":
            set_theme_style(self.lbl_motion_status, """
                QLabel {
                    background-color:@ct-selected-bg;
                    color:@ct-text-main;
                    border:1px solid @ct-selected-bg;
                    border-radius:5px;
                    padding:2px 9px;
                    font-size:@font-body;
                    font-weight:400;
                }
            """)
        else:
            set_theme_style(self.lbl_motion_status, """
                QLabel {
                    background-color:@ct-control-bg;
                    color:@ct-text-main;
                    border:1px solid @ct-control-border;
                    border-radius:5px;
                    padding:2px 9px;
                    font-size:@font-body;
                    font-weight:400;
                }
            """)

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

            # 백그라운드에서 주행 경로의 OSM 미니맵 타일 선행 캐싱
            OpenStreetMapTileLoader.prefetch_for_decoders(decoders)

            # 주차/센트리 및 주행 영상 모두에서 단일 카메라 모드 완벽 지원
            self.btn_layout_single.setEnabled(True)
            self.btn_layout_single.setToolTip(tr("layout_single_tooltip"))

            # 현재 선택된 카메라가 새 클립에 없을 경우 기본값(front)으로 안전 복귀
            if self.active_clip_list and len(self.active_clip_list) > 0:
                cams = self.active_clip_list[0].get("cams", {})
                if getattr(self, "current_single_cam", "front") not in cams:
                    self.current_single_cam = "front"
            if hasattr(self, 'single_cam_popup') and self.single_cam_popup and self.single_cam_popup.isVisible():
                avail = set(self.active_clip_list[0].get("cams", {}).keys()) if self.active_clip_list else set()
                self.single_cam_popup.update_states(avail, getattr(self, "current_single_cam", "front"))

            match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', self.active_clip_list[0]["prefix"])
            if match:
                self.base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S")

            has_pillars = ('left_pillar' in self.active_clip_list[0]["cams"]) or ('right_pillar' in self.active_clip_list[0]["cams"])
            self.chks["pillar_pip"].blockSignals(True)
            if self.current_layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1", "단일", "단일 카메라", "단일 카메라 (풀스크린)"]:
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
            self.last_raw_preview_frames.clear()
            self.last_preview_slot_size.clear()
            self.last_preview_cam_frame_idx = {}
            self.current_active_clip_idx = -1

            if detected_fps is not None:
                self.fps = detected_fps
            else:
                self.fps = 24.0 if has_pillars else 36.0

            self.play_start_wall_time = None
            self.play_start_frame = 0

            self.slider.blockSignals(True)
            self.slider.set_fps(self.fps)
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
                        self._set_motion_badge_style("detected" if cached_blocks else "idle")
                        self.lbl_motion_status.setText(tr("motion_detected", count=len(cached_blocks)))
                    else:
                        self.start_light_motion_scanner(self.active_clip_list)
                else:
                    self.btn_prev_event.setVisible(False)
                    self.btn_next_event.setVisible(False)
                    self._set_motion_badge_style("idle")
                    self.lbl_motion_status.setText(tr("motion_ready"))
            else:
                self.chk_motion.setVisible(False)
                self.lbl_sensitivity_tag.setVisible(False)
                self.combo_sensitivity.setVisible(False)
                self.btn_prev_event.setVisible(False)
                self.btn_next_event.setVisible(False)
                self.detected_event_blocks = []
                self.slider.set_event_blocks([])
                self._set_motion_badge_style("idle")
                self.lbl_motion_status.setText(tr("motion_driving"))

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
        self.cam_fps = {}
        self.cam_fc = {}
        self.current_cap_pos = {}
        self.last_preview_cam_frame_idx = {}
        for k, cap in self.caps.items():
            v_fps = cap.get(cv2.CAP_PROP_FPS)
            self.cam_fps[k] = v_fps if v_fps and 10.0 <= v_fps <= 120.0 else self.fps
            self.cam_fc[k] = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 2160
            self.current_cap_pos[k] = 0

        # 이전 클립에는 있었으나 현재 클립에 없는 카메라는 프리뷰 캐시에서 제거 (잔상/동결 방지)
        stale_keys = [k for k in list(self.last_valid_preview_frames.keys()) if k not in self.caps]
        for k in stale_keys:
            del self.last_valid_preview_frames[k]
            self.last_raw_preview_frames.pop(k, None)
            self.last_preview_slot_size.pop(k, None)

        self.decoder = self.active_decoders[clip_idx] if clip_idx < len(self.active_decoders) else None
        self.current_active_clip_idx = clip_idx

        # 현재 활성 클립의 base_time 보관 (HUD 시간 렌더링용, self.base_time은 세션 시작 시간으로 유지)
        match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', clip_info.get("prefix", ""))
        if match:
            self.current_clip_base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S")

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
                    self._set_motion_badge_style("detected" if cached_blocks else "idle")
                    self.lbl_motion_status.setText(tr("motion_detected", count=len(cached_blocks)))
                else:
                    self.start_light_motion_scanner(self.active_clip_list)
            else:
                self.btn_prev_event.setVisible(False)
                self.btn_next_event.setVisible(False)
                self.detected_event_blocks = []
                self.slider.set_event_blocks([])
                self.stop_motion_scanner()
                self._set_motion_badge_style("idle")
                self.lbl_motion_status.setText(tr("motion_off"))

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

        self._set_motion_badge_style("scanning")
        self.lbl_motion_status.setText(tr("motion_scanning"))
        self.detected_event_blocks = []

        sensitivity_preset = self.combo_sensitivity.currentData() or "보통"
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

        self._set_motion_badge_style("detected" if event_blocks else "idle")
        self.lbl_motion_status.setText(tr("motion_detected", count=len(event_blocks)))

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

        # 모든 재생 가능한 개별/그룹 클립 아이템 수집
        all_clips = []
        it = QTreeWidgetItemIterator(self.tree_explorer)
        while it.value():
            item = it.value()
            cdata = item.data(0, Qt.ItemDataRole.UserRole) or {}
            if "clip_list" in cdata and not cdata.get("is_virtual_dir", False):
                all_clips.append(item)
            it += 1

        if curr_item in all_clips:
            idx = all_clips.index(curr_item)
            target_idx = idx + direction
            if 0 <= target_idx < len(all_clips):
                return all_clips[target_idx]
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
        self.reset_preview_privacy()
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
        self.reset_preview_privacy()
        idx = self.slider.value()
        # 동일 프레임에서 옵션(얼굴/번호판 모자이크, 선명화, 밝기 등)만 변경될 때는
        # VideoCapture 재탐색(cap.set)을 하지 않고 이미 디코딩된 클린 프레임 캐시를 재사용하여
        # 가변 프레임레이트(VFR) 리피터 영상의 프레임 튐(Jump) 현상을 원천 차단합니다.
        frames = self.read_frames_with_cache(idx, is_seeking=False, reuse_same_frame=True)
        self.render_and_display(frames, idx)

    def read_frames_with_cache(self, global_idx, is_seeking=False, skip_count=0, reuse_same_frame=False):
        if not hasattr(self, 'last_preview_cam_frame_idx'):
            self.last_preview_cam_frame_idx = {}
        if not hasattr(self, 'last_raw_preview_frames'):
            self.last_raw_preview_frames = {}
        if not hasattr(self, 'last_preview_slot_size'):
            self.last_preview_slot_size = {}

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

        if is_seeking:
            self.reset_preview_privacy()

        front_fps = getattr(self, 'cam_fps', {}).get('front', self.fps if self.fps > 0 else 36.0)
        t_sec = local_idx / front_fps if front_fps > 0 else 0.0

        # 현재 활성 레이아웃에 실제로 표시되는 카메라만 디코딩 및 AI 처리 (불필요한 연산 차단)
        layout_mode = getattr(self, 'current_layout_mode', "기본 (1:3 세로배치)")
        is_single = layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1", "단일", "단일 카메라", "단일 카메라 (풀스크린)"]
        is_2x2 = layout_mode in ["2x2 분할 (전후/좌우)", "2x2"]

        if is_single:
            active_cam_keys = {self.get_current_single_cam()}
        else:
            active_cam_keys = {'front', 'back', 'left_repeater', 'right_repeater'}
            if self.chks.get("pillar_pip") and self.chks["pillar_pip"].isChecked():
                active_cam_keys.update({'left_pillar', 'right_pillar'})

        # 미리보기 화면에서 메인 화면을 제외한 서브 카메라들은 실제 슬롯 크기로 즉시 축소하여
        # 저사양 PC에서 후속 연산(밝기/대비, AI 블러, 렌더러 리사이즈, 메모리 복사) 부하를 대폭 절감
        has_bottom_hud = any(
            self.chks.get(hk) and self.chks[hk].isChecked()
            for hk in ("map", "timestamp", "speed", "fsd", "turn_signal", "steering", "pedal")
        )
        front_h = 900 if has_bottom_hud else 1080
        sub_h = front_h // 3
        half_h = front_h // 2

        if is_single:
            single_cam = self.get_current_single_cam()
            slot_sizes = {single_cam: None}
        elif is_2x2:
            slot_sizes = {
                'front': (960, half_h),
                'back': (960, front_h - half_h),
                'left_repeater': (960, half_h),
                'right_repeater': (960, front_h - half_h),
                'left_pillar': (236, 132),
                'right_pillar': (236, 132),
            }
        else:
            slot_sizes = {
                'front': None,  # 메인 화면은 풀사이즈 유지
                'back': (480, sub_h),
                'left_repeater': (480, sub_h),
                'right_repeater': (480, front_h - 2 * sub_h),
                'left_pillar': (320, 180),
                'right_pillar': (320, 180),
            }

        def _fit_slot(raw_img, target_slot):
            if raw_img is None or raw_img.size == 0 or target_slot is None:
                return raw_img
            return OverlayRenderer.crop_and_resize_cover(raw_img, target_slot[0], target_slot[1], is_preview=True)

        all_cam_keys = ['front', 'back', 'left_repeater', 'right_repeater', 'left_pillar', 'right_pillar']
        frames = {k: None for k in all_cam_keys}
        cams_to_read = []

        for k in all_cam_keys:
            if k not in active_cam_keys:
                continue
            target_slot = slot_sizes.get(k)
            if reuse_same_frame and self.last_preview_cam_frame_idx.get(k) == global_idx:
                if k in self.last_valid_preview_frames and self.last_preview_slot_size.get(k) == target_slot:
                    frames[k] = self.last_valid_preview_frames[k]
                    continue
                raw_cached = self.last_raw_preview_frames.get(k)
                if raw_cached is not None and raw_cached.size > 0:
                    sized = _fit_slot(raw_cached, target_slot)
                    self.last_valid_preview_frames[k] = sized
                    self.last_preview_slot_size[k] = target_slot
                    frames[k] = sized
                    continue
            if k in self.caps:
                cams_to_read.append(k)

        def _read_one_cam(k):
            cap = self.caps[k]
            target_slot = slot_sizes.get(k)
            k_fps = getattr(self, 'cam_fps', {}).get(k, front_fps)
            k_fc = getattr(self, 'cam_fc', {}).get(k, 2160)
            target_k_f = max(0, min(k_fc - 1, int(round(t_sec * k_fps))))

            pos = getattr(self, 'current_cap_pos', {}).get(k, -1)
            raw_img = None
            ret = False
            new_frame_read = False

            if is_seeking or pos < 0 or target_k_f < pos or (target_k_f - pos) > 15:
                cap.set(cv2.CAP_PROP_POS_FRAMES, target_k_f)
                ret, raw_img = cap.read()
                self.current_cap_pos[k] = target_k_f + 1
                new_frame_read = True
            elif target_k_f == pos:
                ret, raw_img = cap.read()
                self.current_cap_pos[k] = pos + 1
                new_frame_read = True
            elif target_k_f > pos:
                while pos < target_k_f:
                    cap.grab()
                    pos += 1
                ret, raw_img = cap.read()
                self.current_cap_pos[k] = target_k_f + 1
                new_frame_read = True
            else:
                # target_k_f == pos - 1: 이전 프레임과 동일한 프레임 유지
                if k in self.last_valid_preview_frames and self.last_preview_slot_size.get(k) == target_slot:
                    return k, self.last_valid_preview_frames[k]
                raw_img = self.last_raw_preview_frames.get(k)
                ret = raw_img is not None and raw_img.size > 0
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, target_k_f)
                    ret, raw_img = cap.read()
                    self.current_cap_pos[k] = target_k_f + 1
                    new_frame_read = True

            if ret and raw_img is not None and raw_img.size > 0:
                if new_frame_read:
                    self.last_raw_preview_frames[k] = raw_img
                sized_img = _fit_slot(raw_img, target_slot)
                self.last_valid_preview_frames[k] = sized_img
                self.last_preview_slot_size[k] = target_slot
                self.last_preview_cam_frame_idx[k] = global_idx
                return k, sized_img
            elif k in self.last_valid_preview_frames:
                return k, self.last_valid_preview_frames[k]
            return k, None

        if len(cams_to_read) > 1 and hasattr(self, '_preview_pool') and self._preview_pool:
            for k, img in self._preview_pool.map(_read_one_cam, cams_to_read):
                frames[k] = img
        else:
            for k in cams_to_read:
                _, img = _read_one_cam(k)
                frames[k] = img

        return frames

    def on_enhance_plate_toggled(self, state):
        """번호판 선명화 ON 시 번호판 모자이크 자동 해제 (상호 배타)."""
        if self.chks["enhance_plate"].isChecked():
            chk_blur = self.chks["blur_plate"]
            if chk_blur.isChecked():
                chk_blur.blockSignals(True)
                chk_blur.setChecked(False)
                chk_blur.blockSignals(False)
        self.sync_preview()

    def on_blur_plate_toggled(self, state):
        """번호판 모자이크 ON 시 번호판 선명화 자동 해제 (상호 배타)."""
        if self.chks["blur_plate"].isChecked():
            chk_enh = self.chks["enhance_plate"]
            if chk_enh.isChecked():
                chk_enh.blockSignals(True)
                chk_enh.setChecked(False)
                chk_enh.blockSignals(False)
        self.sync_preview()

    def reset_preview_privacy(self):
        """프리뷰 탐색/클립 전환 시 블러 트래커를 리셋하여 잔상 튐을 방지합니다."""
        if hasattr(self, '_preview_privacy_filters'):
            for pf in self._preview_privacy_filters.values():
                pf.reset()

    def render_and_display(self, frames, global_idx):
        frames = dict(frames)
        opts = self.get_current_options()
        clip_idx = self.current_active_clip_idx if self.current_active_clip_idx >= 0 else 0
        local_idx = global_idx - self.clip_frame_offsets[clip_idx]

        # 프리뷰 블러/선명화 적용 (원본 프레임 보호를 위해 복사본에 적용)
        blur_face = opts.get("blur_face", False)
        blur_plate = opts.get("blur_plate", False)
        enhance_plate = opts.get("enhance_plate", False) and not blur_plate
        if blur_face or blur_plate or enhance_plate:
            if not hasattr(self, '_preview_privacy_filters') or \
               getattr(self, '_preview_blur_face', None) != blur_face or \
               getattr(self, '_preview_blur_plate', None) != blur_plate or \
               getattr(self, '_preview_enhance_plate', None) != enhance_plate:
                self._preview_privacy_filters = {}
                self._preview_blur_face = blur_face
                self._preview_blur_plate = blur_plate
                self._preview_enhance_plate = enhance_plate

            for k in frames:
                if frames[k] is not None and k not in self._preview_privacy_filters:
                    self._preview_privacy_filters[k] = PrivacyFilter(
                        blur_face=blur_face,
                        blur_plate=blur_plate,
                        enhance_plate=enhance_plate,
                        detect_interval=4
                    )

            force_detect = not (hasattr(self, 'timer') and self.timer.isActive())
            valid_cams = [k for k in frames if frames[k] is not None]
            if len(valid_cams) > 1 and hasattr(self, '_preview_pool'):
                def _proc_cam(k):
                    return k, self._preview_privacy_filters[k].apply(frames[k].copy(), force_detect=force_detect)
                results = list(self._preview_pool.map(_proc_cam, valid_cams))
                for k, processed_frame in results:
                    frames[k] = processed_frame
            else:
                for k in valid_cams:
                    frames[k] = self._preview_privacy_filters[k].apply(frames[k].copy(), force_detect=force_detect)

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
        self.last_grid_image = grid
        self._display_grid_image(grid)


    def _display_grid_image(self, grid_mat):
        h, w, ch = grid_mat.shape
        qimg = QImage(grid_mat.data, w, h, ch * w, QImage.Format.Format_BGR888)
        pixmap = QPixmap.fromImage(qimg)

        is_playing = hasattr(self, 'timer') and self.timer.isActive()
        transform_mode = (
            Qt.TransformationMode.FastTransformation
            if is_playing
            else Qt.TransformationMode.SmoothTransformation
        )
        scaled_pixmap = pixmap.scaled(
            self.preview_lbl.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            transform_mode
        )
        self.preview_lbl.setPixmap(scaled_pixmap)

    def toggle_play(self):
        if self.is_exporting or not self.active_clip_list:
            return

        if self.timer.isActive():
            self.timer.stop()
            self.btn_play.setText(tr("btn_play"))
            set_theme_style(self.btn_play, self._play_btn_idle_style)
            self.play_start_wall_time = None
            if self.last_grid_image is not None:
                self._display_grid_image(self.last_grid_image)
        else:
            if self.slider.value() >= self.total_frames - 1:
                self.slider.setValue(0)
            self.play_start_wall_time = time.perf_counter()
            self.play_start_frame = self.slider.value()
            self.timer.start(16)
            self.btn_play.setText(tr("btn_pause"))
            set_theme_style(self.btn_play, self._play_btn_active_style)

    def on_preview_speed_slider_changed(self, val):
        speed = val * 0.5  # 1→0.5x, 2→1.0x, ... 10→5.0x
        self.lbl_preview_speed_val.setText(f"{speed:.1f}x")
        # 재생 중이면 기준 시간 리셋 (배속 변경 시 튀는 현상 방지)
        if self.timer.isActive():
            self.play_start_frame = self.slider.value()
            self.play_start_wall_time = time.perf_counter()
        else:
            self.sync_preview()

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

        is_single = mode in ["단일", "단일 카메라", "단일 카메라 (풀스크린)", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]

        # 단일 카메라 모드 시 PIP OFF 및 체크박스 비활성화
        if is_single:
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
            background:@ct-selected-bg;
            color:@ct-text-main;
            border:1px solid @ct-border-active;
            font-weight:400;
            border-radius:4px;
            font-size:@font-body;
        """
        inactive_style = """
            background-color:@ct-control-bg;
            color:@ct-text-main;
            border:1px solid @ct-control-border;
            border-radius:4px;
            font-size:@font-body;
        """
        is_single = self.current_layout_mode in ["단일", "단일 카메라", "단일 카메라 (풀스크린)", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]
        set_theme_style(self.btn_layout_1to3, active_style if self.current_layout_mode == "기본 (1:3 세로배치)" else inactive_style)
        set_theme_style(self.btn_layout_2x2, active_style if self.current_layout_mode == "2x2 분할 (전후/좌우)" else inactive_style)
        set_theme_style(self.btn_layout_single, active_style if is_single else inactive_style)

    def get_current_single_cam(self):
        return getattr(self, "current_single_cam", "front")

    def on_click_layout_single(self):
        """ '단일' 레이아웃 버튼 클릭 핸들러:
        1) 단일 모드로 전환 (기존에 단일 모드가 아니었을 경우 기본 전면으로 세팅)
        2) 6채널 카메라 선택 팝업 표시 (바깥 클릭 시 자동 닫힘)
        """
        is_already_single = self.current_layout_mode in [
            "단일", "단일 카메라", "단일 카메라 (풀스크린)", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"
        ]

        if not is_already_single:
            # 단일을 눌렀을 때 기본은 전면인 상태
            self.current_single_cam = "front"
            self.set_layout_mode("단일")

        self.show_single_cam_popup()

    def show_single_cam_popup(self):
        """ 6개 카메라 선택 플로팅 팝업 표시 """
        if not hasattr(self, 'single_cam_popup') or self.single_cam_popup is None:
            self.single_cam_popup = SingleCameraPopup(self)
            self.single_cam_popup.cam_selected.connect(self.on_single_cam_selected)

        # 현재 클립의 가용 카메라 목록 전달 (필러 카메라 포함)
        available_cams = set()
        if self.active_clip_list and len(self.active_clip_list) > 0:
            available_cams = set(self.active_clip_list[0].get("cams", {}).keys())
        else:
            # 클립 로드 전일 경우 기본 4채널
            available_cams = {"front", "back", "left_repeater", "right_repeater"}

        curr_cam = getattr(self, "current_single_cam", "front")
        self.single_cam_popup.update_states(available_cams, curr_cam)

        # 팝업 위치 계산 (btn_layout_single 바로 아래 정렬 및 스크린 벗어남 방지)
        self.single_cam_popup.adjustSize()
        popup_w = self.single_cam_popup.sizeHint().width()
        popup_h = self.single_cam_popup.sizeHint().height()

        btn_rect = self.btn_layout_single.rect()
        btn_top_left = self.btn_layout_single.mapToGlobal(btn_rect.topLeft())
        btn_bottom_left = self.btn_layout_single.mapToGlobal(btn_rect.bottomLeft())

        # 버튼 중앙에 팝업 중앙 맞춤
        x = btn_bottom_left.x() + (self.btn_layout_single.width() - popup_w) // 2
        y = btn_bottom_left.y() + 4

        # 스크린 경계 보호
        screen = self.screen()
        if screen:
            screen_geo = screen.availableGeometry()
            if x + popup_w > screen_geo.right():
                x = screen_geo.right() - popup_w - 10
            if x < screen_geo.left():
                x = screen_geo.left() + 10
            if y + popup_h > screen_geo.bottom():
                y = btn_top_left.y() - popup_h - 4

        self.single_cam_popup.move(x, y)
        self.single_cam_popup.show()
        self.single_cam_popup.raise_()

    def on_single_cam_selected(self, cam_key):
        """ 팝업에서 특정 카메라 버튼 클릭 시 해당 카메라로 뷰 전환 """
        self.current_single_cam = cam_key
        if self.current_layout_mode not in ["단일", "단일 카메라", "단일 카메라 (풀스크린)", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]:
            self.set_layout_mode("단일")
        else:
            self.sync_preview()

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
            (f"{max_fps} FPS", max_fps),
            ("60 FPS", 60),
            ("30 FPS", 30),
            ("24 FPS", 24)
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
        if not (hasattr(self, 'timer') and self.timer.isActive()):
            self.sync_preview()

    def update_estimated_size(self):
        if not self.active_clip_list or self.total_frames <= 0:
            self.lbl_est_size.setText(tr("lbl_est_size", size="0"))
            return

        start_dt = self.start_point["dt"] if self.start_point else self.base_time
        end_dt = self.end_point["dt"] if self.end_point else (self.base_time + timedelta(seconds=self.total_frames / self.fps))

        dur_sec = (end_dt - start_dt).total_seconds()
        if dur_sec <= 0:
            self.lbl_est_size.setText(tr("lbl_est_size", size="0"))
            return

        export_speed = self.combo_export_speed.currentData()
        if export_speed is None:
            exp_speed_str = self.combo_export_speed.currentText()
            m = re.search(r'([\d\.]+)x', exp_speed_str)
            export_speed = float(m.group(1)) if m else 1.0

        out_duration_sec = dur_sec / export_speed

        res_id = self.combo_res.currentData() or "qhd"
        res_key = self.combo_res.currentText()
        target_fps = self.combo_fps.currentData() or 30
        # ── FPS 보정: H.264 CRF 인코딩은 프레임 간 압축으로 인해 FPS 증가 시 비트레이트가 비선형(약 0.45승)으로 증가 ──
        fps_ratio = (target_fps / 30.0) ** 0.45

        layout_mode = getattr(self, 'current_layout_mode', "기본 (1:3 세로배치)")

        # ── 해상도 × 레이아웃 실측 MB/분 테이블 ──────────────────────────────
        MEASURED_MBPM = {
            ("기본 (1:3 세로배치)",       "qhd"):     77.6,
            ("기본 (1:3 세로배치)",       "fhd"):     50.2,
            ("기본 (1:3 세로배치)",       "hd"):      27.1,
            ("기본 (1:3 세로배치)",       "compact"): 17.7,
            ("2x2 분할 (전후/좌우)",      "qhd"):     90.8,
            ("2x2 분할 (전후/좌우)",      "fhd"):     61.9,
            ("2x2 분할 (전후/좌우)",      "hd"):      32.0,
            ("2x2 분할 (전후/좌우)",      "compact"): 22.0,
            ("전면 단독 (전방 풀스크린)",  "qhd"):     78.8,
            ("전면 단독 (전방 풀스크린)",  "fhd"):     42.5,
            ("전면 단독 (전방 풀스크린)",  "hd"):      19.2,
            ("전면 단독 (전방 풀스크린)",  "compact"): 13.7,
        }
        lookup_layout = "전면 단독 (전방 풀스크린)" if layout_mode in ["단일", "단일 카메라", "단일 카메라 (풀스크린)", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"] else layout_mode
        base_mbpm = MEASURED_MBPM.get((lookup_layout, res_id), 50.2)

        # ── 오버레이 보정: 맵/PIP 없을 때 -13% (실측: 43.7/50.2 = 0.870) ──
        has_map = self.chks.get("map") and self.chks["map"].isChecked()
        has_pip = self.chks.get("pillar_pip") and self.chks["pillar_pip"].isChecked()
        overlay_factor = 1.0 if (has_map or has_pip) else 0.87

        # ── 배속 보정: 배속이 높아질수록 화면 변화량이 커져 H.264 압축 효율이 감소(초당 비트레이트 증가) ──
        speed_overhead = {1.0: 1.00, 0.5: 0.90, 1.5: 1.10, 2.0: 1.20, 3.0: 1.32, 4.0: 1.45, 5.0: 1.55}.get(export_speed, 1.0 + (export_speed - 1.0) * 0.15)

        # ── 최종 예상 용량 계산 ──
        est_mb = base_mbpm * fps_ratio * overlay_factor * (out_duration_sec / 60.0) * speed_overhead * 0.70

        self.lbl_est_size.setText(tr("lbl_est_size", size=f"{est_mb:.1f}"))

    def get_current_options(self):
        opts = {k: v.isChecked() for k, v in self.chks.items()}
        opts["layout"] = getattr(self, 'current_layout_mode', "기본 (1:3 세로배치)")
        opts["single_cam"] = self.get_current_single_cam()
        exp_speed_str = self.combo_export_speed.currentText()
        m = re.search(r'([\d\.]+)x', exp_speed_str)
        opts["export_speed"] = float(m.group(1)) if m else 1.0
        opts["preview_speed"] = (self.slider_preview_speed.value() * 0.5) if hasattr(self, 'slider_preview_speed') else 1.0
        opts["is_preview"] = not getattr(self, 'is_exporting', False)
        opts["telemetry_order"] = list(getattr(self, "telemetry_order", OverlayRenderer.DEFAULT_TELEMETRY_ORDER))
        opts["dragging_col"] = getattr(self, "_dragging_telemetry_key", None)

        opts["brightness"] = self.slider_brightness.value()
        opts["contrast"] = self.slider_contrast.value() / 100.0
        return opts

    def get_datetime_for_frame(self, global_f):
        """전역 슬라이더 프레임 번호에 대응하는 실제 타임스탬프 계산 (클립별 정확한 b_time 기반)."""
        if not self.active_clip_list:
            return self.base_time

        clip_idx = 0
        if hasattr(self, 'clip_frame_offsets') and len(self.clip_frame_offsets) > 1:
            for i in range(len(self.clip_frame_offsets) - 1):
                if self.clip_frame_offsets[i] <= global_f < self.clip_frame_offsets[i+1]:
                    clip_idx = i
                    break
            else:
                clip_idx = len(self.active_clip_list) - 1
            local_f = global_f - self.clip_frame_offsets[clip_idx]
        else:
            local_f = global_f

        c_info = self.active_clip_list[clip_idx]
        match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info.get("prefix", ""))
        clip_base = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time

        fps = self.fps if self.fps > 0 else 36.0
        return clip_base + timedelta(seconds=local_f / fps)

    def set_in_point(self):
        if self.is_exporting or not self.active_clip_list:
            return
        curr_f = self.slider.value()
        curr_dt = self.get_datetime_for_frame(curr_f)

        is_group = False
        if self.current_item:
            cdata = self.current_item.data(0, Qt.ItemDataRole.UserRole) or {}
            is_group = cdata.get("is_group", False)

        if self.end_point:
            if is_group:
                if curr_f >= self.end_point["frame"]:
                    QMessageBox.warning(self, tr("warning_title"), tr("order_warn_start"))
                    return
            else:
                if curr_dt >= self.end_point["dt"]:
                    QMessageBox.warning(self, tr("warning_title"), tr("order_warn_start"))
                    return

            dur_sec = (self.end_point["dt"] - curr_dt).total_seconds()
            if dur_sec > 1800.0:
                QMessageBox.warning(self, tr("warning_title"), tr("warn_max_duration", min=dur_sec / 60.0))
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
        curr_dt = self.get_datetime_for_frame(curr_f)

        is_group = False
        if self.current_item:
            cdata = self.current_item.data(0, Qt.ItemDataRole.UserRole) or {}
            is_group = cdata.get("is_group", False)

        if self.start_point:
            if is_group:
                if curr_f <= self.start_point["frame"]:
                    QMessageBox.warning(self, tr("warning_title"), tr("order_warn_end"))
                    return
            else:
                if curr_dt <= self.start_point["dt"]:
                    QMessageBox.warning(self, tr("warning_title"), tr("order_warn_end"))
                    return

            dur_sec = (curr_dt - self.start_point["dt"]).total_seconds()
            if dur_sec > 1800.0:
                QMessageBox.warning(self, tr("warning_title"), tr("warn_max_duration", min=dur_sec / 60.0))
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
        """트리의 구간 강조를 현재 테마의 기본 색상으로 복원."""
        brush_white = QBrush(QColor(color("text-main")))

        it = QTreeWidgetItemIterator(self.tree_explorer)
        while it.value():
            item = it.value()
            cdata = item.data(0, Qt.ItemDataRole.UserRole) or {}

            if cdata.get("is_virtual_dir", False) or "clip_list" in cdata:
                item.setBackground(0, QBrush())
                item.setBackground(1, QBrush())
                kind = cdata.get("clip_type", "recent")
                category_brush = QBrush(QColor(color("clip-" + kind)))
                item.setForeground(0, category_brush if cdata.get("is_virtual_dir") else brush_white)
                item.setForeground(1, category_brush)
            it += 1

    def _apply_tree_range_highlights(self, s_dt, e_dt):
        """선택 구간에 걸치는 클립을 현재 테마의 구간 색상으로 강조."""
        brush_white     = QBrush(QColor(color("text-main")))
        brush_yellow_bg = QBrush(QColor(color("warning-bg")))
        brush_yellow_fg = QBrush(QColor(color("warning-text")))
        clip_dur = timedelta(seconds=60)              # 테슬라 클립 기본 길이 1분

        it = QTreeWidgetItemIterator(self.tree_explorer)
        while it.value():
            item = it.value()
            cdata = item.data(0, Qt.ItemDataRole.UserRole) or {}
            if "clip_list" in cdata and not cdata.get("is_virtual_dir", False):
                clips = cdata.get("clip_list", [])
                if clips:
                    try:
                        def parse_prefix(pfx):
                            m = re.match(r'(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})', pfx)
                            return datetime.strptime(f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}", "%Y-%m-%d %H:%M:%S") if m else None

                        clip_start = parse_prefix(clips[0]["prefix"])
                        clip_end   = parse_prefix(clips[-1]["prefix"])
                        if clip_end:
                            clip_end += clip_dur

                        if clip_start and clip_end and clip_start < e_dt and clip_end > s_dt:
                            item.setBackground(0, brush_yellow_bg)
                            item.setBackground(1, brush_yellow_bg)
                            item.setForeground(0, brush_yellow_fg)
                            item.setForeground(1, brush_yellow_fg)
                        else:
                            item.setBackground(0, QBrush())
                            item.setBackground(1, QBrush())
                            item.setForeground(0, brush_white)
                            item.setForeground(1, QBrush(QColor(color("clip-" + cdata.get("clip_type", "recent")))))
                    except Exception:
                        pass
            it += 1

    def update_range_highlight(self):
        if not self.active_clip_list or (not self.start_point and not self.end_point):
            self.slider.set_range_points(None, None)
            total_sec = self.total_frames / self.fps if self.total_frames > 0 else 0
            self.lbl_range_info.setText(tr("range_none", total=tr("time_sec", sec=f"{total_sec:.1f}")))
            self._clear_tree_range_highlights()
            self.update_estimated_size()
            return

        in_f = None
        out_f = None

        is_group = False
        if self.current_item:
            cdata = self.current_item.data(0, Qt.ItemDataRole.UserRole) or {}
            is_group = cdata.get("is_group", False)

        if is_group:
            # 센트리 그룹: 전역 슬라이더에 모든 클립이 하나의 타임라인으로 통합되어 있음
            in_f = self.start_point["frame"] if self.start_point else 0
            out_f = self.end_point["frame"] if self.end_point else (self.total_frames - 1)
        else:
            # 개별 클립 모드: 현재 활성 아이템과 start_point / end_point 아이템의 순서 비교
            curr_item = self.current_item
            start_item = self.start_point.get("item") if self.start_point else None
            end_item = self.end_point.get("item") if self.end_point else None

            all_clip_items = []
            it = QTreeWidgetItemIterator(self.tree_explorer)
            while it.value():
                it_item = it.value()
                it_data = it_item.data(0, Qt.ItemDataRole.UserRole) or {}
                if not it_data.get("is_group", False) and not it_data.get("is_virtual_dir", False) and it_data.get("clip_list"):
                    all_clip_items.append(it_item)
                it += 1

            curr_idx = all_clip_items.index(curr_item) if curr_item in all_clip_items else -1
            start_idx = all_clip_items.index(start_item) if start_item in all_clip_items else -1
            end_idx = all_clip_items.index(end_item) if end_item in all_clip_items else -1

            # in_f 결정
            if self.start_point:
                if start_item == curr_item:
                    in_f = self.start_point["frame"]
                elif start_idx != -1 and curr_idx != -1:
                    in_f = 0 if start_idx < curr_idx else None
                else:
                    in_f = None
            elif self.end_point:
                in_f = 0

            # out_f 결정
            if self.end_point:
                if end_item == curr_item:
                    out_f = self.end_point["frame"]
                elif end_idx != -1 and curr_idx != -1:
                    out_f = (self.total_frames - 1) if end_idx > curr_idx else None
                else:
                    out_f = None
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
            self.lbl_range_info.setText(tr("range_selected", start=t1, end=t2, dur=tr("time_sec", sec=f"{dur:.1f}")))
        elif self.start_point:
            t1 = self.start_point["dt"].strftime("%H:%M:%S")
            self.lbl_range_info.setText(tr("range_start_only", start=t1))
        elif self.end_point:
            t2 = self.end_point["dt"].strftime("%H:%M:%S")
            self.lbl_range_info.setText(tr("range_end_only", end=t2))

        # ── 트리 하이라이트 ──────────────────────────────────────────────
        self._clear_tree_range_highlights()
        s_dt_hl = self.start_point["dt"] if self.start_point else (
            self.base_time)
        e_dt_hl = self.end_point["dt"] if self.end_point else (
            self.get_datetime_for_frame(self.total_frames - 1) if self.total_frames > 0 else self.base_time)
        self._apply_tree_range_highlights(s_dt_hl, e_dt_hl)

        self.update_estimated_size()


    def build_target_clip_chain(self):
        if not self.start_point or not self.end_point:
            # 구간 미지정 시 현재 로드된 클립 전체 내보내기
            target_clips = []
            for i, clip in enumerate(self.active_clip_list):
                match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', clip["prefix"])
                clip_base_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time
                if hasattr(self, 'clip_frame_offsets') and i + 1 < len(self.clip_frame_offsets):
                    fcount = self.clip_frame_offsets[i+1] - self.clip_frame_offsets[i]
                else:
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

        # 1) 동일한 아이템 내에서 선택되었거나, 활성 세션이 그룹(센트리 등)인 경우
        is_group = False
        if start_item:
            cdata = start_item.data(0, Qt.ItemDataRole.UserRole) or {}
            is_group = cdata.get("is_group", False)
        elif self.current_item:
            cdata = self.current_item.data(0, Qt.ItemDataRole.UserRole) or {}
            is_group = cdata.get("is_group", False)

        if (start_item == end_item or is_group) and self.active_clip_list:
            if is_group:
                # 센트리 그룹: clip_frame_offsets 기반으로 프레임 단위 정확 매칭
                global_s = min(self.start_point["frame"], self.end_point["frame"])
                global_e = max(self.start_point["frame"], self.end_point["frame"])
                target_clips = []
                for i, c_info in enumerate(self.active_clip_list):
                    if hasattr(self, 'clip_frame_offsets') and i + 1 < len(self.clip_frame_offsets):
                        c_start_f = self.clip_frame_offsets[i]
                        c_end_f = self.clip_frame_offsets[i+1] - 1
                    else:
                        c_start_f = 0
                        c_end_f = self.total_frames - 1
                    fcount = max(1, c_end_f - c_start_f + 1)

                    if c_end_f < global_s or c_start_f > global_e:
                        continue

                    local_s = max(0, global_s - c_start_f)
                    local_e = min(fcount - 1, global_e - c_start_f)

                    if local_s <= local_e:
                        match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info.get("prefix", ""))
                        b_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time
                        target_clips.append({
                            "cams": c_info["cams"],
                            "base_time": b_time,
                            "start_f": local_s,
                            "end_f": local_e
                        })
                return target_clips
            else:
                cdata = start_item.data(0, Qt.ItemDataRole.UserRole) if start_item else {}
                c_info = cdata.get("clip_list", [self.active_clip_list[0]])[0]
                match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info.get("prefix", ""))
                b_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time
                s_f = min(self.start_point["frame"], self.end_point["frame"])
                e_f = max(self.start_point["frame"], self.end_point["frame"])
                return [{
                    "cams": c_info["cams"],
                    "base_time": b_time,
                    "start_f": s_f,
                    "end_f": e_f
                }]

        # 2) 서로 다른 개별 클립 간에 걸쳐 선택된 경우 (같은 폴더 및 가상 디렉토리 간 연계 지원)
        if start_item and end_item:
            all_clip_items = []
            it = QTreeWidgetItemIterator(self.tree_explorer)
            while it.value():
                it_item = it.value()
                it_data = it_item.data(0, Qt.ItemDataRole.UserRole) or {}
                if not it_data.get("is_group", False) and not it_data.get("is_virtual_dir", False) and it_data.get("clip_list"):
                    all_clip_items.append(it_item)
                it += 1

            if start_item in all_clip_items and end_item in all_clip_items:
                start_idx = all_clip_items.index(start_item)
                end_idx = all_clip_items.index(end_item)

                if start_idx > end_idx:
                    QMessageBox.warning(self, tr("warning_title"), tr("warn_invalid_order"))
                    return []

                target_clips = []
                for i in range(start_idx, end_idx + 1):
                    child_item = all_clip_items[i]
                    cdata = child_item.data(0, Qt.ItemDataRole.UserRole)
                    if not cdata or not cdata.get("clip_list"):
                        continue

                    c_info = cdata["clip_list"][0]
                    cap = cv2.VideoCapture(c_info["cams"]["front"])
                    fcount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or int(60 * self.fps)
                    cap.release()

                    match = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', c_info["prefix"])
                    b_time = datetime.strptime(match.group(1), "%Y-%m-%d_%H-%M-%S") if match else self.base_time

                    if i == start_idx:
                        s_frame = self.start_point["frame"]
                    else:
                        s_frame = 0

                    if i == end_idx:
                        e_frame = self.end_point["frame"]
                    else:
                        e_frame = fcount - 1

                    s_frame = max(0, min(s_frame, fcount - 1))
                    e_frame = max(0, min(e_frame, fcount - 1))

                    if s_frame <= e_frame:
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
        self.tree_explorer.setEnabled(enabled)
        self.combo_res.setEnabled(enabled)
        self.btn_layout_1to3.setEnabled(enabled)
        self.btn_layout_2x2.setEnabled(enabled)
        self.btn_layout_single.setEnabled(enabled)
        if not enabled and hasattr(self, 'single_cam_popup') and self.single_cam_popup and self.single_cam_popup.isVisible():
            self.single_cam_popup.close()
        self.combo_export_speed.setEnabled(enabled)
        self.combo_fps.setEnabled(enabled)

        self.slider.setEnabled(enabled)
        self.btn_prev.setEnabled(enabled)
        self.btn_play.setEnabled(enabled)
        self.btn_next.setEnabled(enabled)
        self.btn_in.setEnabled(enabled)
        self.btn_out.setEnabled(enabled)
        self.btn_reset_range.setEnabled(enabled)
        if hasattr(self, 'slider_preview_speed'):
            self.slider_preview_speed.setEnabled(enabled)
        if hasattr(self, 'slider_brightness'):
            self.slider_brightness.setEnabled(enabled)
        if hasattr(self, 'slider_contrast'):
            self.slider_contrast.setEnabled(enabled)
        if hasattr(self, 'chk_motion'):
            self.chk_motion.setEnabled(enabled)
        if hasattr(self, 'combo_sensitivity'):
            self.combo_sensitivity.setEnabled(enabled)
        if hasattr(self, 'btn_prev_event'):
            self.btn_prev_event.setEnabled(enabled)
        if hasattr(self, 'btn_next_event'):
            self.btn_next_event.setEnabled(enabled)
        if hasattr(self, 'chk_auto_next'):
            self.chk_auto_next.setEnabled(enabled)
        if hasattr(self, 'chks'):
            for k, chk in self.chks.items():
                if k == "pillar_pip" and enabled:
                    has_pillar = any(
                        "left_pillar" in c.get("cams", {}) or "right_pillar" in c.get("cams", {})
                        for c in self.active_clip_list
                    )
                    chk.setEnabled(has_pillar)
                else:
                    chk.setEnabled(enabled)

    def on_click_export_button(self):
        if not self.active_clip_list:
            QMessageBox.warning(self, tr("warning_title"), tr("warn_no_clip_selected"))
            return

        if self.is_exporting:
            if hasattr(self, 'worker') and self.worker:
                self.btn_export.setText(tr("btn_export_cancelling"))
                self.btn_export.setEnabled(False)
                self.worker.stop()
            return

        default_dir = get_app_dir()
        default_path = os.path.join(default_dir, "CT_output.mp4")
        out_path, _ = QFileDialog.getSaveFileName(self, tr("export_save_dialog"), default_path, "MP4 (*.mp4)")
        if not out_path:
            return

        target_clips = self.build_target_clip_chain()
        if not target_clips:
            QMessageBox.warning(self, tr("warning_title"), tr("warn_no_valid_frames"))
            return

        if self.timer.isActive():
            self.toggle_play()

        self.is_exporting = True
        self.set_controls_enabled(False)
        self.btn_export.setText(tr("btn_export_cancel"))
        set_theme_style(self.btn_export, "background-color:@ct-danger-bg; color:@ct-text-main; font-weight:400;")
        self.on_export_progress(0)

        res_id = self.combo_res.currentData() or "qhd"
        target_size = RESOLUTIONS.get(res_id, RESOLUTIONS["qhd"])["size"]

        export_speed = self.combo_export_speed.currentData()
        if export_speed is None:
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
        set_theme_style(self.btn_qr_share, "background-color:@ct-selected-bg; color:@ct-text-main; font-weight:400;")

        # 작업표시줄 알림
        QApplication.alert(self, 0)

        # 완료 다이얼로그 (폴더 열기 / QR 전송 / 확인 선택 가능)
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle(tr("export_success_title"))
        msg_box.setText(tr("export_success_msg", path=output_path))
        msg_box.setIcon(QMessageBox.Icon.Information)

        btn_open = msg_box.addButton(tr("qr_open_folder"), QMessageBox.ButtonRole.ActionRole)
        btn_qr = msg_box.addButton(f"{tr('chk_qr_share')}", QMessageBox.ButtonRole.ActionRole)
        set_theme_style(btn_qr, "background-color:@ct-selected-bg; color:@ct-text-main; font-weight:400; padding:4px 10px;")
        btn_ok = msg_box.addButton(tr("btn_confirm"), QMessageBox.ButtonRole.AcceptRole)

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
        QMessageBox.critical(self, tr("export_fail_title"), err_msg)

    def on_export_cancelled(self):
        self.cleanup_export_ui()
        QMessageBox.information(self, tr("export_cancelled_title"), tr("export_cancelled_msg"))

    def cleanup_export_ui(self):
        self.is_exporting = False
        self.set_controls_enabled(True)
        self.btn_export.setEnabled(True)
        self.btn_export.setText(tr("btn_export"))
        set_theme_style(self.btn_export, "background-color:@ct-primary-bg; color:@ct-primary-text; font-weight:600; font-size:@font-body;")
        self.pbar.setValue(0)

        idx = self.slider.value()
        frames = self.read_frames_with_cache(idx, is_seeking=True)
        self.render_and_display(frames, idx)

    def on_click_qr_share(self):
        if self.last_exported_video_path and os.path.exists(self.last_exported_video_path):
            self.open_qr_share_dialog(self.last_exported_video_path)
        else:
            QMessageBox.warning(self, tr("warning_title"), tr("warn_file_not_found"))

    def open_qr_share_dialog(self, video_path):
        dlg = QRShareDialog(video_path, self)
        dlg.exec()

    def on_update_found(self, tag_name, url):
        reply = QMessageBox.question(
            self, tr("update_title"),
            tr("update_msg", tag=tag_name),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            webbrowser.open(url)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'toast_widget') and self.toast_widget and self.toast_widget.isVisible():
            self.toast_widget.reposition()
        if hasattr(self, 'single_cam_popup') and self.single_cam_popup and self.single_cam_popup.isVisible():
            self.single_cam_popup.close()
        if self.last_grid_image is not None and not self.is_exporting:
            self._display_grid_image(self.last_grid_image)

    def moveEvent(self, event):
        super().moveEvent(event)
        if hasattr(self, 'single_cam_popup') and self.single_cam_popup and self.single_cam_popup.isVisible():
            self.single_cam_popup.close()

    def closeEvent(self, event):
        self.stop_motion_scanner()
        if hasattr(self, 'checker') and self.checker and self.checker.isRunning():
            self.checker.wait(500)
        if hasattr(self, 'update_checker') and self.update_checker and self.update_checker.isRunning():
            self.update_checker.wait(500)
        if hasattr(self, 'loader_worker') and self.loader_worker and self.loader_worker.isRunning():
            self.loader_worker.stop()
            self.loader_worker.wait()
        if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait()
        if hasattr(self, '_preview_pool') and self._preview_pool:
            self._preview_pool.shutdown(wait=False)
        for cap in self.caps.values():
            cap.release()
        super().closeEvent(event)


# 하위 호환용 알리아스
TeslaStudioPro = CTDashcamStudio
