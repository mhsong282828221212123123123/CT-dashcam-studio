import os
from datetime import timedelta
import numpy as np
import cv2
import qrcode
from PIL import Image, ImageDraw, ImageFont
from core.map_loader import OpenStreetMapTileLoader
from core.utils import resource_path


class OverlayRenderer:
    _STEERING_ASSET_CACHE = {}
    _RAW_STEERING_PNG = None
    _RAW_STEERING_LOADED = False
    _FONT_CACHE = {}
    _HUD_TEMPLATE_CACHE = {}
    _QR_CACHE = {}

    RELEASES_QR_URL = "https://github.com/mhsong282828221212123123123/CT-dashcam-studio/releases/"
    DEFAULT_TELEMETRY_ORDER = ["timestamp", "steering", "turn_signal", "speed", "pedal", "fsd"]
    COL_WIDTH_BASE = {
        "timestamp":   185,
        "speed":       162,
        "pedal":       176,
        "steering":    118,
        "turn_signal": 118,
        "fsd":         156,
    }

    @classmethod
    def _get_releases_qr_image(cls, qr_px):
        qr_px = max(40, int(round(qr_px)))
        if qr_px in cls._QR_CACHE:
            return cls._QR_CACHE[qr_px]
        qr = qrcode.QRCode(
            version=None,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=4,
            border=2,
        )
        qr.add_data(cls.RELEASES_QR_URL)
        qr.make(fit=True)
        # BGR 순서: 눈부신 순백색 대신 다크 HUD 톤에 어울리는 차분한 슬레이트 그레이 배경 + 딥 다크 네이비 모듈
        qr_pil = qr.make_image(fill_color=(24, 18, 14), back_color=(168, 154, 144)).convert("RGB")
        qr_pil = qr_pil.resize((qr_px, qr_px), Image.Resampling.NEAREST)
        draw_q = ImageDraw.Draw(qr_pil)
        draw_q.rectangle([0, 0, qr_px - 1, qr_px - 1], outline=(82, 70, 60), width=max(1, int(round(qr_px / 64.0))))
        cls._QR_CACHE[qr_px] = qr_pil
        return qr_pil

    @classmethod
    def _get_pil_font(cls, kind, size):
        size = max(8, int(round(size)))
        key = (kind, size)
        if key in cls._FONT_CACHE:
            return cls._FONT_CACHE[key]

        candidates = {
            "bold": [
                "c:/windows/fonts/segoeuib.ttf",
                "c:/windows/fonts/malgunbd.ttf",
                "c:/windows/fonts/arialbd.ttf",
            ],
            "semibold": [
                "c:/windows/fonts/seguisb.ttf",
                "c:/windows/fonts/segoeuib.ttf",
                "c:/windows/fonts/malgun.ttf",
            ],
            "mono": [
                "c:/windows/fonts/consolab.ttf",
                "c:/windows/fonts/segoeuib.ttf",
            ],
            "kr": [
                "c:/windows/fonts/malgunbd.ttf",
                "c:/windows/fonts/malgun.ttf",
                "c:/windows/fonts/gulim.ttc",
            ],
        }.get(kind, ["c:/windows/fonts/segoeuib.ttf", "c:/windows/fonts/malgun.ttf"])

        font = None
        for fp in candidates:
            try:
                font = ImageFont.truetype(fp, size)
                break
            except Exception:
                pass
        if font is None:
            font = ImageFont.load_default()
        cls._FONT_CACHE[key] = font
        return font

    @classmethod
    def _get_base_steering_asset(cls, canvas_size):
        if canvas_size in cls._STEERING_ASSET_CACHE:
            return cls._STEERING_ASSET_CACHE[canvas_size]

        if not cls._RAW_STEERING_LOADED:
            cls._RAW_STEERING_LOADED = True
            try:
                asset_path = resource_path(os.path.join("models", "steering_hud.png"))
                if os.path.exists(asset_path):
                    raw = cv2.imread(asset_path, cv2.IMREAD_UNCHANGED)
                    if raw is not None and raw.ndim == 3 and raw.shape[2] == 4:
                        cls._RAW_STEERING_PNG = raw
            except Exception:
                cls._RAW_STEERING_PNG = None

        if cls._RAW_STEERING_PNG is not None:
            interp = cv2.INTER_AREA if canvas_size <= cls._RAW_STEERING_PNG.shape[0] else cv2.INTER_LINEAR
            resized = cv2.resize(cls._RAW_STEERING_PNG, (canvas_size, canvas_size), interpolation=interp)
            cls._STEERING_ASSET_CACHE[canvas_size] = resized
            return resized

        cx, cy = canvas_size // 2, canvas_size // 2
        img = np.zeros((canvas_size, canvas_size, 4), dtype=np.uint8)

        radius = int(canvas_size * 0.42)
        rim_thick = max(2, int(canvas_size * 0.075))

        cv2.circle(img, (cx, cy), radius, (210, 215, 220, 255), rim_thick, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), radius + rim_thick // 2, (70, 75, 80, 255), max(1, int(canvas_size * 0.008)), cv2.LINE_AA)
        cv2.circle(img, (cx, cy), radius - rim_thick // 2, (70, 75, 80, 255), max(1, int(canvas_size * 0.008)), cv2.LINE_AA)

        hub_radius = int(canvas_size * 0.18)
        cv2.circle(img, (cx, cy), hub_radius, (35, 38, 42, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), hub_radius, (160, 165, 170, 255), max(2, int(canvas_size * 0.015)), cv2.LINE_AA)

        spoke_thick = max(2, int(canvas_size * 0.06))
        inner_rim_r = radius - rim_thick // 2

        cv2.line(img, (cx - inner_rim_r, cy), (cx - hub_radius, cy), (140, 145, 150, 255), spoke_thick, cv2.LINE_AA)
        cv2.line(img, (cx + hub_radius, cy), (cx + inner_rim_r, cy), (140, 145, 150, 255), spoke_thick, cv2.LINE_AA)
        cv2.line(img, (cx, cy + hub_radius), (cx, cy + inner_rim_r), (140, 145, 150, 255), spoke_thick, cv2.LINE_AA)

        cv2.circle(img, (cx, cy), int(hub_radius * 0.6), (100, 105, 110, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), int(hub_radius * 0.6), (200, 205, 210, 255), max(1, int(canvas_size * 0.012)), cv2.LINE_AA)

        cls._STEERING_ASSET_CACHE[canvas_size] = img
        return img

    @classmethod
    def draw_rotated_steering(cls, canvas, center, angle_deg, scale=1.0, wheel_px=82):
        wheel_size = max(20, int(wheel_px * scale))

        # 3x 슈퍼샘플링 고해상도 회전 후 INTER_AREA 다운샘플링
        scale_factor = 3
        canvas_size = wheel_size * scale_factor
        cx, cy = canvas_size // 2, canvas_size // 2

        img = cls._get_base_steering_asset(canvas_size)

        M = cv2.getRotationMatrix2D((cx, cy), -angle_deg, 1.0)
        rotated = cv2.warpAffine(
            img, M, (canvas_size, canvas_size),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0, 0)
        )

        wheel_img = cv2.resize(rotated, (wheel_size, wheel_size), interpolation=cv2.INTER_AREA)

        h, w = wheel_img.shape[:2]
        x_top = int(center[0] - w / 2)
        y_top = int(center[1] - h / 2)

        ch_h, ch_w = canvas.shape[:2]
        x1, x2 = max(0, x_top), min(ch_w, x_top + w)
        y1, y2 = max(0, y_top), min(ch_h, y_top + h)

        rx1, rx2 = x1 - x_top, x2 - x_top
        ry1, ry2 = y1 - y_top, y2 - y_top

        if x1 < x2 and y1 < y2:
            alpha = wheel_img[ry1:ry2, rx1:rx2, 3:4] / 255.0
            rgb = wheel_img[ry1:ry2, rx1:rx2, :3]
            canvas[y1:y2, x1:x2] = (alpha * rgb + (1.0 - alpha) * canvas[y1:y2, x1:x2]).astype(np.uint8)

    @staticmethod
    def draw_turn_signals(canvas, center_x, center_y, left_on, right_on, frame_idx, fps=30.0, scale=1.0):
        time_sec = frame_idx / fps
        is_blink_phase = (int(time_sec * 3.5) % 2) == 0

        l_active = left_on and is_blink_phase
        r_active = right_on and is_blink_phase

        l_fill = (85, 245, 0) if l_active else (38, 44, 54)
        r_fill = (85, 245, 0) if r_active else (38, 44, 54)
        l_edge = (200, 255, 210) if l_active else (68, 76, 90)
        r_edge = (200, 255, 210) if r_active else (68, 76, 90)

        thick = max(1, int(round(1.8 * scale)))

        l_tip_x = center_x - int(48 * scale)
        l_head_x = center_x - int(22 * scale)
        l_stem_end_x = center_x - int(8 * scale)

        r_tip_x = center_x + int(48 * scale)
        r_head_x = center_x + int(22 * scale)
        r_stem_end_x = center_x + int(8 * scale)

        head_y = int(17 * scale)
        stem_y = int(7 * scale)

        pts_left = np.array([
            [l_tip_x, center_y],
            [l_head_x, center_y - head_y],
            [l_head_x, center_y - stem_y],
            [l_stem_end_x, center_y - stem_y],
            [l_stem_end_x, center_y + stem_y],
            [l_head_x, center_y + stem_y],
            [l_head_x, center_y + head_y]
        ], np.int32)

        pts_right = np.array([
            [r_tip_x, center_y],
            [r_head_x, center_y - head_y],
            [r_head_x, center_y - stem_y],
            [r_stem_end_x, center_y - stem_y],
            [r_stem_end_x, center_y + stem_y],
            [r_head_x, center_y + stem_y],
            [r_head_x, center_y + head_y]
        ], np.int32)

        cv2.fillPoly(canvas, [pts_left], l_fill, cv2.LINE_AA)
        cv2.polylines(canvas, [pts_left], True, l_edge, thick, cv2.LINE_AA)

        cv2.fillPoly(canvas, [pts_right], r_fill, cv2.LINE_AA)
        cv2.polylines(canvas, [pts_right], True, r_edge, thick, cv2.LINE_AA)

    @staticmethod
    def draw_pedal(frame, top_left, size, val_percent, label, color, scale=1.0):
        x, y = top_left
        w, h = size
        cv2.rectangle(frame, (x, y), (x + w, y + h), (30, 35, 45), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (68, 76, 92), max(1, int(1 * scale)))
        fill_h = int((h - 2) * (max(0.0, min(100.0, float(val_percent))) / 100.0))
        if fill_h > 0:
            cv2.rectangle(frame, (x + 1, y + h - 1 - fill_h), (x + w - 1, y + h - 1), color, -1)

    @classmethod
    def _draw_badge(cls, canvas, x, y, label, scale=1.0, text_color=(21, 204, 250), border_color=None,
                    font_scale_mult=0.44, alpha=0.82, align_right_x=None):
        font_px = max(10, int(round(31 * font_scale_mult * scale)))
        font = cls._get_pil_font("bold", font_px)

        dummy_img = Image.new("RGB", (4, 4))
        dummy_draw = ImageDraw.Draw(dummy_img)
        bbox = dummy_draw.textbbox((0, 0), label, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]

        pad_x = max(6, int(round(10 * scale)))
        pad_y = max(4, int(round(6 * scale)))
        tag_w = tw + pad_x * 2
        tag_h = th + pad_y * 2

        if align_right_x is not None:
            x = int(align_right_x - tag_w)

        ch_h, ch_w = canvas.shape[:2]
        x1, y1 = max(0, int(x)), max(0, int(y))
        x2, y2 = min(ch_w, int(x + tag_w)), min(ch_h, int(y + tag_h))

        if x1 < x2 and y1 < y2:
            roi = canvas[y1:y2, x1:x2]
            bg = np.zeros_like(roi, dtype=np.uint8)
            cv2.addWeighted(bg, alpha, roi, 1.0 - alpha, 0, dst=roi)
            if border_color is not None:
                cv2.rectangle(canvas, (x1, y1), (x2 - 1, y2 - 1), border_color, max(1, int(1 * scale)))

            pil_roi = Image.fromarray(roi)
            draw_roi = ImageDraw.Draw(pil_roi)
            tx = pad_x - bbox[0]
            ty = (tag_h - th) // 2 - bbox[1]
            draw_roi.text((tx, ty), label, font=font, fill=text_color)
            canvas[y1:y2, x1:x2] = np.asarray(pil_roi)

    @classmethod
    def crop_and_resize_cover(cls, frame, w, h, is_preview=False):
        if frame is None or frame.size == 0 or w <= 0 or h <= 0:
            return frame
        fh, fw = frame.shape[:2]
        if fw == w and fh == h:
            return frame

        slot_aspect = w / h
        frame_aspect = fw / fh

        if abs(frame_aspect - slot_aspect) < 1e-3:
            cropped = frame
        elif frame_aspect > slot_aspect:
            crop_w = max(1, int(fh * slot_aspect))
            start_x = (fw - crop_w) // 2
            cropped = frame[:, start_x:start_x+crop_w]
        else:
            crop_h = max(1, int(fw / slot_aspect))
            start_y = (fh - crop_h) // 2
            cropped = frame[start_y:start_y+crop_h, :]

        if cropped.shape[1] == w and cropped.shape[0] == h:
            return cropped

        interp = cv2.INTER_LINEAR if (is_preview or w >= cropped.shape[1]) else cv2.INTER_AREA
        return cv2.resize(cropped, (w, h), interpolation=interp)

    @classmethod
    def draw_sub_slot_cover(cls, canvas, x, y, w, h, frame, label, scale=1.0, is_preview=False):
        if frame is not None and frame.size > 0:
            canvas[y:y+h, x:x+w] = cls.crop_and_resize_cover(frame, w, h, is_preview=is_preview)
        else:
            cv2.rectangle(canvas, (x, y), (x+w, y+h), (20, 22, 26), -1)
            cv2.putText(canvas, "NO CAMERA", (x + int(w * 0.28), y + int(h * 0.58)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, (60, 65, 75), 1, cv2.LINE_AA)

        cv2.rectangle(canvas, (x, y), (x+w, y+h), (40, 45, 50), 1)
        # 각 카메라 프레임 좌상단에 밀착 (반투명 검정 rgba(0,0,0,0.82) + 밝은 노란색 #FACC15 볼드 텍스트, 테두리 없음)
        cls._draw_badge(canvas, x, y, label, scale=scale, text_color=(21, 204, 250), border_color=None, alpha=0.82)

    @classmethod
    def apply_rendering_overlay(cls, grid_img, progress_pct):
        h, w = grid_img.shape[:2]

        white_layer = np.full_like(grid_img, 255)
        blended = cv2.addWeighted(white_layer, 0.55, grid_img, 0.45, 0)

        box_w, box_h = int(680 * (w / 1920.0)), int(180 * (h / 1080.0))
        cx, cy = w // 2, h // 2
        x1, y1 = cx - box_w // 2, cy - box_h // 2
        x2, y2 = cx + box_w // 2, cy + box_h // 2

        cv2.rectangle(blended, (x1, y1), (x2, y2), (18, 22, 28), -1)
        cv2.rectangle(blended, (x1, y1), (x2, y2), (0, 200, 255), max(2, int(3 * (w / 1920.0))))

        text = f"영상 저장 렌더링 중... {progress_pct}%"
        font_size = int(36 * (w / 1920.0))

        try:
            img_pil = Image.fromarray(cv2.cvtColor(blended, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(img_pil)

            font = cls._FONT_CACHE.get(font_size)
            if font is None:
                font_paths = [
                    "c:/windows/fonts/malgun.ttf",
                    "c:/windows/fonts/gulim.ttc",
                    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
                    "/System/Library/Fonts/AppleSDGothicNeo.ttc"
                ]
                for fp in font_paths:
                    try:
                        font = ImageFont.truetype(fp, font_size)
                        cls._FONT_CACHE[font_size] = font
                        break
                    except Exception:
                        pass

            if font is not None:
                bbox = draw.textbbox((0, 0), text, font=font)
                tw = bbox[2] - bbox[0]
                th = bbox[3] - bbox[1]
                draw.text((cx - tw // 2, cy - th // 2), text, font=font, fill=(0, 230, 255))
                return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        except Exception:
            pass

        cv2.putText(blended, text, (cx - 240, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
        return blended

    @classmethod
    def compute_telemetry_layout(cls, options, out_w=1920, out_h=1080):
        scale = out_w / 1920.0
        has_bottom_hud = any(
            options.get(k) for k in ("map", "timestamp", "speed", "fsd", "turn_signal", "steering", "pedal")
        )
        bottom_h = int(180 * scale) if has_bottom_hud else 0
        front_h = out_h - bottom_h
        map_w = int(410 * scale) if options.get("map") else 0
        offset_x = map_w
        avail_w = out_w - offset_x

        qr_px = int(122 * scale)
        qr_pad_r = int(16 * scale)
        qr_zone_w = qr_px + qr_pad_r + int(24 * scale)
        cols_avail_w = max(int(200 * scale), avail_w - qr_zone_w)

        raw_order = options.get("telemetry_order") or cls.DEFAULT_TELEMETRY_ORDER
        order = [k for k in raw_order if k in cls.COL_WIDTH_BASE]
        for k in cls.DEFAULT_TELEMETRY_ORDER:
            if k not in order:
                order.append(k)

        active_cols = [(k, int(cls.COL_WIDTH_BASE[k] * scale)) for k in order if options.get(k)]
        num_active = len(active_cols)
        total_col_w = sum(w_col for _, w_col in active_cols)

        col_box_map = {}
        fsd_dividers = []
        if num_active > 0:
            fsd_indices = [i for i, (k, _) in enumerate(active_cols) if k == "fsd"]
            num_divs = 0
            if fsd_indices and num_active >= 2:
                fi = fsd_indices[0]
                if fi > 0:
                    num_divs += 1
                if fi < num_active - 1:
                    num_divs += 1
            div_unit = int(12 * scale)
            div_extra = num_divs * div_unit

            if options.get("map") and num_active == 6:
                side_pad_l = int(32 * scale)
                side_pad_r = int(20 * scale)
                inner_gap = max(int(14 * scale), (cols_avail_w - side_pad_l - side_pad_r - total_col_w - div_extra) / 5.0)
                curr_x = float(side_pad_l)
                for idx_c, (k, w_col) in enumerate(active_cols):
                    if k == "fsd" and idx_c > 0:
                        fsd_dividers.append(int(round(curr_x - inner_gap / 2.0 + div_unit / 2.0)))
                        curr_x += div_unit
                    col_box_map[k] = (int(round(curr_x)), w_col)
                    curr_x += w_col + inner_gap
                    if k == "fsd" and idx_c < num_active - 1:
                        fsd_dividers.append(int(round(curr_x - inner_gap / 2.0 + div_unit / 2.0)))
                        curr_x += div_unit
            else:
                free_w = max(int(20 * scale), cols_avail_w - total_col_w - div_extra)
                gap = free_w / float(num_active + 1)
                curr_x = gap
                for idx_c, (k, w_col) in enumerate(active_cols):
                    if k == "fsd" and idx_c > 0:
                        fsd_dividers.append(int(round(curr_x - gap / 2.0 + div_unit / 2.0)))
                        curr_x += div_unit
                    col_box_map[k] = (int(round(curr_x)), w_col)
                    curr_x += w_col + gap
                    if k == "fsd" and idx_c < num_active - 1:
                        fsd_dividers.append(int(round(curr_x - gap / 2.0 + div_unit / 2.0)))
                        curr_x += div_unit

        return col_box_map, tuple(fsd_dividers), offset_x, avail_w, front_h, bottom_h, qr_px, qr_pad_r

    @classmethod
    def render(cls, frames, frame_idx, base_time, options, decoder, fps=36.0, target_size=(1920, 1080)):
        out_w, out_h = target_size
        scale = out_w / 1920.0

        brightness = options.get("brightness", 0)
        contrast = options.get("contrast", 1.0)
        if brightness != 0 or contrast != 1.0:
            frames = {
                k: (cv2.convertScaleAbs(img, alpha=contrast, beta=brightness) if (img is not None and img.size > 0) else img)
                for k, img in frames.items()
            }

        col_box_map, fsd_dividers, offset_x, avail_w, front_h, bottom_h, qr_px, qr_pad_r = cls.compute_telemetry_layout(
            options, out_w, out_h
        )
        has_bottom_hud = bottom_h > 0
        sub_w = int(480 * scale)
        front_w_orig = out_w - sub_w

        canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)

        layout_mode = options.get("layout", "기본 (1:3 세로배치)")
        is_preview = bool(options.get("is_preview", False))

        if layout_mode == "2x2 분할 (전후/좌우)" or layout_mode == "2x2":
            half_w = out_w // 2
            half_h = front_h // 2

            # 4분할 슬롯 렌더링
            cls.draw_sub_slot_cover(canvas, 0, 0, half_w, half_h, frames.get('front'), "FRONT", scale, is_preview=is_preview)
            cls.draw_sub_slot_cover(canvas, 0, half_h, half_w, front_h - half_h, frames.get('back'), "REAR", scale, is_preview=is_preview)
            cls.draw_sub_slot_cover(canvas, half_w, 0, out_w - half_w, half_h, frames.get('left_repeater'), "LEFT REPEATER", scale, is_preview=is_preview)
            cls.draw_sub_slot_cover(canvas, half_w, half_h, out_w - half_w, front_h - half_h, frames.get('right_repeater'), "RIGHT REPEATER", scale, is_preview=is_preview)

            # 2x2 분할 중앙 십자(+) 구분선 (미려한 다크 슬레이트 보더)
            border_c = (55, 60, 70)
            thick_grid = max(2, int(2.5 * scale))
            cv2.line(canvas, (half_w, 0), (half_w, front_h), border_c, thick_grid)
            cv2.line(canvas, (0, half_h), (out_w, half_h), border_c, thick_grid)

            sp_right_x = out_w - int(8 * scale)

        elif layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1", "단일", "단일 카메라", "단일 카메라 (풀스크린)"]:
            # 단일 카메라 풀스크린 렌더링
            single_cam = options.get("single_cam", "front")
            cam_label_map = {
                'front': 'FRONT',
                'back': 'REAR',
                'left_repeater': 'LEFT REPEATER',
                'right_repeater': 'RIGHT REPEATER',
                'left_pillar': 'LEFT PILLAR',
                'right_pillar': 'RIGHT PILLAR'
            }
            label = cam_label_map.get(single_cam, single_cam.upper())
            cls.draw_sub_slot_cover(canvas, 0, 0, out_w, front_h, frames.get(single_cam), label, scale, is_preview=is_preview)
            sp_right_x = out_w - int(8 * scale)

        else:
            # 기본 (1:3 세로배치)
            front_w = front_w_orig
            f_front = frames.get('front')
            if f_front is not None and f_front.size > 0:
                canvas[0:front_h, 0:front_w] = cls.crop_and_resize_cover(f_front, front_w, front_h, is_preview=is_preview)

            cls._draw_badge(canvas, 0, 0, "FRONT MAIN", scale=scale, text_color=(21, 204, 250), border_color=None, alpha=0.82)

            sub_configs = [
                ("REAR", frames.get('back')),
                ("LEFT REPEATER", frames.get('left_repeater')),
                ("RIGHT REPEATER", frames.get('right_repeater')),
            ]

            num_slots = len(sub_configs)
            sub_h_single = front_h // num_slots

            for i, (label, f_img) in enumerate(sub_configs):
                sy = i * sub_h_single
                sh = sub_h_single if i < num_slots - 1 else (front_h - sy)
                cls.draw_sub_slot_cover(canvas, front_w, sy, sub_w, sh, f_img, label, scale, is_preview=is_preview)

            sp_right_x = front_w - int(8 * scale)

        export_speed = float(options.get("export_speed", 1.0))
        preview_speed = float(options.get("preview_speed", 1.0))
        speed_text = None
        if is_preview:
            if abs(export_speed - 1.0) > 0.01 and abs(preview_speed - 1.0) > 0.01:
                speed_text = f"VIEW {preview_speed:.1f}x | SAVE {export_speed:.1f}x"
            elif abs(export_speed - 1.0) > 0.01:
                speed_text = f"SAVE {export_speed:.1f}x SPEED"
            elif abs(preview_speed - 1.0) > 0.01:
                speed_text = f"VIEW {preview_speed:.1f}x"
        else:
            if abs(export_speed - 1.0) > 0.01:
                speed_text = f"{export_speed:.1f}x SPEED"

        if speed_text:
            cls._draw_badge(
                canvas, 0, 0, speed_text, scale=scale,
                text_color=(21, 204, 250), border_color=None,
                font_scale_mult=0.44, alpha=0.82, align_right_x=sp_right_x + int(8 * scale)
            )

        show_pillar_pip = options.get("pillar_pip", False)
        if show_pillar_pip:
            is_2x2 = layout_mode in ["2x2 분할 (전후/좌우)", "2x2"]
            pillar_w = int((236 if is_2x2 else 320) * scale)
            pillar_h = int((132 if is_2x2 else 180) * scale)

            if 'left_pillar' in frames:
                lp_x = int(8 * scale)
                lp_y = front_h - pillar_h - int(8 * scale)
                cls.draw_sub_slot_cover(canvas, lp_x, lp_y, pillar_w, pillar_h, frames.get('left_pillar'), "LEFT PILLAR", scale, is_preview=is_preview)

            if 'right_pillar' in frames:
                if is_2x2 or layout_mode in ["전면 단독 (전방 풀스크린)", "전면 단독", "1:1", "단일", "단일 카메라", "단일 카메라 (풀스크린)"]:
                    rp_x = out_w - pillar_w - int(8 * scale)
                else:
                    rp_x = front_w_orig - pillar_w - int(8 * scale)
                rp_y = front_h - pillar_h - int(8 * scale)
                cls.draw_sub_slot_cover(canvas, rp_x, rp_y, pillar_w, pillar_h, frames.get('right_pillar'), "RIGHT PILLAR", scale, is_preview=is_preview)

        if not has_bottom_hud:
            qr_no_hud = np.asarray(cls._get_releases_qr_image(qr_px))
            qx1 = out_w - qr_pad_r - qr_px
            qy1 = out_h - int(28 * scale) - qr_px
            if qx1 >= 0 and qy1 >= 0:
                canvas[qy1:qy1+qr_px, qx1:qx1+qr_px] = qr_no_hud
            wm_x = out_w - int(140 * scale)
            wm_y = out_h - int(10 * scale)
            cv2.putText(canvas, "Companion Turret", (wm_x, wm_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40 * scale, (140, 145, 155), 1, cv2.LINE_AA)
            return canvas

        data = decoder.get_frame_telemetry(frame_idx, fps) if decoder else {
            "speed_kmh": 0, "steering_deg": 0, "accel_pct": 0, "brake_applied": False, "brake_pct": 0,
            "ap_mode": "MANUAL", "left_blinker": False, "right_blinker": False, "lat": 0.0, "lon": 0.0, "heading": 0.0
        }

        map_w = offset_x
        map_h = bottom_h

        if options.get("map"):
            minimap = OpenStreetMapTileLoader.generate_minimap(
                data.get("lat", 0.0), data.get("lon", 0.0), heading=data.get("heading", 0.0), zoom=16, map_w=map_w, map_h=map_h
            )
            # 미니맵 좌측 하단에 선명한 OpenStreetMap 표기 배지 렌더링
            pil_map = Image.fromarray(minimap)
            draw_map = ImageDraw.Draw(pil_map, "RGBA")
            font_osm = cls._get_pil_font("semibold", 12 * scale)
            osm_txt = "Map: OpenStreetMap"
            o_bbox = draw_map.textbbox((0, 0), osm_txt, font=font_osm)
            ow = o_bbox[2] - o_bbox[0]
            oh = o_bbox[3] - o_bbox[1]
            pad_x = max(5, int(8 * scale))
            pad_y = max(3, int(4 * scale))
            bx1 = int(8 * scale)
            by2 = map_h - int(8 * scale)
            by1 = by2 - (oh + pad_y * 2)
            bx2 = bx1 + ow + pad_x * 2
            draw_map.rounded_rectangle(
                [bx1, by1, bx2, by2],
                radius=max(3, int(5 * scale)),
                fill=(22, 18, 15, 215),
                outline=(95, 82, 70, 230),
                width=max(1, int(scale))
            )
            draw_map.text(
                (bx1 + pad_x, by1 + pad_y - int(1 * scale)),
                osm_txt,
                font=font_osm,
                fill=(240, 232, 220, 255)
            )
            canvas[front_h:out_h, 0:map_w] = np.asarray(pil_map)

        time_sec = frame_idx / fps

        # ── 수직 정렬 그리드 상수 (5개 헤더 상단선, 콘텐츠 상단선/수직 중심선/하단 베이스라인 완벽 고정) ──
        hdr_y = int(16 * scale)
        content_top = int(58 * scale)
        baseline_y = int(138 * scale)
        center_y = (content_top + baseline_y) // 2  # 정확히 98 * scale

        # ── 정적 HUD 통합 다크 패널 템플릿 캐시 (개별 박스 없이 깔끔한 오픈 레이아웃, BGR 순서) ──
        tpl_key = (
            avail_w, bottom_h, round(scale, 3), bool(options.get("map")), "v5_qr_slate",
            fsd_dividers,
            tuple((k, x, w) for k, (x, w) in col_box_map.items())
        )
        hud_tpl = cls._HUD_TEMPLATE_CACHE.get(tpl_key)
        if hud_tpl is None:
            hud_tpl = np.zeros((bottom_h, avail_w, 3), dtype=np.uint8)
            for y_row in range(bottom_h):
                t = y_row / max(1.0, float(bottom_h - 1))
                b_val = int(round(28 * (1.0 - t) + 16 * t))
                g_val = int(round(21 * (1.0 - t) + 12 * t))
                r_val = int(round(17 * (1.0 - t) + 10 * t))
                hud_tpl[y_row, :] = (b_val, g_val, r_val)

            # 상단 경계선 및 미니맵 우측 경계선
            top_thick = max(1, int(round(1.5 * scale)))
            hud_tpl[0:top_thick, :] = (66, 54, 44)
            if options.get("map"):
                hud_tpl[:, 0:top_thick] = (66, 54, 44)

            pil_tpl = Image.fromarray(hud_tpl)
            draw_tpl = ImageDraw.Draw(pil_tpl)
            font_hdr = cls._get_pil_font("bold", 18.5 * scale)
            font_wm = cls._get_pil_font("semibold", 11.5 * scale)

            header_meta = {
                "speed":       ("SPEED",     (15, 225, 255)),
                "pedal":       ("PEDALS",    (130, 235, 80)),
                "steering":    ("STEERING",  (255, 215, 0)),
                "turn_signal": ("BLINKER",   (120, 240, 60)),
                "timestamp":   ("TIME",      (255, 210, 0)),
            }

            # SPEED, PEDALS, STEERING, BLINKER, TIME 5개 컬럼 상단 헤더를 동일한 수평선(hdr_y)에 고정
            h_ref_bbox = draw_tpl.textbbox((0, 0), "SPEED", font=font_hdr)
            hdr_draw_y = hdr_y - h_ref_bbox[1]
            hdr_ink_h = h_ref_bbox[3] - h_ref_bbox[1]
            dot_cy = hdr_y + hdr_ink_h // 2

            for k, (cx1, cw) in col_box_map.items():
                if k == "fsd":
                    continue
                title_txt, dot_bgr = header_meta.get(k, (k.upper(), (255, 220, 0)))
                dot_r = max(3, int(4.2 * scale))
                dot_cx = cx1 + dot_r
                draw_tpl.ellipse(
                    [dot_cx - dot_r, dot_cy - dot_r, dot_cx + dot_r, dot_cy + dot_r],
                    fill=dot_bgr
                )
                draw_tpl.text(
                    (dot_cx + int(9 * scale), hdr_draw_y),
                    title_txt,
                    font=font_hdr,
                    fill=(222, 212, 202)
                )

            # 주행 모드(FSD) 컬럼 구분용 옅은 수직 구분선(Divider)
            div_y1 = int(16 * scale)
            div_y2 = bottom_h - int(24 * scale)
            for div_x in fsd_dividers:
                draw_tpl.line(
                    [(div_x, div_y1), (div_x, div_y2)],
                    fill=(72, 60, 50),
                    width=max(1, int(round(1.5 * scale)))
                )

            # 우측 하단 GitHub Releases QR 코드 및 Companion Turret 워터마크
            qr_x1 = avail_w - qr_pad_r - qr_px
            qr_y1 = hdr_y
            qr_pil = cls._get_releases_qr_image(qr_px)
            pil_tpl.paste(qr_pil, (qr_x1, qr_y1))

            wm_text = "Companion Turret"
            wm_bbox = draw_tpl.textbbox((0, 0), wm_text, font=font_wm)
            wm_w = wm_bbox[2] - wm_bbox[0]
            wm_x = qr_x1 + (qr_px - wm_w) // 2
            draw_tpl.text(
                (wm_x, bottom_h - int(21 * scale)),
                wm_text,
                font=font_wm,
                fill=(138, 124, 114)
            )

            hud_tpl = np.asarray(pil_tpl).copy()
            cls._HUD_TEMPLATE_CACHE[tpl_key] = hud_tpl

        hud_slice = hud_tpl.copy()
        pil_hud = Image.fromarray(hud_slice)
        draw = ImageDraw.Draw(pil_hud)

        # 미리보기에서 마우스로 드래그 중인 텔레메트리 컬럼 하이라이트 표시
        dragging_col = options.get("dragging_col") if is_preview else None
        if dragging_col and dragging_col in col_box_map:
            dcx1, dcw = col_box_map[dragging_col]
            pad_hl = int(8 * scale)
            draw.rounded_rectangle(
                [max(2, dcx1 - pad_hl), int(8 * scale), min(avail_w - 2, dcx1 + dcw + pad_hl), bottom_h - int(10 * scale)],
                radius=max(4, int(8 * scale)),
                outline=(255, 215, 0),
                width=max(1, int(round(2 * scale)))
            )

        font_date = cls._get_pil_font("bold", 22 * scale)
        font_time = cls._get_pil_font("bold", 35 * scale)
        font_spd_num = cls._get_pil_font("bold", 48 * scale)
        font_spd_unit = cls._get_pil_font("semibold", 18 * scale)
        font_pedal_lbl = cls._get_pil_font("bold", 14.5 * scale)
        font_pedal_val = cls._get_pil_font("bold", 20 * scale)
        font_brk_state = cls._get_pil_font("bold", 14.5 * scale)

        # 1. SPEED (상단 숫자 ink = content_top, 하단 게이지 바 바닥 = baseline_y)
        if "speed" in col_box_map:
            cx1, cw = col_box_map["speed"]
            spd_val = int(round(float(data.get("speed_kmh", 0))))
            spd_str = f"{spd_val}"
            s_bbox = draw.textbbox((0, 0), spd_str, font=font_spd_num)
            s_w = s_bbox[2] - s_bbox[0]
            s_h = s_bbox[3] - s_bbox[1]

            yellow_bgr = (10, 228, 255)
            num_draw_y = content_top - s_bbox[1]
            draw.text((cx1, num_draw_y), spd_str, font=font_spd_num, fill=yellow_bgr)

            u_bbox = draw.textbbox((0, 0), "km/h", font=font_spd_unit)
            unit_draw_y = (content_top + s_h) - u_bbox[3]
            draw.text((cx1 + s_w + int(8 * scale), unit_draw_y), "km/h", font=font_spd_unit, fill=(215, 205, 195))

            bar_x1 = cx1
            bar_x2 = cx1 + int(150 * scale)
            bar_h_spd = max(4, int(7 * scale))
            bar_y2 = baseline_y
            bar_y1 = bar_y2 - bar_h_spd
            bar_r = max(2, int(3.5 * scale))
            draw.rounded_rectangle([bar_x1, bar_y1, bar_x2, bar_y2], radius=bar_r, fill=(52, 41, 34))
            spd_pct = max(0.0, min(1.0, spd_val / 150.0))
            if spd_pct > 0.01:
                fill_x2 = max(bar_x1 + bar_r * 2, int(round(bar_x1 + (bar_x2 - bar_x1) * spd_pct)))
                draw.rounded_rectangle([bar_x1, bar_y1, fill_x2, bar_y2], radius=bar_r, fill=yellow_bgr)

        # 2. PEDALS (ACC 수직 게이지 상단 = content_top, 게이지 및 BRK 배지 하단 = baseline_y 일직선 정렬)
        if "pedal" in col_box_map:
            cx1, cw = col_box_map["pedal"]
            acc_pct = int(round(max(0.0, min(100.0, float(data.get("accel_pct", 0))))))
            brk_on = bool(data.get("brake_applied", False)) or (float(data.get("brake_pct", 0)) > 0)

            bar_w = int(22 * scale)
            by1 = content_top
            by2 = baseline_y
            bar_h = by2 - by1
            bx1 = cx1
            bx2 = bx1 + bar_w
            br = max(4, bar_w // 2)

            draw.rounded_rectangle(
                [bx1, by1, bx2, by2],
                radius=br,
                fill=(22, 18, 14),
                outline=(95, 82, 70),
                width=max(1, int(round(1.3 * scale)))
            )
            for t_frac in (0.25, 0.50, 0.75):
                ty = int(round(by1 + bar_h * t_frac))
                draw.line([(bx2 + int(2 * scale), ty), (bx2 + int(6 * scale), ty)], fill=(100, 88, 76), width=max(1, int(scale)))

            if acc_pct > 0:
                fh = max(br * 2, int(round((bar_h - 2) * (acc_pct / 100.0))))
                fy1 = max(by1 + 1, by2 - fh)
                draw.rounded_rectangle([bx1 + 1, fy1, bx2 - 1, by2 - 1], radius=max(2, br - 1), fill=(110, 235, 0))

            # ACC / BRK 상단 라벨 및 하단 퍼센트 / 배지 (하단 라인을 baseline_y에 일치)
            acc_tx = bx2 + int(10 * scale)
            brk_x = cx1 + int(98 * scale)

            lbl_bbox = draw.textbbox((0, 0), "ACC", font=font_pedal_lbl)
            lbl_draw_y = (content_top + int(16 * scale)) - lbl_bbox[1]
            draw.text((acc_tx, lbl_draw_y), "ACC", font=font_pedal_lbl, fill=(215, 205, 195))
            draw.text((brk_x, lbl_draw_y), "BRK", font=font_pedal_lbl, fill=(215, 205, 195))

            lamp_h = int(29 * scale)
            lamp_y2 = baseline_y
            lamp_y1 = lamp_y2 - lamp_h
            lamp_x2 = brk_x + int(72 * scale)
            lamp_r = max(4, int(6 * scale))

            acc_str = f"{acc_pct}%"
            v_bbox = draw.textbbox((0, 0), acc_str, font=font_pedal_val)
            acc_val_c = (150, 255, 80) if acc_pct > 0 else (165, 155, 145)
            # % 텍스트 하단도 baseline_y(배지 하단)와 수평 정렬
            val_draw_y = baseline_y - v_bbox[3]
            draw.text((acc_tx, val_draw_y), acc_str, font=font_pedal_val, fill=acc_val_c)

            if brk_on:
                lamp_bg = (38, 28, 195)
                lamp_bd = (95, 90, 255)
                dot_c = (140, 140, 255)
                txt_c = (252, 252, 255)
                state_str = "ON"
            else:
                lamp_bg = (30, 24, 20)
                lamp_bd = (75, 64, 55)
                dot_c = (90, 80, 72)
                txt_c = (155, 145, 136)
                state_str = "OFF"

            draw.rounded_rectangle(
                [brk_x, lamp_y1, lamp_x2, lamp_y2],
                radius=lamp_r,
                fill=lamp_bg,
                outline=lamp_bd,
                width=max(1, int(round(1.3 * scale)))
            )
            ld_r = max(2, int(4.2 * scale))
            ld_cx = brk_x + int(15 * scale)
            ld_cy = (lamp_y1 + lamp_y2) // 2
            draw.ellipse([ld_cx - ld_r, ld_cy - ld_r, ld_cx + ld_r, ld_cy + ld_r], fill=dot_c)
            st_bbox = draw.textbbox((0, 0), state_str, font=font_brk_state)
            st_h = st_bbox[3] - st_bbox[1]
            st_y = lamp_y1 + (lamp_y2 - lamp_y1 - st_h) // 2 - st_bbox[1]
            draw.text((ld_cx + int(10 * scale), st_y), state_str, font=font_brk_state, fill=txt_c)

        # 5. TIME (날짜 상단 = content_top, 시간 텍스트 하단 = baseline_y 일직선 정렬)
        if "timestamp" in col_box_map:
            cx1, cw = col_box_map["timestamp"]
            curr_dt = base_time + timedelta(seconds=time_sec)
            d_str = curr_dt.strftime("%Y-%m-%d")
            t_str = curr_dt.strftime("%H:%M:%S")

            d_bbox = draw.textbbox((0, 0), d_str, font=font_date)
            t_bbox = draw.textbbox((0, 0), t_str, font=font_time)

            d_draw_y = content_top - d_bbox[1]
            t_draw_y = baseline_y - t_bbox[3]

            draw.text((cx1, d_draw_y), d_str, font=font_date, fill=(245, 202, 115))
            draw.text((cx1 - int(1 * scale), t_draw_y), t_str, font=font_time, fill=(252, 252, 255))

        # 6. 주행 모드 (맨 우측 끝 열: MANUAL, TACC, AUTOPILOT, FSD 4단계 + 반투명 사각 박스 하이라이트)
        if "fsd" in col_box_map:
            cx1, cw = col_box_map["fsd"]
            ap_raw = data.get("ap_mode", "MANUAL")
            active_key = "FSD" if ap_raw == "FSD ACTIVE" else ap_raw

            font_ap_item = cls._get_pil_font("bold", 19.5 * scale)
            modes_list = [
                ("MANUAL",    "MANUAL",    (248, 248, 252), (215, 215, 225), (62, 54, 48),  (165, 155, 145)),
                ("TACC",      "TACC",      (255, 205, 90),  (255, 185, 40),  (74, 48, 20),  (255, 175, 45)),
                ("AUTOPILOT", "AUTOPILOT", (165, 255, 100), (110, 245, 45),  (34, 66, 22),  (115, 235, 50)),
                ("FSD",       "FSD",       (255, 238, 80),  (255, 220, 0),   (78, 56, 14),  (255, 210, 0)),
            ]
            # 첫 항목 상단을 hdr_y(16*scale)에 맞추고 마지막 항목 하단을 baseline_y(138*scale)에 일치
            start_y = hdr_y
            box_h = int(27 * scale)
            step_y = int(31.5 * scale)
            box_w = int(152 * scale)
            box_r = max(3, int(6 * scale))
            inactive_txt = (98, 90, 82)
            inactive_dot = (68, 62, 56)

            for r_idx, (lbl_m, key_m, act_txt_c, act_dot_c, box_bg, box_bd) in enumerate(modes_list):
                is_act = (active_key == key_m)
                by1 = int(round(start_y + r_idx * step_y))
                by2 = by1 + box_h

                if is_act:
                    draw.rounded_rectangle(
                        [cx1, by1, cx1 + box_w, by2],
                        radius=box_r,
                        fill=box_bg,
                        outline=box_bd,
                        width=max(1, int(round(1.5 * scale)))
                    )

                d_r = max(2, int((4.2 if is_act else 3.2) * scale))
                d_cx = cx1 + int(12 * scale)
                d_cy = (by1 + by2) // 2
                draw.ellipse(
                    [d_cx - d_r, d_cy - d_r, d_cx + d_r, d_cy + d_r],
                    fill=act_dot_c if is_act else inactive_dot
                )

                m_bbox = draw.textbbox((0, 0), lbl_m, font=font_ap_item)
                m_th = m_bbox[3] - m_bbox[1]
                ty = by1 + (box_h - m_th) // 2 - m_bbox[1]
                draw.text(
                    (d_cx + int(10 * scale), ty),
                    lbl_m,
                    font=font_ap_item,
                    fill=act_txt_c if is_act else inactive_txt
                )

        hud_final = np.asarray(pil_hud)
        canvas[front_h:out_h, offset_x:out_w] = hud_final

        # 3. STEERING 나노바나나 핸들 아이콘 (콘텐츠 영역의 정확한 수직 중앙 center_y에 배치)
        if "steering" in col_box_map:
            cx1, cw = col_box_map["steering"]
            x_steer_wheel = offset_x + cx1 + int(52 * scale)
            y_steer_wheel = front_h + center_y
            cls.draw_rotated_steering(
                canvas, (x_steer_wheel, y_steer_wheel),
                data.get("steering_deg", 0), scale, wheel_px=96
            )

        # 4. BLINKER 화살표 (콘텐츠 영역의 정확한 수직 중앙 center_y에 배치)
        if "turn_signal" in col_box_map:
            cx1, cw = col_box_map["turn_signal"]
            x_blinker_center = offset_x + cx1 + int(52 * scale)
            y_blinker_center = front_h + center_y
            cls.draw_turn_signals(
                canvas, x_blinker_center, y_blinker_center,
                data.get("left_blinker", False), data.get("right_blinker", False),
                frame_idx, fps, scale * 0.96
            )

        # 상단 구분선 및 미니맵 경계선 (메탈릭 다크 슬레이트 엣지)
        cv2.line(canvas, (0, front_h), (out_w, front_h), (88, 72, 58), max(1, int(2 * scale)))
        if options.get("map"):
            cv2.line(canvas, (map_w, front_h), (map_w, out_h), (88, 72, 58), max(1, int(2 * scale)))

        return canvas
