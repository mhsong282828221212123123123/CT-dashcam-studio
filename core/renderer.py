from datetime import timedelta
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont
from core.map_loader import OpenStreetMapTileLoader


class OverlayRenderer:
    @classmethod
    def draw_rotated_steering(cls, canvas, center, angle_deg, scale=1.0):
        wheel_size = max(20, int(90 * scale))
        
        # 4x 슈퍼샘플링 고해상도 렌더링으로 원형 왜곡 및 계단 현상 방지
        scale_factor = 4
        canvas_size = wheel_size * scale_factor
        cx, cy = canvas_size // 2, canvas_size // 2
        
        img = np.zeros((canvas_size, canvas_size, 4), dtype=np.uint8)
        
        radius = int(canvas_size * 0.42)
        rim_thick = max(2, int(canvas_size * 0.075))
        
        # 1. 외곽 원형 림 (Rim)
        cv2.circle(img, (cx, cy), radius, (210, 215, 220, 255), rim_thick, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), radius + rim_thick // 2, (70, 75, 80, 255), max(1, int(canvas_size * 0.008)), cv2.LINE_AA)
        cv2.circle(img, (cx, cy), radius - rim_thick // 2, (70, 75, 80, 255), max(1, int(canvas_size * 0.008)), cv2.LINE_AA)
        
        # 2. 중앙 허브 (Hub)
        hub_radius = int(canvas_size * 0.18)
        cv2.circle(img, (cx, cy), hub_radius, (35, 38, 42, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), hub_radius, (160, 165, 170, 255), max(2, int(canvas_size * 0.015)), cv2.LINE_AA)
        
        # 3. 3방향 스포크 (Spokes)
        spoke_thick = max(2, int(canvas_size * 0.06))
        inner_rim_r = radius - rim_thick // 2
        
        # 좌측 수평 스포크
        cv2.line(img, (cx - inner_rim_r, cy), (cx - hub_radius, cy), (140, 145, 150, 255), spoke_thick, cv2.LINE_AA)
        # 우측 수평 스포크
        cv2.line(img, (cx + hub_radius, cy), (cx + inner_rim_r, cy), (140, 145, 150, 255), spoke_thick, cv2.LINE_AA)
        # 하단 수직 스포크
        cv2.line(img, (cx, cy + hub_radius), (cx, cy + inner_rim_r), (140, 145, 150, 255), spoke_thick, cv2.LINE_AA)
        
        # 4. 허브 중앙 미니 링
        cv2.circle(img, (cx, cy), int(hub_radius * 0.6), (100, 105, 110, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), int(hub_radius * 0.6), (200, 205, 210, 255), max(1, int(canvas_size * 0.012)), cv2.LINE_AA)
        
        # 실제 조향 방향과 일치하도록 -angle_deg 적용
        M = cv2.getRotationMatrix2D((cx, cy), -angle_deg, 1.0)
        rotated = cv2.warpAffine(img, M, (canvas_size, canvas_size), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
        
        # INTER_AREA로 다운샘플링하여 완벽한 안티에일리어싱 적용
        wheel_img = cv2.resize(rotated, (wheel_size, wheel_size), interpolation=cv2.INTER_AREA)
        
        # 캔버스에 알파 블렌딩
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

        l_color = (0, 255, 100) if l_active else (45, 50, 55)
        r_color = (0, 255, 100) if r_active else (45, 50, 55)
        
        thick = max(1, int(2 * scale))

        l_tip_x = center_x - int(42 * scale)
        l_head_x = center_x - int(16 * scale)
        l_stem_end_x = center_x - int(6 * scale)

        head_y = int(18 * scale)
        stem_y = int(8 * scale)

        pts_left = np.array([
            [l_tip_x, center_y],
            [l_head_x, center_y - head_y],
            [l_head_x, center_y - stem_y],
            [l_stem_end_x, center_y - stem_y],
            [l_stem_end_x, center_y + stem_y],
            [l_head_x, center_y + stem_y],
            [l_head_x, center_y + head_y]
        ], np.int32)

        r_tip_x = center_x + int(42 * scale)
        r_head_x = center_x + int(16 * scale)
        r_stem_end_x = center_x + int(6 * scale)

        pts_right = np.array([
            [r_tip_x, center_y],
            [r_head_x, center_y - head_y],
            [r_head_x, center_y - stem_y],
            [r_stem_end_x, center_y - stem_y],
            [r_stem_end_x, center_y + stem_y],
            [r_head_x, center_y + stem_y],
            [r_head_x, center_y + head_y]
        ], np.int32)

        cv2.fillPoly(canvas, [pts_left], l_color)
        cv2.polylines(canvas, [pts_left], True, (255, 255, 255) if l_active else (70, 75, 80), thick)

        cv2.fillPoly(canvas, [pts_right], r_color)
        cv2.polylines(canvas, [pts_right], True, (255, 255, 255) if r_active else (70, 75, 80), thick)

    @staticmethod
    def draw_pedal(frame, top_left, size, val_percent, label, color, scale=1.0):
        x, y = top_left
        w, h = size
        thick = max(1, int(1 * scale))
        cv2.rectangle(frame, (x, y), (x + w, y + h), (70, 70, 75), thick)
        fill_h = int((h - 2) * (val_percent / 100.0))
        if fill_h > 0:
            cv2.rectangle(frame, (x + 1, y + h - 1 - fill_h), (x + w - 1, y + h - 1), color, -1)
        
        f1 = 0.45 * scale
        f2 = 0.45 * scale
        cv2.putText(frame, label, (x + int(2 * scale), y - int(8 * scale)), cv2.FONT_HERSHEY_SIMPLEX, f1, (200, 200, 200), thick)
        cv2.putText(frame, f"{val_percent}%", (x - int(4 * scale), y + h + int(18 * scale)), cv2.FONT_HERSHEY_SIMPLEX, f2, (160, 160, 160), thick)

    @staticmethod
    def draw_sub_slot_cover(canvas, x, y, w, h, frame, label, scale=1.0):
        if frame is not None and frame.size > 0:
            fh, fw = frame.shape[:2]
            slot_aspect = w / h
            frame_aspect = fw / fh

            if frame_aspect > slot_aspect:
                crop_w = int(fh * slot_aspect)
                start_x = (fw - crop_w) // 2
                cropped = frame[:, start_x:start_x+crop_w]
            else:
                crop_h = int(fw / slot_aspect)
                start_y = (fh - crop_h) // 2
                cropped = frame[start_y:start_y+crop_h, :]

            resized = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_NEAREST)
            canvas[y:y+h, x:x+w] = resized
        else:
            cv2.rectangle(canvas, (x, y), (x+w, y+h), (20, 22, 26), -1)
            cv2.putText(canvas, "NO CAMERA", (x + int(w * 0.28), y + int(h * 0.58)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, (60, 65, 75), 1)
            
        tag_w = int(140 * scale)
        tag_h = int(24 * scale)
        # 반투명 배경 배지 스타일
        cv2.rectangle(canvas, (x + 6, y + 6), (x + 6 + tag_w, y + 6 + tag_h), (12, 14, 18), -1)
        cv2.rectangle(canvas, (x + 6, y + 6), (x + 6 + tag_w, y + 6 + tag_h), (55, 60, 70), 1)
        cv2.putText(canvas, label, (x + 12, y + int(23 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44 * scale, (0, 230, 255), 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (x, y), (x+w, y+h), (40, 45, 50), 1)

    @staticmethod
    def apply_rendering_overlay(grid_img, progress_pct):
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
            
            font_paths = [
                "c:/windows/fonts/malgun.ttf",
                "c:/windows/fonts/gulim.ttc",
                "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
                "/System/Library/Fonts/AppleSDGothicNeo.ttc"
            ]
            font = None
            for fp in font_paths:
                try:
                    font = ImageFont.truetype(fp, font_size)
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
    def render(cls, frames, frame_idx, base_time, options, decoder, fps=36.0, target_size=(1920, 1080)):
        out_w, out_h = target_size
        scale = out_w / 1920.0

        brightness = options.get("brightness", 0)
        contrast = options.get("contrast", 1.0)
        if brightness != 0 or contrast != 1.0:
            for k, img in frames.items():
                if img is not None and img.size > 0:
                    frames[k] = cv2.convertScaleAbs(img, alpha=contrast, beta=brightness)

        bottom_h = int(180 * scale)
        front_h = out_h - bottom_h
        sub_w = int(480 * scale)
        front_w_orig = out_w - sub_w

        canvas = np.zeros((out_h, out_w, 3), dtype=np.uint8)
        
        layout_mode = options.get("layout", "기본 (1:3 세로배치)")
        
        if layout_mode == "2x2 분할 (전후/좌우)" or layout_mode == "2x2":
            half_w = out_w // 2
            half_h = front_h // 2
            
            # 4분할 슬롯 렌더링
            cls.draw_sub_slot_cover(canvas, 0, 0, half_w, half_h, frames.get('front'), "FRONT", scale)
            cls.draw_sub_slot_cover(canvas, 0, half_h, half_w, front_h - half_h, frames.get('back'), "REAR", scale)
            cls.draw_sub_slot_cover(canvas, half_w, 0, out_w - half_w, half_h, frames.get('left_repeater'), "LEFT REPEATER", scale)
            cls.draw_sub_slot_cover(canvas, half_w, half_h, out_w - half_w, front_h - half_h, frames.get('right_repeater'), "RIGHT REPEATER", scale)
            
            # 2x2 분할 중앙 십자(+) 구분선 (미려한 다크 슬레이트 보더)
            border_c = (55, 60, 70)
            thick_grid = max(2, int(2.5 * scale))
            cv2.line(canvas, (half_w, 0), (half_w, front_h), border_c, thick_grid)
            cv2.line(canvas, (0, half_h), (out_w, half_h), border_c, thick_grid)
            
            sp_x = out_w - int(130 * scale) - int(8 * scale)

        elif layout_mode == "전면 단독 (전방 풀스크린)" or layout_mode == "전면 단독" or layout_mode == "1:1":
            # 전면 카메라 단독 풀스크린 렌더링
            cls.draw_sub_slot_cover(canvas, 0, 0, out_w, front_h, frames.get('front'), "FRONT", scale)
            sp_x = out_w - int(130 * scale) - int(8 * scale)

        else:
            # 기본 (1:3 세로배치)
            front_w = front_w_orig
            f_front = frames.get('front')
            if f_front is not None and f_front.size > 0:
                fh, fw = f_front.shape[:2]
                f_aspect = fw / fh
                s_aspect = front_w / front_h
                if f_aspect > s_aspect:
                    crop_w = int(fh * s_aspect)
                    sx = (fw - crop_w) // 2
                    cropped = f_front[:, sx:sx+crop_w]
                else:
                    crop_h = int(fw / s_aspect)
                    sy = (fh - crop_h) // 2
                    cropped = f_front[sy:sy+crop_h, :]

                canvas[0:front_h, 0:front_w] = cv2.resize(cropped, (front_w, front_h), interpolation=cv2.INTER_LINEAR)
            
            tag_w = int(140 * scale)
            tag_h = int(24 * scale)
            cv2.rectangle(canvas, (8, 8), (8 + tag_w, 8 + tag_h), (12, 14, 18), -1)
            cv2.rectangle(canvas, (8, 8), (8 + tag_w, 8 + tag_h), (55, 60, 70), 1)
            cv2.putText(canvas, "FRONT MAIN", (14, int(24 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.44 * scale, (0, 230, 255), 1, cv2.LINE_AA)

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
                cls.draw_sub_slot_cover(canvas, front_w, sy, sub_w, sh, f_img, label, scale)
            
            sp_x = front_w - int(130 * scale) - int(8 * scale)

        export_speed = float(options.get("export_speed", 1.0))
        if abs(export_speed - 1.0) > 0.01:
            speed_text = f"{export_speed:.1f}x SPEED"
            sp_tag_w = int(130 * scale)
            sp_tag_h = int(28 * scale)
            sp_y = int(8 * scale)

            cv2.rectangle(canvas, (sp_x, sp_y), (sp_x + sp_tag_w, sp_y + sp_tag_h), (10, 12, 16), -1)
            cv2.rectangle(canvas, (sp_x, sp_y), (sp_x + sp_tag_w, sp_y + sp_tag_h), (0, 230, 255), max(1, int(1.5 * scale)))
            cv2.putText(canvas, speed_text, (sp_x + int(10 * scale), sp_y + int(20 * scale)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52 * scale, (0, 230, 255), max(1, int(2 * scale)))

        show_pillar_pip = options.get("pillar_pip", False)
        if show_pillar_pip:
            pillar_w = int(320 * scale)
            pillar_h = int(180 * scale)

            if 'left_pillar' in frames:
                lp_x = int(8 * scale)
                lp_y = front_h - pillar_h - int(8 * scale)
                cls.draw_sub_slot_cover(canvas, lp_x, lp_y, pillar_w, pillar_h, frames.get('left_pillar'), "LEFT PILLAR", scale)

            if 'right_pillar' in frames:
                if layout_mode in ["2x2 분할 (전후/좌우)", "2x2", "전면 단독 (전방 풀스크린)", "전면 단독", "1:1"]:
                    rp_x = out_w - pillar_w - int(8 * scale)
                else:
                    rp_x = front_w_orig - pillar_w - int(8 * scale)
                rp_y = front_h - pillar_h - int(8 * scale)
                cls.draw_sub_slot_cover(canvas, rp_x, rp_y, pillar_w, pillar_h, frames.get('right_pillar'), "RIGHT PILLAR", scale)

        canvas[front_h:out_h, 0:out_w] = (16, 18, 22)
        cv2.line(canvas, (0, front_h), (out_w, front_h), (60, 60, 60), max(1, int(2*scale)))

        data = decoder.get_frame_telemetry(frame_idx, fps) if decoder else {
            "speed_kmh": 0, "steering_deg": 0, "accel_pct": 0, "brake_pct": 0,
            "ap_mode": "MANUAL", "left_blinker": False, "right_blinker": False, "lat": 0.0, "lon": 0.0, "heading": 0.0
        }

        map_w = int(430 * scale) if options.get("map") else 0
        map_h = bottom_h

        if options.get("map"):
            minimap = OpenStreetMapTileLoader.generate_minimap(
                data.get("lat", 0.0), data.get("lon", 0.0), heading=data.get("heading", 0.0), zoom=16, map_w=map_w, map_h=map_h
            )
            canvas[front_h:out_h, 0:map_w] = minimap
            cv2.line(canvas, (map_w, front_h), (map_w, out_h), (60, 60, 60), max(1, int(2*scale)))

        offset_x = map_w
        thick_lbl = max(1, int(2 * scale))
        thick_val = max(2, int(2 * scale))
        thick_num = max(2, int(3 * scale))
        time_sec = frame_idx / fps

        lbl_y = front_h + int(42 * scale)
        val_y = front_h + int(118 * scale)
        mid_y = front_h + int(100 * scale)

        lbl_scale = 0.70 * scale
        lbl_color = (185, 185, 190)

        # 미니맵 유무에 따른 6개 컬럼 최적 가로 균등 분배 (Space-Between)
        if options.get("map"):
            # 남은 1490px에 균등 분배 (430px ~ 1920px)
            x_time = offset_x + int(24 * scale)
            x_speed = offset_x + int(330 * scale)
            x_ap = offset_x + int(560 * scale)
            x_blinker_lbl = offset_x + int(810 * scale)
            x_blinker_center = offset_x + int(880 * scale)
            x_steer_lbl = offset_x + int(1020 * scale)
            x_steer_wheel = offset_x + int(1075 * scale)
            x_steer_val = offset_x + int(1135 * scale)
            x_pedal_lbl = offset_x + int(1290 * scale)
            x_pedal_acc = offset_x + int(1300 * scale)
            x_pedal_brk = offset_x + int(1380 * scale)
        else:
            # 전체 1920px에 균등 분배 (0px ~ 1920px)
            x_time = int(50 * scale)
            x_speed = int(440 * scale)
            x_ap = int(740 * scale)
            x_blinker_lbl = int(1050 * scale)
            x_blinker_center = int(1120 * scale)
            x_steer_lbl = int(1310 * scale)
            x_steer_wheel = int(1370 * scale)
            x_steer_val = int(1430 * scale)
            x_pedal_lbl = int(1650 * scale)
            x_pedal_acc = int(1660 * scale)
            x_pedal_brk = int(1740 * scale)

        if options.get("timestamp"):
            t_str = (base_time + timedelta(seconds=time_sec)).strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(canvas, "TIME", (x_time, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cv2.putText(canvas, t_str, (x_time, val_y), cv2.FONT_HERSHEY_SIMPLEX, 0.85 * scale, (255, 255, 255), thick_val)

            if options.get("map"):
                map_src_y = front_h + int(152 * scale)
                cv2.putText(canvas, "Map: OpenStreetMap", (x_time, map_src_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42 * scale, (120, 125, 130), 1, cv2.LINE_AA)

        if options.get("speed"):
            cv2.putText(canvas, "SPEED", (x_speed, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cv2.putText(canvas, f"{data['speed_kmh']} km/h", (x_speed, val_y + int(4 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 1.25 * scale, (0, 230, 255), thick_num)

        if options.get("fsd"):
            ap_text = data.get("ap_mode", "MANUAL")
            if ap_text == "FSD ACTIVE":
                color = (255, 160, 0)
            elif ap_text == "AUTOPILOT":
                color = (0, 220, 100)
            elif ap_text == "TACC":
                color = (0, 140, 255)
            else:
                color = (140, 140, 140)

            cv2.putText(canvas, "AUTOPILOT", (x_ap, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cv2.putText(canvas, ap_text, (x_ap, val_y), cv2.FONT_HERSHEY_SIMPLEX, 0.90 * scale, color, thick_val)

            detected_prof = data.get("fsd_profile")
            if ap_text == "FSD ACTIVE" and detected_prof:
                prof_tag = f"[ {detected_prof} ]"
                prof_y = front_h + int(152 * scale)
                cv2.putText(canvas, prof_tag, (x_ap, prof_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44 * scale, (200, 225, 255), 1, cv2.LINE_AA)

        if options.get("turn_signal"):
            cv2.putText(canvas, "BLINKER", (x_blinker_lbl, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cls.draw_turn_signals(canvas, x_blinker_center, mid_y + int(10 * scale), data['left_blinker'], data['right_blinker'], frame_idx, fps, scale)

        if options.get("steering"):
            cv2.putText(canvas, "STEERING", (x_steer_lbl, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            cls.draw_rotated_steering(canvas, (x_steer_wheel, mid_y + int(12 * scale)), data['steering_deg'], scale)
            cv2.putText(canvas, f"{data['steering_deg']} deg", (x_steer_val, val_y), cv2.FONT_HERSHEY_SIMPLEX, 0.85 * scale, (220, 220, 220), thick_val)

        if options.get("pedal"):
            cv2.putText(canvas, "PEDALS", (x_pedal_lbl, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, lbl_scale, lbl_color, thick_lbl)
            pedal_y = front_h + int(82 * scale)
            cls.draw_pedal(canvas, (x_pedal_acc, pedal_y), (int(28 * scale), int(50 * scale)), data['accel_pct'], "ACC", (0, 220, 0), scale)
            cls.draw_pedal(canvas, (x_pedal_brk, pedal_y), (int(28 * scale), int(50 * scale)), data['brake_pct'], "BRK", (0, 0, 220), scale)

        wm_x = out_w - int(185 * scale)
        wm_y = out_h - int(16 * scale)
        cv2.putText(canvas, "Companion Turret", (wm_x, wm_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42 * scale, (100, 105, 115), 1)

        return canvas
