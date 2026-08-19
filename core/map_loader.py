import math
import urllib.request
import numpy as np
import cv2

OSM_TILE_CACHE = {}


class OpenStreetMapTileLoader:
    @staticmethod
    def latlon_to_pixel(lat, lon, zoom):
        lat_rad = math.radians(lat)
        n = 2.0 ** zoom
        x = (lon + 180.0) / 360.0 * n * 256.0
        y = (1.0 - math.log(math.tan(lat_rad) + (1.0 / math.cos(lat_rad))) / math.pi) / 2.0 * n * 256.0
        return x, y

    @classmethod
    def fetch_tile(cls, x, y, zoom):
        key = (zoom, x, y)
        if key in OSM_TILE_CACHE:
            return OSM_TILE_CACHE[key]

        url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
        req = urllib.request.Request(url, headers={'User-Agent': 'TeslaDashcamStudioPro/1.0'})
        try:
            with urllib.request.urlopen(req, timeout=0.2) as resp:
                arr = np.frombuffer(resp.read(), dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    OSM_TILE_CACHE[key] = img
                    return img
        except Exception:
            pass
        return None

    _CAR_ASSET_CACHE = None

    @classmethod
    def _get_base_car_asset(cls):
        if cls._CAR_ASSET_CACHE is not None:
            return cls._CAR_ASSET_CACHE

        canvas_dim = 512
        scale = canvas_dim / 64.0  # 8.0x super-sampling
        cx, cy = canvas_dim // 2, canvas_dim // 2
        
        img = np.zeros((canvas_dim, canvas_dim, 4), dtype=np.uint8)
        
        w_body = int(27 * scale)
        h_body_f = int(35 * scale)
        h_body_r = int(31 * scale)
        wb = w_body // 2
        
        # 1. 4개 타이어/휠 (펜더 내부 안착)
        wheel_w = int(3.6 * scale)
        wheel_h = int(10.0 * scale)
        wheel_color = (15, 15, 15, 255)
        
        fw_y = cy - int(h_body_f * 0.48)
        rw_y = cy + int(h_body_r * 0.48)
        
        for wy in (fw_y, rw_y):
            cv2.rectangle(img, (cx - wb - int(1.2 * scale), wy - wheel_h // 2),
                               (cx - wb + wheel_w - int(1.2 * scale), wy + wheel_h // 2), wheel_color, -1)
            cv2.rectangle(img, (cx + wb - wheel_w + int(1.2 * scale), wy - wheel_h // 2),
                               (cx + wb + int(1.2 * scale), wy + wheel_h // 2), wheel_color, -1)

        # 2. 사이드 미러 (Side Mirrors)
        mirror_y = cy - int(h_body_f * 0.20)
        mirror_w = int(5.6 * scale)
        mirror_h = int(3.2 * scale)
        
        lm_pts = np.array([
            [cx - wb + int(1 * scale), mirror_y + int(1.5 * scale)],
            [cx - wb - mirror_w, mirror_y - int(1.5 * scale)],
            [cx - wb - mirror_w, mirror_y + mirror_h],
            [cx - wb + int(1 * scale), mirror_y + int(3.5 * scale)],
        ], dtype=np.int32)

        rm_pts = np.array([
            [cx + wb - int(1 * scale), mirror_y + int(1.5 * scale)],
            [cx + wb + mirror_w, mirror_y - int(1.5 * scale)],
            [cx + wb + mirror_w, mirror_y + mirror_h],
            [cx + wb - int(1 * scale), mirror_y + int(3.5 * scale)],
        ], dtype=np.int32)

        # 3. 테슬라 유선형 메인 차체 (Model 3/Y)
        body_pts = np.array([
            [cx, cy - h_body_f],                                 # 프론트 범퍼 중앙
            [cx + int(wb * 0.42), cy - int(h_body_f * 0.95)],   # 전면 곡선
            [cx + int(wb * 0.82), cy - int(h_body_f * 0.74)],   # 헤드라이트/펜더
            [cx + int(wb * 0.98), cy - int(h_body_f * 0.38)],   # 프론트 도어
            [cx + int(wb * 0.90), cy - int(h_body_f * 0.05)],   # 캐빈 웨이스트라인
            [cx + int(wb * 0.98), cy + int(h_body_r * 0.38)],   # 리어 펜더
            [cx + int(wb * 0.92), cy + int(h_body_r * 0.82)],   # 리어 테일 코너
            [cx + int(wb * 0.55), cy + h_body_r],               # 리어 범퍼 곡선
            [cx, cy + int(h_body_r * 1.02)],                    # 리어 범퍼 중앙
            [cx - int(wb * 0.55), cy + h_body_r],               # 리어 범퍼 곡선
            [cx - int(wb * 0.92), cy + int(h_body_r * 0.82)],   # 리어 테일 코너
            [cx - int(wb * 0.98), cy + int(h_body_r * 0.38)],   # 리어 펜더
            [cx - int(wb * 0.90), cy - int(h_body_f * 0.05)],   # 캐빈 웨이스트라인
            [cx - int(wb * 0.98), cy - int(h_body_f * 0.38)],   # 프론트 도어
            [cx - int(wb * 0.82), cy - int(h_body_f * 0.74)],   # 헤드라이트/펜더
            [cx - int(wb * 0.42), cy - int(h_body_f * 0.95)],   # 전면 곡선
        ], dtype=np.int32)

        # 4. 또렷하고 굵은 블랙 외곽선 (Black Outer Border)
        border_thick = max(2, int(2.4 * scale))
        cv2.polylines(img, [lm_pts], True, (0, 0, 0, 255), border_thick, cv2.LINE_AA)
        cv2.polylines(img, [rm_pts], True, (0, 0, 0, 255), border_thick, cv2.LINE_AA)
        cv2.polylines(img, [body_pts], True, (0, 0, 0, 255), border_thick, cv2.LINE_AA)

        # 5. 비비드 테슬라 레드 바디 채우기 (Vivid Tesla Red)
        car_red = (28, 32, 235, 255)
        cv2.fillPoly(img, [lm_pts], car_red, cv2.LINE_AA)
        cv2.fillPoly(img, [rm_pts], car_red, cv2.LINE_AA)
        cv2.fillPoly(img, [body_pts], car_red, cv2.LINE_AA)
        
        cv2.polylines(img, [body_pts], True, (20, 20, 20, 255), max(1, int(1.0 * scale)), cv2.LINE_AA)
        cv2.polylines(img, [lm_pts], True, (20, 20, 20, 255), max(1, int(0.8 * scale)), cv2.LINE_AA)
        cv2.polylines(img, [rm_pts], True, (20, 20, 20, 255), max(1, int(0.8 * scale)), cv2.LINE_AA)

        # 6. 본넷 캐릭터 라인
        hl_color = (60, 65, 250, 255)
        cv2.line(img, (cx - int(wb * 0.32), cy - int(h_body_f * 0.85)),
                      (cx - int(wb * 0.40), cy - int(h_body_f * 0.48)), hl_color, max(1, int(1.2 * scale)), cv2.LINE_AA)
        cv2.line(img, (cx + int(wb * 0.32), cy - int(h_body_f * 0.85)),
                      (cx + int(wb * 0.40), cy - int(h_body_f * 0.48)), hl_color, max(1, int(1.2 * scale)), cv2.LINE_AA)

        # 7. 파노라믹 틴티드 글래스 루프
        gw = int(wb * 0.74)
        ws_pts = np.array([
            [cx, cy - int(h_body_f * 0.46)],
            [cx + int(gw * 0.85), cy - int(h_body_f * 0.33)],
            [cx + gw, cy - int(h_body_f * 0.05)],
            [cx - gw, cy - int(h_body_f * 0.05)],
            [cx - int(gw * 0.85), cy - int(h_body_f * 0.33)],
        ], dtype=np.int32)
        cv2.fillPoly(img, [ws_pts], (22, 24, 30, 255), cv2.LINE_AA)

        roof_pts = np.array([
            [cx - gw, cy - int(h_body_f * 0.02)],
            [cx + gw, cy - int(h_body_f * 0.02)],
            [cx + int(gw * 0.92), cy + int(h_body_r * 0.45)],
            [cx + int(gw * 0.68), cy + int(h_body_r * 0.74)],
            [cx - int(gw * 0.68), cy + int(h_body_r * 0.74)],
            [cx - int(gw * 0.92), cy + int(h_body_r * 0.45)],
        ], dtype=np.int32)
        cv2.fillPoly(img, [roof_pts], (15, 16, 22, 255), cv2.LINE_AA)
        
        cv2.line(img, (cx - gw, cy + int(h_body_r * 0.06)), (cx + gw, cy + int(h_body_r * 0.06)), (28, 32, 235, 255), max(1, int(1.6 * scale)), cv2.LINE_AA)

        cv2.polylines(img, [ws_pts], True, (50, 55, 65, 255), max(1, int(1.0 * scale)), cv2.LINE_AA)
        cv2.polylines(img, [roof_pts], True, (50, 55, 65, 255), max(1, int(1.0 * scale)), cv2.LINE_AA)

        # 8. 블레이드 LED 헤드라이트 (아이스 화이트)
        l_hl = np.array([
            [cx - int(wb * 0.40), cy - int(h_body_f * 0.89)],
            [cx - int(wb * 0.80), cy - int(h_body_f * 0.71)],
            [cx - int(wb * 0.64), cy - int(h_body_f * 0.67)],
        ], dtype=np.int32)
        cv2.fillPoly(img, [l_hl], (255, 255, 230, 255), cv2.LINE_AA)

        r_hl = np.array([
            [cx + int(wb * 0.40), cy - int(h_body_f * 0.89)],
            [cx + int(wb * 0.80), cy - int(h_body_f * 0.71)],
            [cx + int(wb * 0.64), cy - int(h_body_f * 0.67)],
        ], dtype=np.int32)
        cv2.fillPoly(img, [r_hl], (255, 255, 230, 255), cv2.LINE_AA)

        # 9. 리어 테일라이트
        cv2.line(img, (cx - int(wb * 0.82), cy + int(h_body_r * 0.76)),
                      (cx - int(wb * 0.40), cy + int(h_body_r * 0.96)), (40, 40, 255, 255), max(1, int(2.2 * scale)), cv2.LINE_AA)
        cv2.line(img, (cx + int(wb * 0.82), cy + int(h_body_r * 0.76)),
                      (cx + int(wb * 0.40), cy + int(h_body_r * 0.96)), (40, 40, 255, 255), max(1, int(2.2 * scale)), cv2.LINE_AA)

        cls._CAR_ASSET_CACHE = img
        return img

    @classmethod
    def draw_topview_car(cls, heading_deg, size=46):
        base_asset = cls._get_base_car_asset()
        canvas_dim = base_asset.shape[0]
        cx, cy = canvas_dim // 2, canvas_dim // 2
        
        # 고해상도 공간에서 Lanczos4로 방위각 회전
        M = cv2.getRotationMatrix2D((cx, cy), -heading_deg, 1.0)
        rotated_512 = cv2.warpAffine(base_asset, M, (canvas_dim, canvas_dim), flags=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
        
        # INTER_AREA를 이용한 완벽한 안티에일리어싱 다운샘플링
        return cv2.resize(rotated_512, (size, size), interpolation=cv2.INTER_AREA)

    @classmethod
    def generate_minimap(cls, lat, lon, heading=0.0, zoom=16, map_w=360, map_h=180):
        if lat == 0.0 and lon == 0.0:
            return cls._draw_offline_radar(map_w, map_h, "NO GPS", heading=heading)

        px, py = cls.latlon_to_pixel(lat, lon, zoom)
        tile_x = int(px // 256)
        tile_y = int(py // 256)

        stitched = np.full((256 * 3, 256 * 3, 3), (30, 30, 30), dtype=np.uint8)
        has_any_tile = False

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                tx, ty = tile_x + dx, tile_y + dy
                tile_img = cls.fetch_tile(tx, ty, zoom)
                
                pos_x = (dx + 1) * 256
                pos_y = (dy + 1) * 256

                if tile_img is not None:
                    stitched[pos_y:pos_y+256, pos_x:pos_x+256] = tile_img
                    has_any_tile = True
                else:
                    cv2.rectangle(stitched, (pos_x, pos_y), (pos_x+256, pos_y+256), (45, 45, 45), 1)

        if not has_any_tile:
            return cls._draw_offline_radar(map_w, map_h, f"{lat:.3f},{lon:.3f}", heading=heading)

        offset_x = 256 + int(px % 256)
        offset_y = 256 + int(py % 256)

        # 1.1배 텍스트 및 도로명 배율 확대 (1.1x 슈퍼샘플링 렌더링)
        zoom_scale = 1.10
        crop_w = int(round(map_w / zoom_scale))
        crop_h = int(round(map_h / zoom_scale))
        half_cw = crop_w // 2
        half_ch = crop_h // 2

        crop_y1 = max(0, offset_y - half_ch)
        crop_y2 = min(stitched.shape[0], offset_y + half_ch)
        crop_x1 = max(0, offset_x - half_cw)
        crop_x2 = min(stitched.shape[1], offset_x + half_cw)

        crop = stitched[crop_y1:crop_y2, crop_x1:crop_x2].copy()
        crop = cv2.resize(crop, (map_w, map_h), interpolation=cv2.INTER_LANCZOS4)

        half_w = map_w // 2
        half_h = map_h // 2

        # 차량 진행 방향에 맞추어 회전하는 탑뷰 자동차 아이콘 오버레이
        car_size = max(22, int(36 * (map_h / 180.0)))
        car_icon = cls.draw_topview_car(heading, size=car_size)
        
        ch, cw = car_icon.shape[:2]
        x1 = half_w - cw // 2
        y1 = half_h - ch // 2
        x2 = x1 + cw
        y2 = y1 + ch
        
        if x1 >= 0 and y1 >= 0 and x2 <= map_w and y2 <= map_h:
            alpha = car_icon[:, :, 3:4] / 255.0
            rgb = car_icon[:, :, :3]
            crop[y1:y2, x1:x2] = (alpha * rgb + (1.0 - alpha) * crop[y1:y2, x1:x2]).astype(np.uint8)

        cv2.rectangle(crop, (0, 0), (map_w - 1, map_h - 1), (60, 60, 60), 2)
        return crop

    @classmethod
    def _draw_offline_radar(cls, map_w, map_h, text, heading=0.0):
        img = np.full((map_h, map_w, 3), (20, 24, 28), dtype=np.uint8)
        cx, cy = map_w // 2, map_h // 2
        r = min(map_w, map_h)
        cv2.circle(img, (cx, cy), int(r * 0.4), (50, 60, 70), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), int(r * 0.2), (50, 60, 70), 1, cv2.LINE_AA)
        cv2.line(img, (cx, 10), (cx, map_h - 10), (40, 50, 60), 1)
        cv2.line(img, (10, cy), (map_w - 10, cy), (40, 50, 60), 1)
        
        car_size = max(22, int(36 * (map_h / 180.0)))
        car_icon = cls.draw_topview_car(heading, size=car_size)
        ch, cw = car_icon.shape[:2]
        x1, y1 = cx - cw // 2, cy - ch // 2
        x2, y2 = x1 + cw, y1 + ch
        if x1 >= 0 and y1 >= 0 and x2 <= map_w and y2 <= map_h:
            alpha = car_icon[:, :, 3:4] / 255.0
            rgb = car_icon[:, :, :3]
            img[y1:y2, x1:x2] = (alpha * rgb + (1.0 - alpha) * img[y1:y2, x1:x2]).astype(np.uint8)

        cv2.putText(img, text, (12, map_h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 185), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0, 0), (map_w - 1, map_h - 1), (60, 60, 60), 1)
        return img
