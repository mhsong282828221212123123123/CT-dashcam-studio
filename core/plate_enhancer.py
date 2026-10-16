import cv2
import numpy as np


class PlateEnhancer:
    """
    번호판 박스 영역 국소 가독성 개선기 (밝기 감마 / CLAHE 대비 / Unsharp 선명도).
    - 박스 단위로만 처리하여 비용이 미미하고 주변 영상은 보존.
    - 박스별 감마 값을 EMA로 평활화하여 프레임 간 깜빡임 방지.
    """

    MIN_WIDTH = 30          # 가로 30px 미만 원거리 노이즈 방지 스킵
    MIN_HEIGHT = 12         # 세로 12px 미만 원거리 노이즈 방지 스킵
    TARGET_MEAN_L = 150.0   # 저노출/일반 번호판 목표 평균 밝기 (LAB L)
    EMA = 0.75              # 이전 값 가중치
    PAD_RATIO = 0.05

    def __init__(self, strength=1.0):
        self.strength = float(strength)
        # [(cx, cy, gamma)] 이전 프레임 박스별 평활화 상태
        self._states = []

    def reset(self):
        self._states = []

    def _raw_gamma(self, mean_l):
        """밝기 상태에 따른 감마 계산 (고노출: 어둡게 복원, 저노출: 밝게 보정)."""
        if mean_l >= 180.0:
            # 고노출(헤드라이트/조명 반사): 감마 > 1.0으로 살짝 어둡게 하여 글자 윤곽 타버림 방지
            g = 1.10 + min(0.30, (mean_l - 180.0) / 160.0)
            return float(np.clip(g, 1.1, 1.4))
        else:
            # 저노출/일반: 목표 밝기(150) 대비 감마 산출 (1.0 미만이면 밝아짐)
            g = 1.0
            if 1.0 < mean_l < 254.0:
                g = np.log(self.TARGET_MEAN_L / 255.0) / np.log(mean_l / 255.0)
            return float(np.clip(g, 0.55, 1.15))

    def _smoothed_gamma(self, cx, cy, size, raw_gamma, new_states):
        best, best_d = None, max(30.0, size * 1.5)
        for i, (px, py, pg) in enumerate(self._states):
            d = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5
            if d < best_d:
                best, best_d = pg, d
        g = raw_gamma if best is None else self.EMA * best + (1.0 - self.EMA) * raw_gamma
        new_states.append((cx, cy, g))
        return g

    def enhance_roi(self, roi, gamma_override=None):
        """
        BGR ROI를 2배 확대(cv2.INTER_CUBIC) 후 밝기 상태별 조건부 보정(CLAHE + Unsharp Mask)을 수행하고,
        원래 ROI 크기(w, h)로 cv2.INTER_AREA 축소하여 반환. (결과, 사용 gamma)
        """
        h, w = roi.shape[:2]
        if h < 2 or w < 2:
            return roi, 1.0

        # 2. Sub-pixel 보정: 2배 확대 (INTER_CUBIC)로 계단 현상 및 Halo 억제
        work = cv2.resize(roi, (w * 2, h * 2), interpolation=cv2.INTER_CUBIC)
        work = cv2.bilateralFilter(work, 5, 30, 5)

        # 3. 밝기 상태에 따른 조건부 보정 (LAB L 채널 평균값 기반 분기)
        lab = cv2.cvtColor(work, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        mean_l = float(np.mean(l_ch))

        raw_gamma = self._raw_gamma(mean_l)
        gamma = raw_gamma if gamma_override is None else gamma_override
        gamma_eff = 1.0 + (gamma - 1.0) * self.strength

        if mean_l >= 180.0:
            # 고노출 번호판 (평균 밝기 180 이상): 헤드라이트 반사 등으로 이미 밝은 상태
            # CLAHE를 약하게(clipLimit=1.0) 적용하여 대비 과열 방지
            clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(4, 4))
            l_ch = clahe.apply(l_ch)
            # 감마를 살짝 낮춰(어둡게 하여) 글자 윤곽이 타버리는 현상 방지
            lut = np.array([np.clip(((i / 255.0) ** gamma_eff) * 255.0, 0, 255) for i in range(256)], dtype=np.uint8)
            l_ch = cv2.LUT(l_ch, lut)
        else:
            # 저노출/일반 번호판: 기존처럼 CLAHE(clipLimit=2.0) 및 밝기 감마 보정
            clip_limit = max(1.0, 2.0 * self.strength)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(4, 4))
            l_ch = clahe.apply(l_ch)
            lut = np.array([np.clip(((i / 255.0) ** gamma_eff) * 255.0, 0, 255) for i in range(256)], dtype=np.uint8)
            l_ch = cv2.LUT(l_ch, lut)

        out = cv2.cvtColor(cv2.merge([l_ch, a_ch, b_ch]), cv2.COLOR_LAB2BGR)

        # Unsharp Mask 선명화 (2배 업스케일 공간에서 선명화 적용 후 다운스케일)
        amount = 0.85 * self.strength
        blur = cv2.GaussianBlur(out, (0, 0), 1.2)
        out = cv2.addWeighted(out, 1.0 + amount, blur, -amount, 0)

        # 원래 ROI 크기(w, h)로 cv2.INTER_AREA를 사용해 축소
        out = cv2.resize(out, (w, h), interpolation=cv2.INTER_AREA)
        return out, raw_gamma

    def apply(self, frame, boxes):
        """프레임 내 번호판 박스들을 제자리에서 보정 (경계 페더링 합성)."""
        fh, fw = frame.shape[:2]
        new_states = []
        for rx, ry, rw, rh in boxes:
            # 1. 최소 크기 필터링 (원거리 노이즈 방지: w < 30 또는 h < 12 픽셀 미만 스킵)
            if rw < self.MIN_WIDTH or rh < self.MIN_HEIGHT:
                continue

            px = int(rw * self.PAD_RATIO)
            py = int(rh * self.PAD_RATIO)
            x1, y1 = max(0, rx - px), max(0, ry - py)
            x2, y2 = min(fw, rx + rw + px), min(fh, ry + rh + py)
            bw, bh = x2 - x1, y2 - y1
            if bw < self.MIN_WIDTH or bh < self.MIN_HEIGHT:
                continue

            roi = frame[y1:y2, x1:x2]
            cx, cy = x1 + bw / 2.0, y1 + bh / 2.0

            # 박스별 감마를 EMA로 평활화하여 프레임 간 깜빡임 방지
            lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            mean_l = float(np.mean(lab_roi[:, :, 0]))
            raw_gamma = self._raw_gamma(mean_l)
            g = self._smoothed_gamma(cx, cy, bw, raw_gamma, new_states)
            enhanced, _ = self.enhance_roi(roi, gamma_override=g)

            # 4. 경계선 페더링 (알파 블렌딩 합성)
            pad = max(2, min(bw, bh) // 10)
            mask = np.zeros((bh, bw), dtype=np.float32)
            if bh > pad * 2 and bw > pad * 2:
                mask[pad:bh - pad, pad:bw - pad] = 1.0
                ksize = int(pad * 2 + 1) | 1
                sigma = float(pad) / 2.0
                mask = cv2.GaussianBlur(mask, (ksize, ksize), sigma)
                mask[0, :] = 0.0
                mask[-1, :] = 0.0
                mask[:, 0] = 0.0
                mask[:, -1] = 0.0
                mask = cv2.GaussianBlur(mask, (3, 3), 0.5)
                mask = np.clip(mask, 0.0, 1.0)
            else:
                mask[:, :] = 1.0

            mask3 = mask[:, :, None]
            blended = enhanced.astype(np.float32) * mask3 + roi.astype(np.float32) * (1.0 - mask3)
            frame[y1:y2, x1:x2] = np.clip(blended, 0.0, 255.0).astype(np.uint8)

        self._states = new_states
        return frame
