import cv2
import numpy as np


class PlateEnhancer:
    """
    번호판 박스 영역 국소 가독성 개선기 (밝기 감마 / CLAHE 대비 / Unsharp 선명도).
    - 박스 단위로만 처리하여 비용이 미미하고 주변 영상은 보존.
    - 박스별 감마 값을 EMA로 평활화하여 프레임 간 깜빡임 방지.
    """

    MIN_WIDTH = 40          # 이보다 작은 번호판은 복원 한계로 스킵
    UPSCALE_TARGET_W = 160  # 이보다 작으면 확대 후 처리
    MAX_UPSCALE = 3.0
    TARGET_MEAN_L = 150.0   # 목표 평균 밝기 (LAB L)
    EMA = 0.75              # 이전 값 가중치
    PAD_RATIO = 0.06
    FEATHER = 4

    def __init__(self, strength=1.0):
        self.strength = float(strength)
        # [(cx, cy, gamma)] 이전 프레임 박스별 평활화 상태
        self._states = []

    def reset(self):
        self._states = []

    def _raw_gamma(self, mean_l):
        g = 1.0
        if 1.0 < mean_l < 254.0:
            g = np.log(self.TARGET_MEAN_L / 255.0) / np.log(mean_l / 255.0)
        return float(np.clip(g, 0.55, 1.5))

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
        """BGR ROI를 보정하여 동일 크기로 반환. (결과, 사용 gamma)"""
        h, w = roi.shape[:2]
        scale = 1.0
        work = roi
        if w < self.UPSCALE_TARGET_W:
            scale = min(self.MAX_UPSCALE, self.UPSCALE_TARGET_W / float(w))
            work = cv2.resize(roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

        work = cv2.bilateralFilter(work, 5, 40, 5)

        lab = cv2.cvtColor(work, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)

        # 감마 보정: 어두우면 밝힘, 과노출이면 약하게 낮춤
        raw_gamma = self._raw_gamma(float(np.mean(l_ch)))
        gamma = raw_gamma if gamma_override is None else gamma_override
        # 강도 반영 (strength 1.0 = 풀 보정)
        gamma = 1.0 + (gamma - 1.0) * self.strength
        lut = np.array([((i / 255.0) ** gamma) * 255.0 for i in range(256)], dtype=np.uint8)
        l_ch = cv2.LUT(l_ch, lut)

        # CLAHE 대비 개선
        clahe = cv2.createCLAHE(clipLimit=2.5 * self.strength + 0.5, tileGridSize=(4, 4))
        l_ch = clahe.apply(l_ch)

        out = cv2.cvtColor(cv2.merge([l_ch, a_ch, b_ch]), cv2.COLOR_LAB2BGR)

        # Unsharp Mask 선명화
        amount = 0.9 * self.strength
        blur = cv2.GaussianBlur(out, (0, 0), 1.2)
        out = cv2.addWeighted(out, 1.0 + amount, blur, -amount, 0)

        if scale != 1.0:
            out = cv2.resize(out, (w, h), interpolation=cv2.INTER_AREA)
        return out, raw_gamma

    def apply(self, frame, boxes):
        """프레임 내 번호판 박스들을 제자리에서 보정 (경계 페더링 합성)."""
        fh, fw = frame.shape[:2]
        new_states = []
        for rx, ry, rw, rh in boxes:
            px = int(rw * self.PAD_RATIO)
            py = int(rh * self.PAD_RATIO)
            x1, y1 = max(0, rx - px), max(0, ry - py)
            x2, y2 = min(fw, rx + rw + px), min(fh, ry + rh + py)
            bw, bh = x2 - x1, y2 - y1
            if bw < self.MIN_WIDTH or bh < 8:
                continue

            roi = frame[y1:y2, x1:x2]
            cx, cy = x1 + bw / 2.0, y1 + bh / 2.0

            # 박스별 감마를 EMA로 평활화 (단일 패스)
            mean_l = float(np.mean(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)))
            raw_gamma = self._raw_gamma(mean_l)
            g = self._smoothed_gamma(cx, cy, bw, raw_gamma, new_states)
            enhanced, _ = self.enhance_roi(roi, gamma_override=g)

            # 경계 페더링 마스크
            mask = np.zeros((bh, bw), dtype=np.float32)
            f = min(self.FEATHER, bw // 4, bh // 4)
            if f >= 1:
                mask[f:bh - f, f:bw - f] = 1.0
                mask = cv2.GaussianBlur(mask, (0, 0), max(1.0, f / 2.0))
            else:
                mask[:, :] = 1.0
            mask3 = mask[:, :, None]
            blended = enhanced.astype(np.float32) * mask3 + roi.astype(np.float32) * (1.0 - mask3)
            frame[y1:y2, x1:x2] = np.clip(blended, 0, 255).astype(np.uint8)

        self._states = new_states
        return frame
