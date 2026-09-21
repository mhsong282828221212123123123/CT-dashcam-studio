# === App Version & Update Settings ===
APP_VERSION = "1.2.1"
GITHUB_REPO = "mhsong282828221212123123123/CT-dashcam-studio"

# 해상도별 36fps 기준 인코딩 비트레이트 (Mbps)
RESOLUTIONS = {
    "QHD (2560x1440) - 최고화질": {"size": (2560, 1440), "bitrate_mbps": 50.0},
    "FHD (1920x1080) - 권장 표준": {"size": (1920, 1080), "bitrate_mbps": 35.0},
    "HD (1280x720) - 용량 절감": {"size": (1280, 720), "bitrate_mbps": 18.0},
    "Compact (960x540) - 모바일용": {"size": (960, 540), "bitrate_mbps": 9.0},
}

SENSITIVITY_LEVELS = {
    "낮음": {"pixel_diff": 35, "min_area": 800},
    "보통": {"pixel_diff": 25, "min_area": 400},
    "높음": {"pixel_diff": 15, "min_area": 150},
    "매우높음": {"pixel_diff": 10, "min_area": 50}
}
