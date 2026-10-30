"""Bundled desktop typeface and shared point sizes."""
from PyQt6.QtGui import QFontDatabase
from core.utils import resource_path

SIZES = {"body": 10, "small": 9, "title": 12, "symbol": 15}
_family = None


def font_family():
    global _family
    if _family is None:
        families = []
        for weight in ("Regular", "SemiBold", "Bold"):
            path = resource_path(f"assets/fonts/Pretendard-{weight}.otf")
            font_id = QFontDatabase.addApplicationFont(path)
            if font_id < 0:
                raise RuntimeError(f"Cannot load bundled UI font: {path}")
            families.extend(QFontDatabase.applicationFontFamilies(font_id))
        if "Pretendard" not in families:
            raise RuntimeError(f"Unexpected bundled font family: {families}")
        _family = "Pretendard"
    return _family


def font_size(role):
    return f"{SIZES[role]}pt"
