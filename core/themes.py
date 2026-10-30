"""Shared Qt geometry and the six palettes used by the web application.

Component templates share sizing and use semantic color tokens.
New components should use color(role) or @ct-role in style templates.
"""
import re
from core.typography import font_size, font_family, SIZES
from PyQt6.QtCore import QSettings, QTemporaryDir, Qt
from PyQt6.QtGui import QColor, QPalette, QImage, QPainter, QPen, QFont
from PyQt6.QtWidgets import QApplication, QToolTip

# Kept as Python data so frozen builds do not need an extra JSON resource.
PALETTES = {'graphite': {'bg-app': '#171817',
              'bg-sidebar': '#1d1e1c',
              'bg-card': '#252623',
              'bg-card-hover': '#30312d',
              'border-subtle': '#373933',
              'border-active': '#aebca5',
              'accent-blue': '#aebca5',
              'accent-cyan': '#c4cebc',
              'accent-gold': '#d2c2a0',
              'accent-orange': '#c5a776',
              'accent-green': '#aebca5',
              'accent-red': '#d49b93',
              'text-main': '#e5e4df',
              'text-muted': '#b3b4ac',
              'text-dim': '#85877f',
              'canvas-bg': '#151614',
              'control-bg': '#292b26',
              'control-border': '#45483f',
              'focus-color': '#b7c5ae',
              'folder-bg': '#33372e',
              'folder-border': '#626d56',
              'primary-bg': '#d7dfcf',
              'primary-text': '#20241c',
              'selected-bg': '#384032',
              'selected-border': '#8d9e80',
              'selected-text': '#dce5d4',
              'header-bg': '#252722',
              'hover-bg': '#2d3029',
              'badge-bg': '#292c25',
              'track-bg': '#30342b',
              'range-color': '#b9a984',
              'dialog-bg': '#242621',
              'warning-bg': '#302d24',
              'warning-border': '#645b45',
              'warning-text': '#d7c39d',
              'danger-bg': '#3f2b28',
              'danger-border': '#846058',
              'danger-text': '#e1b9b2',
              'switch-bg': '#4c5045',
              'switch-active': '#87987a'},
 'midnight': {'bg-app': '#14191f',
              'bg-sidebar': '#1a212a',
              'bg-card': '#222d39',
              'bg-card-hover': '#2b4055',
              'border-subtle': '#354354',
              'border-active': '#7e9fbd',
              'accent-blue': '#7e9fbd',
              'accent-cyan': '#e0e7ee',
              'accent-gold': '#d2c2a0',
              'accent-orange': '#c5a776',
              'accent-green': '#7e9fbd',
              'accent-red': '#d49b93',
              'text-main': '#e0e7ee',
              'text-muted': '#aebdcd',
              'text-dim': '#8092a6',
              'canvas-bg': '#14191f',
              'control-bg': '#222d39',
              'control-border': '#354354',
              'focus-color': '#7e9fbd',
              'folder-bg': '#1e2935',
              'folder-border': '#7e9fbd',
              'primary-bg': '#bfd3e6',
              'primary-text': '#192a3b',
              'selected-bg': '#2b4055',
              'selected-border': '#7e9fbd',
              'selected-text': '#dceaf5',
              'header-bg': '#1e2935',
              'hover-bg': '#2b4055',
              'badge-bg': '#1e2935',
              'track-bg': '#202f40',
              'range-color': '#b9a984',
              'dialog-bg': '#1a212a',
              'warning-bg': '#302d24',
              'warning-border': '#645b45',
              'warning-text': '#d7c39d',
              'danger-bg': '#3f2b28',
              'danger-border': '#846058',
              'danger-text': '#e1b9b2',
              'switch-bg': '#354354',
              'switch-active': '#7e9fbd',
              'drop-bg': '#1a212a'},
 'forest': {'bg-app': '#141b18',
            'bg-sidebar': '#1b2520',
            'bg-card': '#26332c',
            'bg-card-hover': '#2e4434',
            'border-subtle': '#3b4d42',
            'border-active': '#7da086',
            'accent-blue': '#7da086',
            'accent-cyan': '#e0e9e2',
            'accent-gold': '#d2c2a0',
            'accent-orange': '#c5a776',
            'accent-green': '#7da086',
            'accent-red': '#d49b93',
            'text-main': '#e0e9e2',
            'text-muted': '#aebeb2',
            'text-dim': '#809888',
            'canvas-bg': '#141b18',
            'control-bg': '#26332c',
            'control-border': '#3b4d42',
            'focus-color': '#7da086',
            'folder-bg': '#202f25',
            'folder-border': '#7da086',
            'primary-bg': '#c0d7c4',
            'primary-text': '#1d2d22',
            'selected-bg': '#2e4434',
            'selected-border': '#7da086',
            'selected-text': '#dcecdf',
            'header-bg': '#202f25',
            'hover-bg': '#2e4434',
            'badge-bg': '#202f25',
            'track-bg': '#253a2c',
            'range-color': '#b9a984',
            'dialog-bg': '#1b2520',
            'warning-bg': '#302d24',
            'warning-border': '#645b45',
            'warning-text': '#d7c39d',
            'danger-bg': '#3f2b28',
            'danger-border': '#846058',
            'danger-text': '#e1b9b2',
            'switch-bg': '#3b4d42',
            'switch-active': '#7da086',
            'drop-bg': '#1b2520'},
 'burgundy': {'bg-app': '#1d1518',
              'bg-sidebar': '#271d22',
              'bg-card': '#36272e',
              'bg-card-hover': '#4a2d39',
              'border-subtle': '#503b45',
              'border-active': '#b18597',
              'accent-blue': '#b18597',
              'accent-cyan': '#eee0e5',
              'accent-gold': '#d2c2a0',
              'accent-orange': '#c5a776',
              'accent-green': '#b18597',
              'accent-red': '#d49b93',
              'text-main': '#eee0e5',
              'text-muted': '#c6aeb8',
              'text-dim': '#a08793',
              'canvas-bg': '#1d1518',
              'control-bg': '#36272e',
              'control-border': '#503b45',
              'focus-color': '#b18597',
              'folder-bg': '#302229',
              'folder-border': '#b18597',
              'primary-bg': '#e3c2cd',
              'primary-text': '#361c26',
              'selected-bg': '#4a2d39',
              'selected-border': '#b18597',
              'selected-text': '#f4e0e8',
              'header-bg': '#302229',
              'hover-bg': '#4a2d39',
              'badge-bg': '#302229',
              'track-bg': '#3d2832',
              'range-color': '#b9a984',
              'dialog-bg': '#271d22',
              'warning-bg': '#302d24',
              'warning-border': '#645b45',
              'warning-text': '#d7c39d',
              'danger-bg': '#3f2b28',
              'danger-border': '#846058',
              'danger-text': '#e1b9b2',
              'switch-bg': '#503b45',
              'switch-active': '#b18597',
              'drop-bg': '#271d22'},
 'amber': {'bg-app': '#1c1912',
           'bg-sidebar': '#252117',
           'bg-card': '#332e20',
           'bg-card-hover': '#483d22',
           'border-subtle': '#4b4430',
           'border-active': '#ad985e',
           'accent-blue': '#ad985e',
           'accent-cyan': '#ece7da',
           'accent-gold': '#d2c2a0',
           'accent-orange': '#c5a776',
           'accent-green': '#ad985e',
           'accent-red': '#d49b93',
           'text-main': '#ece7da',
           'text-muted': '#c4bba5',
           'text-dim': '#9c927b',
           'canvas-bg': '#1c1912',
           'control-bg': '#332e20',
           'control-border': '#4b4430',
           'focus-color': '#ad985e',
           'folder-bg': '#2e291c',
           'folder-border': '#ad985e',
           'primary-bg': '#dfd0a6',
           'primary-text': '#302814',
           'selected-bg': '#483d22',
           'selected-border': '#ad985e',
           'selected-text': '#f0e5c7',
           'header-bg': '#2e291c',
           'hover-bg': '#483d22',
           'badge-bg': '#2e291c',
           'track-bg': '#3b3320',
           'range-color': '#b9a984',
           'dialog-bg': '#252117',
           'warning-bg': '#302d24',
           'warning-border': '#645b45',
           'warning-text': '#d7c39d',
           'danger-bg': '#3f2b28',
           'danger-border': '#846058',
           'danger-text': '#e1b9b2',
           'switch-bg': '#4b4430',
           'switch-active': '#ad985e',
           'drop-bg': '#252117'},
 'paper': {'bg-app': '#f1f1ed',
           'bg-sidebar': '#fafaf7',
           'bg-card': '#e9eae4',
           'bg-card-hover': '#dfe7d8',
           'border-subtle': '#c7cbc0',
           'border-active': '#788b6a',
           'accent-blue': '#788b6a',
           'accent-cyan': '#252921',
           'accent-gold': '#6a531f',
           'accent-orange': '#c5a776',
           'accent-green': '#788b6a',
           'accent-red': '#d49b93',
           'text-main': '#252921',
           'text-muted': '#51594a',
           'text-dim': '#68735f',
           'canvas-bg': '#f1f1ed',
           'control-bg': '#e9eae4',
           'control-border': '#c7cbc0',
           'focus-color': '#788b6a',
           'folder-bg': '#edeee8',
           'folder-border': '#788b6a',
           'primary-bg': '#34472e',
           'primary-text': '#ffffff',
           'selected-bg': '#dfe7d8',
           'selected-border': '#788b6a',
           'selected-text': '#26351f',
           'header-bg': '#edeee8',
           'hover-bg': '#dfe7d8',
           'badge-bg': '#edeee8',
           'track-bg': '#e1e5da',
           'range-color': '#9c803c',
           'dialog-bg': '#fafaf7',
           'warning-bg': '#f5edda',
           'warning-border': '#c3ac77',
           'warning-text': '#69511e',
           'danger-bg': '#f2e3e0',
           'danger-border': '#c2958d',
           'danger-text': '#893c32',
           'switch-bg': '#c7cbc0',
           'switch-active': '#788b6a',
           'drop-bg': '#fafaf7'}}
def _SETTINGS():
    return QSettings("CompanionTurret", "CT_Dashcam_Studio")

def resolve_theme(value):
    if value == "classic":
        return "midnight"
    return value if isinstance(value, str) and value in PALETTES else "graphite"

_current = resolve_theme(_SETTINGS().value("app_theme", "graphite"))

def current_theme():
    return _current

# Clip types keep their meaning across application palettes.
for _name, _palette in PALETTES.items():
    _palette.update(dict(zip(
        ("clip-recent", "clip-saved", "clip-sentry"),
        ("#39734e", "#346fa1", "#b14b44") if _name == "paper" else
        ("#93c5a4", "#9dc2e2", "#e0a39c"),
    )))


def color(role):
    return PALETTES[_current][role]

def theme_names(language):
    names = {
        "ko": ("그라파이트", "미드나이트", "포레스트", "버건디", "앰버", "페이퍼"),
        "en": ("Graphite", "Midnight", "Forest", "Burgundy", "Amber", "Paper"),
        "ja": ("グラファイト", "ミッドナイト", "フォレスト", "バーガンディ", "アンバー", "ペーパー"),
        "zh-Hans": ("石墨", "午夜", "森林", "酒红", "琥珀", "纸白"),
        "zh-Hant": ("石墨", "午夜", "森林", "酒紅", "琥珀", "紙白"),
    }
    return dict(zip(PALETTES, names.get(language, names["en"])))

def render_style(template):
    """Resolve semantic tokens only; themes cannot change component geometry."""
    template = re.sub(r"@font-([a-z]+)", lambda match: font_size(match.group(1)), template)
    return re.sub(r"@ct-([a-z-]+)", lambda match: color(match.group(1)), template)


def set_theme_style(widget, template):
    widget.setProperty("ctStyleTemplate", template)
    widget.setStyleSheet(render_style(template))


_icon_directory = None


def _check_icon():
    """Generate a tiny native indicator; frozen builds need no extra asset files."""
    global _icon_directory
    if _icon_directory is None:
        _icon_directory = QTemporaryDir()
    path = _icon_directory.filePath("check-" + _current + ".png")
    image = QImage(16, 16, QImage.Format.Format_ARGB32)
    image.fill(Qt.GlobalColor.transparent)
    painter = QPainter(image)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    pen = QPen(QColor(color("primary-text")), 2.2)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    painter.setPen(pen)
    painter.drawLine(3, 8, 7, 12)
    painter.drawLine(7, 12, 13, 4)
    painter.end()
    image.save(path)
    return path.replace("\\", "/")


def stylesheet():
    p = {**PALETTES[_current], "check-icon": _check_icon(), "font-body": font_size("body"), "font-family": font_family()}
    return """
QWidget { font-family: "%(font-family)s"; background: %(bg-app)s; color: %(text-main)s;  font-size: %(font-body)s; font-weight: 400; }
QLabel, QCheckBox { background: transparent; }
QCheckBox { spacing: 6px; }
QCheckBox::indicator { width: 14px; height: 14px; background: %(control-bg)s; border: 1px solid %(control-border)s; border-radius: 3px; }
QCheckBox::indicator:checked { background: %(primary-bg)s; border-color: %(primary-bg)s; image: url("%(check-icon)s"); }
QCheckBox::indicator:disabled { border-color: %(border-subtle)s; }

QGroupBox { background: %(bg-sidebar)s; border: 1px solid %(border-subtle)s; border-radius: 6px; margin-top: 8px; padding: 6px; font-weight: 400; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; font-weight: 600; }
QPushButton { background: %(control-bg)s; color: %(text-main)s; border: 1px solid %(control-border)s; border-radius: 4px; padding: 3px 8px; font-weight: 400; }
QPushButton:hover { background: %(hover-bg)s; border-color: %(border-active)s; }
QPushButton:pressed, QPushButton[ctActive="true"] { background: %(selected-bg)s; color: %(selected-text)s; border-color: %(selected-border)s; }
QPushButton[ctPrimary="true"] { background: %(primary-bg)s; color: %(primary-text)s; border-color: %(primary-bg)s; }
QWidget:disabled { color: %(text-dim)s; }
QPushButton:disabled { background: %(bg-card)s; color: %(text-dim)s; border-color: %(border-subtle)s; }
QComboBox, QLineEdit, QSpinBox { background: %(control-bg)s; color: %(text-main)s; border: 1px solid %(control-border)s; border-radius: 4px; padding: 3px 6px; }
QComboBox QAbstractItemView, QMenu { background: %(dialog-bg)s; color: %(text-main)s; selection-background-color: %(selected-bg)s; selection-color: %(selected-text)s; }
QTreeWidget {  font-size: %(font-body)s; font-weight: normal; background: %(bg-sidebar)s; alternate-background-color: %(bg-card)s; border: 1px solid %(border-subtle)s; border-radius: 6px; outline: none; }
QTreeWidget::item { height: 30px; padding: 0 4px; }
QTreeWidget::item:hover { background: %(hover-bg)s; }
QTreeWidget::item:selected { background: %(selected-bg)s; color: %(selected-text)s; border: 1px solid %(selected-border)s; }
QHeaderView::section { background: %(header-bg)s; color: %(text-muted)s; border: none; padding: 5px 6px; }
QProgressBar { background: %(track-bg)s; color: %(text-main)s; border: 1px solid %(control-border)s; border-radius: 4px; text-align: center; }
QProgressBar::chunk { background: %(border-active)s; }
QSlider::groove:horizontal { background: %(track-bg)s; height: 5px; }
QSlider::handle:horizontal { background: %(border-active)s; width: 12px; margin: -5px 0; border-radius: 5px; }
QSplitter::handle { background: %(border-subtle)s; }
QScrollBar { background: %(bg-sidebar)s; }
QScrollBar:vertical { width: 11px; }
QScrollBar:horizontal { height: 11px; }
QScrollBar::handle { background: %(control-border)s; border-radius: 4px; min-height: 24px; min-width: 24px; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QToolTip { background: %(dialog-bg)s; color: %(text-main)s; border: 1px solid %(control-border)s; padding: 5px; }
QFrame#shortcutsCard { background: %(bg-card)s; border: 1px solid %(border-subtle)s; border-radius: 8px; }
QLabel#keyBadge, QLabel#mouseBadge { background: %(control-bg)s; color: %(text-main)s; border: 1px solid %(control-border)s; }
""" % p


def apply_theme(name, persist=True):
    global _current
    _current = resolve_theme(name)
    if persist:
        _SETTINGS().setValue("app_theme", _current)
    app = QApplication.instance()
    if app is None:
        return
    p = QPalette()
    roles = {"Window":"bg-app", "WindowText":"text-main", "Base":"bg-sidebar", "AlternateBase":"bg-card", "Text":"text-main", "Button":"control-bg", "ButtonText":"text-main", "Highlight":"selected-bg", "HighlightedText":"selected-text", "ToolTipBase":"dialog-bg", "ToolTipText":"text-main", "Link":"border-active", "Light":"bg-card", "Midlight":"control-bg", "Dark":"control-border", "Mid":"control-border", "Shadow":"control-border", "BrightText":"primary-text"}
    for role, key in roles.items():
        p.setColor(getattr(QPalette.ColorRole, role), QColor(color(key)))
    p.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(color("text-dim")))
    p.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(color("text-dim")))
    ui_font = QFont(font_family())
    ui_font.setPointSizeF(SIZES["body"])
    ui_font.setWeight(QFont.Weight.Normal)
    ui_font.setLetterSpacing(QFont.SpacingType.PercentageSpacing, 100)
    ui_font.setWordSpacing(0)
    ui_font.setKerning(True)
    app.setFont(ui_font)
    QToolTip.setFont(ui_font)
    app.setPalette(p)
    app.setStyleSheet(stylesheet())
    for widget in app.allWidgets():
        template = widget.property("ctStyleTemplate")
        if template is not None:
            widget.setStyleSheet(render_style(template))
        widget.update()
