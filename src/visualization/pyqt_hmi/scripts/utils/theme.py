# -*- coding: utf-8 -*-
"""Design tokens for Ioniq5 HMI A-1 (Solid top-down · Amber)."""
import os
from PyQt5.QtGui import QColor, QFont, QFontDatabase

# ─── colors ─────────────────────────────────────────────────────────
BG_0    = QColor("#0E0F11")
BG_1    = QColor("#15171A")
BG_2    = QColor("#1C1F23")
BG_3    = QColor("#232830")
LINE    = QColor("#232830")
TEXT_0  = QColor("#F4F5F7")
TEXT_1  = QColor("#C8CCD2")
TEXT_2  = QColor("#8A9099")
TEXT_3  = QColor("#5A616B")
AMBER_0 = QColor("#FFB547")
AMBER_1 = QColor("#F59E2C")
AMBER_2 = QColor("#C97A14")
RED     = QColor("#F87171")
GREEN   = QColor("#34D399")
BLUE    = QColor("#60A5FA")
GLASS   = QColor("#15171A")

# ─── geometry ───────────────────────────────────────────────────────
WIN_W, WIN_H = 1600, 900
TOPBAR_H = 64
BOTTOM_H = 88
LEFT_W   = 360
RIGHT_W  = 360
RADIUS   = 10


# ─── fonts (with fallback) ──────────────────────────────────────────
_DISPLAY_CANDIDATES = ["Inter", "Pretendard", "Noto Sans CJK KR", "Noto Sans"]
_MONO_CANDIDATES    = ["JetBrains Mono", "DejaVu Sans Mono", "Monospace"]


def load_application_fonts(fonts_dir):
    """Optionally load .ttf/.otf files dropped into resources/fonts/."""
    if not fonts_dir or not os.path.isdir(fonts_dir):
        return
    for name in os.listdir(fonts_dir):
        if name.lower().endswith((".ttf", ".otf")):
            QFontDatabase.addApplicationFont(os.path.join(fonts_dir, name))


def _pick(candidates, default):
    db = QFontDatabase()
    families = set(db.families())
    for fam in candidates:
        if fam in families:
            return fam
    return default


def display_font(size=14, weight=QFont.Normal):
    fam = _pick(_DISPLAY_CANDIDATES, "Sans Serif")
    f = QFont(fam, size, weight)
    f.setStyleStrategy(QFont.PreferAntialias)
    return f


def mono_font(size=12, weight=QFont.Normal):
    fam = _pick(_MONO_CANDIDATES, "Monospace")
    f = QFont(fam, size, weight)
    f.setStyleHint(QFont.TypeWriter)
    return f


def labeled_mono_font(size=10, letter_spacing=2):
    f = mono_font(size, QFont.Medium)
    f.setLetterSpacing(QFont.AbsoluteSpacing, letter_spacing)
    return f
