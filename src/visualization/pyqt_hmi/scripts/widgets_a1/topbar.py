# -*- coding: utf-8 -*-
"""Top bar: brand · system tag chips · clock."""
import datetime

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QSizePolicy, QWidget

from utils.theme import TOPBAR_H, display_font, labeled_mono_font, mono_font


class _Chip(QFrame):
    def __init__(self, text, parent=None):
        super().__init__(parent)
        self.setProperty("role", "chip")
        h = QHBoxLayout(self)
        h.setContentsMargins(10, 4, 10, 4)
        h.setSpacing(0)
        self._label = QLabel(text)
        self._label.setFont(labeled_mono_font(9, 2))
        self._label.setStyleSheet("color: #8A9099;")
        h.addWidget(self._label)

    def set_text(self, text):
        self._label.setText(text)

    def set_active(self, active):
        if active:
            self._label.setStyleSheet("color: #FFB547;")
        else:
            self._label.setStyleSheet("color: #8A9099;")


class TopBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("topbar")
        self.setFixedHeight(TOPBAR_H)

        h = QHBoxLayout(self)
        h.setContentsMargins(20, 0, 20, 0)
        h.setSpacing(16)

        # ─ brand ───────────────────────────────────────────────
        brand = QLabel("IONIQ 5  ·  HMI A-1")
        brand.setProperty("role", "brand")
        brand.setFont(display_font(13, weight=600))
        h.addWidget(brand)

        h.addStretch(1)

        # ─ system tags (centered area) ─────────────────────────
        self._chip_odd = _Chip("ODD —")
        self._chip_road = _Chip("ROAD —")
        self._chip_link = _Chip("LINK —")
        self._chip_lane = _Chip("LANE —")
        for c in (self._chip_odd, self._chip_road, self._chip_link, self._chip_lane):
            h.addWidget(c)

        h.addStretch(1)

        # ─ clock ───────────────────────────────────────────────
        self._clock = QLabel("--:--:--")
        self._clock.setProperty("role", "clock")
        self._clock.setFont(mono_font(13, weight=500))
        self._clock.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        h.addWidget(self._clock)

        # 1 Hz clock update
        self._t = QTimer(self)
        self._t.timeout.connect(self._tick_clock)
        self._t.start(500)
        self._tick_clock()

    def _tick_clock(self):
        self._clock.setText(datetime.datetime.now().strftime("%H:%M:%S"))

    def set_odd(self, on_odd, road_state):
        if on_odd:
            self._chip_odd.set_text("ODD ON")
            self._chip_odd.set_active(True)
        else:
            self._chip_odd.set_text("ODD OFF")
            self._chip_odd.set_active(False)
        self._chip_road.set_text(f"ROAD {int(road_state) if road_state is not None else 0}")

    def set_link(self, link_id, lane):
        self._chip_link.set_text(f"LINK {link_id if link_id else '—'}")
        self._chip_lane.set_text(f"LANE {lane if lane else '—'}")
