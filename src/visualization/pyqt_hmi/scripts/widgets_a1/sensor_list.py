# -*- coding: utf-8 -*-
"""Sensor / topic health list — small LED dot + topic name + Hz."""
import time
from collections import deque

from PyQt5.QtCore import Qt, QSize, QTimer
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import QFrame, QGridLayout, QLabel, QSizePolicy, QWidget

from utils.theme import (BG_2, GREEN, LINE, RED, TEXT_2, TEXT_3, AMBER_0,
                         labeled_mono_font, mono_font)


class LedDot(QWidget):
    def __init__(self, parent=None, size=10):
        super().__init__(parent)
        self._size = size
        self._status = 2  # 0=ok, 1=warn, 2=error
        self.setFixedSize(QSize(size, size))

    def set_status(self, status):
        s = int(status) if status is not None else 2
        if s != self._status:
            self._status = s
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self._status == 0:
            color = GREEN
        elif self._status == 1:
            color = AMBER_0
        else:
            color = RED
        p.setBrush(QBrush(color))
        p.setPen(QPen(LINE, 0.5))
        r = self._size / 2
        p.drawEllipse(0, 0, self._size, self._size)


class TopicMonitor:
    """Track timestamps to compute incoming Hz."""

    def __init__(self, window=2.0):
        self._stamps = deque(maxlen=200)
        self._window = window

    def tick(self, now_sec):
        # drop entries older than window
        cutoff = now_sec - self._window
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()
        self._stamps.append(now_sec)

    def hz(self):
        if len(self._stamps) < 2:
            return 0.0
        dt = self._stamps[-1] - self._stamps[0]
        if dt <= 0:
            return 0.0
        return (len(self._stamps) - 1) / dt


class SensorList(QFrame):
    """Right-panel list of sensor rows. Each row: LED + name + Hz."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setProperty("role", "card")
        self._rows = {}  # key → (led, name_label, hz_label, monitor)

        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(12, 10, 12, 10)
        self._grid.setHorizontalSpacing(10)
        self._grid.setVerticalSpacing(8)

        # Header
        header = QLabel("SENSORS · HZ")
        header.setProperty("role", "section")
        header.setFont(labeled_mono_font(10, 2))
        self._grid.addWidget(header, 0, 0, 1, 3)
        self._next_row = 1

        # Hz refresh timer (5 Hz UI)
        self._refresh = QTimer(self)
        self._refresh.timeout.connect(self._tick_ui)
        self._refresh.start(200)

    def add_row(self, key, name):
        led = LedDot()
        name_label = QLabel(name)
        name_label.setProperty("role", "stat-k")
        name_label.setFont(mono_font(10))
        name_label.setStyleSheet("color: #C8CCD2;")
        hz_label = QLabel("— Hz")
        hz_label.setProperty("role", "stat-k")
        hz_label.setFont(mono_font(10))
        hz_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        hz_label.setStyleSheet("color: #8A9099;")

        self._grid.addWidget(led, self._next_row, 0)
        self._grid.addWidget(name_label, self._next_row, 1)
        self._grid.addWidget(hz_label, self._next_row, 2)

        self._rows[key] = (led, name_label, hz_label, TopicMonitor())
        self._next_row += 1

    def set_status(self, key, status):
        row = self._rows.get(key)
        if row is None:
            return
        row[0].set_status(status)

    def tick(self, key, now_sec=None):
        row = self._rows.get(key)
        if row is None:
            return
        if now_sec is None:
            now_sec = time.monotonic()
        row[3].tick(now_sec)

    def _tick_ui(self):
        for _, _, hz_label, monitor in self._rows.values():
            hz = monitor.hz()
            if hz <= 0.05:
                hz_label.setText("—  Hz")
            else:
                hz_label.setText(f"{hz:5.1f} Hz")
