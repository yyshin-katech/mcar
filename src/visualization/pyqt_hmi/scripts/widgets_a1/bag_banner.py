# -*- coding: utf-8 -*-
"""REC banner with pulsing red dot."""
from PyQt5.QtCore import (QEasingCurve, QPropertyAnimation, Qt, pyqtSignal)
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (QFrame, QGraphicsOpacityEffect, QHBoxLayout,
                             QLabel, QPushButton, QWidget)

from utils.theme import RED, labeled_mono_font, mono_font


class _PulseDot(QWidget):
    def __init__(self, parent=None, size=10):
        super().__init__(parent)
        self._size = size
        self.setFixedSize(size, size)
        self._effect = QGraphicsOpacityEffect(self)
        self._effect.setOpacity(1.0)
        self.setGraphicsEffect(self._effect)
        self._anim = QPropertyAnimation(self._effect, b"opacity", self)
        self._anim.setStartValue(0.4)
        self._anim.setEndValue(1.0)
        self._anim.setDuration(900)
        self._anim.setLoopCount(-1)
        self._anim.setEasingCurve(QEasingCurve.InOutSine)

    def start(self):
        self._anim.start()

    def stop(self):
        self._anim.stop()
        self._effect.setOpacity(1.0)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(RED))
        p.setPen(QPen(QColor(0, 0, 0, 0)))
        p.drawEllipse(0, 0, self._size, self._size)


class BagBanner(QFrame):
    """Slim REC banner: pulsing dot + label + toggle button."""

    toggled = pyqtSignal(bool)  # True when starting record

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setProperty("role", "card")
        self._recording = False

        h = QHBoxLayout(self)
        h.setContentsMargins(12, 8, 12, 8)
        h.setSpacing(10)

        self._dot = _PulseDot(size=10)
        self._dot.hide()

        self._label = QLabel("READY")
        self._label.setProperty("role", "stat-k")
        self._label.setFont(labeled_mono_font(10, 2))
        self._label.setStyleSheet("color: #C8CCD2;")

        self._info = QLabel("")
        self._info.setFont(mono_font(9))
        self._info.setStyleSheet("color: #5A616B;")
        self._info.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self._btn = QPushButton("REC")
        self._btn.setProperty("variant", "primary")
        self._btn.setFixedWidth(72)
        self._btn.clicked.connect(self._on_btn_clicked)

        h.addWidget(self._dot)
        h.addWidget(self._label)
        h.addWidget(self._info, 1)
        h.addWidget(self._btn)

    def _on_btn_clicked(self):
        target = not self._recording
        self.toggled.emit(target)

    def set_recording(self, on, info_text=""):
        self._recording = bool(on)
        if self._recording:
            self._dot.show()
            self._dot.start()
            self._label.setText("REC")
            self._label.setStyleSheet("color: #F87171;")
            self._btn.setText("STOP")
            self._btn.setProperty("variant", "danger")
        else:
            self._dot.stop()
            self._dot.hide()
            self._label.setText("READY")
            self._label.setStyleSheet("color: #C8CCD2;")
            self._btn.setText("REC")
            self._btn.setProperty("variant", "primary")
        self._info.setText(info_text or "")
        self._btn.style().unpolish(self._btn); self._btn.style().polish(self._btn)
