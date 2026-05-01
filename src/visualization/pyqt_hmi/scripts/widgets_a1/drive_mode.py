# -*- coding: utf-8 -*-
"""P / R / N / D 4-cell gear toggle."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from utils.theme import display_font


_GEAR_MAP = {1: "P", 2: "R", 3: "N", 4: "D"}
_GEAR_ORDER = ["P", "R", "N", "D"]


class DriveModeRow(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._cells = {}
        self._labels = {}
        self._active = "P"

        self.setProperty("role", "section-row")

        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(8)
        for letter in _GEAR_ORDER:
            cell = QFrame()
            cell.setProperty("role", "gear-cell")
            cell.setMinimumHeight(56)
            cv = QVBoxLayout(cell)
            cv.setContentsMargins(0, 0, 0, 0)
            label = QLabel(letter)
            label.setAlignment(Qt.AlignCenter)
            label.setProperty("role", "gear-text")
            label.setFont(display_font(18, weight=500))
            cv.addWidget(label)
            self._cells[letter] = cell
            self._labels[letter] = label
            h.addWidget(cell, 1)
        self._apply_active()

    def set_gear(self, gear_status):
        letter = _GEAR_MAP.get(int(gear_status) if gear_status is not None else 0)
        if letter == self._active:
            return
        self._active = letter or ""
        self._apply_active()

    def _apply_active(self):
        for letter, cell in self._cells.items():
            is_on = (letter == self._active)
            cell.setProperty("active", "true" if is_on else "false")
            label = self._labels[letter]
            label.setProperty("active", "true" if is_on else "false")
            cell.style().unpolish(cell); cell.style().polish(cell)
            label.style().unpolish(label); label.style().polish(label)
