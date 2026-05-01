# -*- coding: utf-8 -*-
"""Label + value tile and a 2×N stat grid."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFrame, QGridLayout, QLabel, QVBoxLayout

from utils.theme import display_font, labeled_mono_font, mono_font


class StatCard(QFrame):
    def __init__(self, label, value="—", unit="", parent=None):
        super().__init__(parent)
        self.setProperty("role", "card")

        v = QVBoxLayout(self)
        v.setContentsMargins(12, 10, 12, 10)
        v.setSpacing(2)

        self._k = QLabel(label.upper())
        self._k.setProperty("role", "stat-k")
        self._k.setFont(labeled_mono_font(9, letter_spacing=2))
        self._k.setAlignment(Qt.AlignLeft)

        self._v = QLabel(str(value))
        self._v.setProperty("role", "stat-v")
        self._v.setFont(display_font(20, weight=500))
        self._v.setAlignment(Qt.AlignLeft)

        self._u = QLabel(unit)
        self._u.setProperty("role", "stat-unit")
        self._u.setFont(mono_font(9))
        self._u.setAlignment(Qt.AlignLeft)

        v.addWidget(self._k)
        v.addWidget(self._v)
        if unit:
            v.addWidget(self._u)
        else:
            self._u.hide()

    def set_value(self, value):
        self._v.setText(str(value))

    def set_unit(self, unit):
        if unit:
            self._u.setText(unit)
            self._u.show()
        else:
            self._u.hide()


class StatGrid(QFrame):
    """Container that holds named StatCards in a 2-column grid."""

    def __init__(self, parent=None, columns=2):
        super().__init__(parent)
        self._cards = {}
        self._cols = columns
        self._row = 0
        self._col = 0
        self._grid = QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(8)
        self._grid.setVerticalSpacing(8)

    def add(self, key, label, value="—", unit=""):
        card = StatCard(label, value=value, unit=unit)
        self._cards[key] = card
        self._grid.addWidget(card, self._row, self._col)
        self._col += 1
        if self._col >= self._cols:
            self._col = 0
            self._row += 1
        return card

    def card(self, key):
        return self._cards.get(key)

    def set(self, key, value, unit=None):
        c = self._cards.get(key)
        if c is None:
            return
        c.set_value(value)
        if unit is not None:
            c.set_unit(unit)
