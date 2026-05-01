# -*- coding: utf-8 -*-
"""Standalone demo — run the A-1 window without ROS, fed by a fake controller.

Usage (from scripts/ dir):
    python3 widgets_a1/_demo.py
"""
import math
import os
import random
import sys
import time

# When invoked as `python3 widgets_a1/_demo.py`, parent dir (scripts/) needs
# to be on sys.path so `widgets_a1.*` and `utils.*` resolve.
_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.dirname(_HERE)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import signal  # noqa: E402

signal.signal(signal.SIGINT, signal.SIG_DFL)

from PyQt5.QtCore import QObject, QTimer, pyqtSignal  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from utils.theme import load_application_fonts  # noqa: E402
from widgets_a1.main_window_a1 import MainWindowA1  # noqa: E402


class FakeController(QObject):
    """Mirrors HmiStateController's signal surface — no rospy."""

    speed_changed       = pyqtSignal(float)
    gear_changed        = pyqtSignal(int)
    mode_changed        = pyqtSignal(int)
    aeb_changed         = pyqtSignal(bool)
    steering_changed    = pyqtSignal(float)
    ego_pose_changed    = pyqtSignal(float, float, float)
    objects_changed     = pyqtSignal(list)
    traffic_changed     = pyqtSignal(int, int)
    diag_changed        = pyqtSignal(dict)
    gps_changed         = pyqtSignal(int, float, float)
    speed_limit_changed = pyqtSignal(int)
    link_lane_changed   = pyqtSignal(object, object)
    odd_changed         = pyqtSignal(int, int)
    popup_changed       = pyqtSignal(str, str)
    bag_state_changed   = pyqtSignal(bool, str)
    topic_event         = pyqtSignal(str)

    DIAG_KEYS = ['gps', 'adcu', 'lidar', 'radar', 'v2x', 'hmi', 'vcu', 'cam', 'ipc']

    def __init__(self):
        super().__init__()
        self._t0 = time.monotonic()
        self._tick = QTimer(self)
        self._tick.timeout.connect(self._step)
        self._tick.start(100)
        self._fake_recording = False

    def request_mode(self, autonomous):
        self.mode_changed.emit(1 if autonomous else 0)

    def toggle_bag(self):
        self._fake_recording = not self._fake_recording
        self.bag_state_changed.emit(self._fake_recording,
                                    "demo-2026-05-01-12-00-00" if self._fake_recording else "")

    def shutdown(self):
        self._tick.stop()

    # ─── 10 Hz fake state stepper ────────────────────────────────
    def _step(self):
        t = time.monotonic() - self._t0
        # speed: triangle wave 0..80
        speed = 40 + 35 * math.sin(t / 6.0)
        self.speed_changed.emit(float(max(0, speed)))

        # AEB: pulse on for 2 s every 30 s
        self.aeb_changed.emit(int(t) % 30 < 2)

        # gear: cycle every 4 s
        gear = (1 + int(t / 4) % 4)
        self.gear_changed.emit(gear)

        # steering oscillation
        self.steering_changed.emit(45 * math.sin(t / 3.0))

        # ego pose: drive in a circle in UTM-ish numbers
        ex = 280000 + 50 * math.cos(t / 8.0)
        en = 4136000 + 50 * math.sin(t / 8.0)
        self.ego_pose_changed.emit(float(ex), float(en), float(t / 8.0))

        # objects orbiting around ego (in ego frame)
        objs = []
        for i in range(3):
            phase = t * 0.7 + i * (2 * math.pi / 3)
            r = 18 + 4 * i
            objs.append({
                'id': 100 + i,
                'x': r * math.cos(phase),
                'y': r * math.sin(phase),
                'width': 1.8,
                'length': 4.5,
                'vx': -r * math.sin(phase) * 0.7,
                'vy':  r * math.cos(phase) * 0.7,
                'orientation': phase + math.pi / 2,
                'type': 'car' if i != 1 else 'pedestrian',
            })
        self.objects_changed.emit(objs)

        # diag: cycle one key into warning
        statuses = {k: 0 for k in self.DIAG_KEYS}
        key = self.DIAG_KEYS[int(t) % len(self.DIAG_KEYS)]
        statuses[key] = 1
        self.diag_changed.emit(statuses)
        for k in self.DIAG_KEYS:
            self.topic_event.emit(k)

        # gps / link / lane / limit / odd
        self.gps_changed.emit(2, 0.012, 0.014)
        self.speed_limit_changed.emit(60)
        self.link_lane_changed.emit(int(t / 5) % 1000 + 1, "L1")
        self.odd_changed.emit(1 if int(t) % 20 < 14 else 0,
                              0 if int(t) % 20 < 18 else 1)

        # traffic: cycle G→A→R every 10 s
        cycle = int(t / 10) % 3
        color = [1, 2, 3][cycle]
        time_left = max(0, 100 - int((t * 10) % 100))
        self.traffic_changed.emit(color, time_left)

        # popup: simple status banner
        if int(t) % 30 < 2:
            self.popup_changed.emit("전방 추돌 경고", "error")
        elif int(t) % 20 < 4:
            self.popup_changed.emit("ODD 이탈 !!!!", "warn")
        else:
            self.popup_changed.emit("", "info")


def main():
    app = QApplication(sys.argv)
    interrupt_timer = QTimer(); interrupt_timer.timeout.connect(lambda: None)
    interrupt_timer.start(100)

    fonts_dir = os.path.normpath(os.path.join(_SCRIPTS, "..", "resources", "fonts"))
    if os.path.isdir(fonts_dir):
        load_application_fonts(fonts_dir)

    qss_path = os.path.normpath(os.path.join(_SCRIPTS, "..", "resources", "qss", "app.qss"))
    if os.path.exists(qss_path):
        with open(qss_path) as f:
            app.setStyleSheet(f.read())

    controller = FakeController()
    window = MainWindowA1(controller)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
