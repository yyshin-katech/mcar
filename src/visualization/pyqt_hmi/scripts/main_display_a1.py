#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Entry point for Ioniq5 HMI A-1 (Solid Top-down · Amber)."""
import os
import signal
import sys

import rospy

# Resolve PYTHONPATH so widgets_a1 / utils / widgets are importable
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# Ctrl-C must be handled by Python *before* QApplication grabs SIGINT
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PyQt5.QtCore import QTimer  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from utils.hmi_state import HmiStateController  # noqa: E402
from utils.theme import load_application_fonts  # noqa: E402
from widgets_a1.main_window_a1 import MainWindowA1  # noqa: E402


def _resolve_qss_path():
    """resources/qss/app.qss lives one level up from scripts/."""
    candidate = os.path.normpath(
        os.path.join(_SCRIPT_DIR, "..", "resources", "qss", "app.qss"))
    return candidate if os.path.exists(candidate) else None


def _resolve_fonts_dir():
    candidate = os.path.normpath(
        os.path.join(_SCRIPT_DIR, "..", "resources", "fonts"))
    return candidate if os.path.isdir(candidate) else None


def main():
    rospy.init_node("ioniq5_hmi_a1", anonymous=False, disable_signals=True)

    app = QApplication(sys.argv)

    # let the Python interpreter wake up periodically so SIGINT lands quickly
    interrupt_timer = QTimer()
    interrupt_timer.timeout.connect(lambda: None)
    interrupt_timer.start(100)

    fonts_dir = _resolve_fonts_dir()
    if fonts_dir:
        load_application_fonts(fonts_dir)

    qss_path = _resolve_qss_path()
    if qss_path:
        with open(qss_path) as f:
            app.setStyleSheet(f.read())
        rospy.loginfo("HMI A-1: loaded QSS from %s", qss_path)
    else:
        rospy.logwarn("HMI A-1: QSS not found, default styling will apply")

    controller = HmiStateController()

    # optional map path via param (~map_file)
    map_path = rospy.get_param(
        "~map_file",
        "/home/ads/mcar_v13/src/localization/gps_system_localizer/src/A2_LINK_epsg5179.shp",
    )

    # optional bag dir via param
    controller.set_bag_dir(rospy.get_param("~bag_dir", os.path.expanduser("~/bag_data")))

    window = MainWindowA1(controller)
    if os.path.exists(map_path):
        window.vehicle_view.load_map(map_path)
        rospy.loginfo("HMI A-1: map loaded: %s", map_path)
    else:
        rospy.logwarn("HMI A-1: map file not found: %s (override with ~map_file)", map_path)

    # show maximized if screen smaller than design canvas
    screen = app.primaryScreen().availableGeometry()
    if screen.width() < 1600 or screen.height() < 900:
        window.showMaximized()
    else:
        window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nIoniq5 HMI A-1 terminated.")
        sys.exit(0)
