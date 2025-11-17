#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import os

# 현재 스크립트의 디렉토리를 Python 경로에 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from PyQt5.QtWidgets import QApplication

from widgets.main_window import MainDisplayWindow

def main():
    app = QApplication(sys.argv)
    window = MainDisplayWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n프로그램 종료")
        sys.exit(0)