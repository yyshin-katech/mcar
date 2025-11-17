#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class StariaControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        rospy.init_node('staria_control_panel')
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('Staria Control Pannel')
        self.setGeometry(100, 100, 1600, 900)
        
        # 메인 위젯
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 레이아웃 설정
        main_layout = QHBoxLayout()
        
        # 왼쪽 패널 생성
        left_panel = self.create_left_panel()
        
        # 오른쪽 Object Viewer
        right_panel = self.create_object_viewer()
        
        main_layout.addWidget(left_panel, 70)
        main_layout.addWidget(right_panel, 30)
        
        main_widget.setLayout(main_layout)
        
    def create_left_panel(self):
        # 여기에 모든 컨트롤 생성
        pass
        
    def create_object_viewer(self):
        # 레이더/라이다 뷰어 생성
        pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = StariaControlPanel()
    window.show()
    sys.exit(app.exec_())