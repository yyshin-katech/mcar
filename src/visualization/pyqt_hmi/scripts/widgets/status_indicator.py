#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt

class StatusIndicator(QLabel):
    """센서 상태 표시 위젯"""
    
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.status = 2  # 0: 정상, 1: 경고, 2: 에러
        self.setFixedSize(120, 40)
        self.update_display()
        
    def set_status(self, status):
        self.status = status
        self.update_display()
        
    def update_display(self):
        if self.status == 0:  # 정상
            bg_color = "#28a745"
            text_color = "white"
        elif self.status == 1:  # 경고
            bg_color = "#ff8c00"
            text_color = "white"
        else:  # 에러
            bg_color = "#dc3545"
            text_color = "white"
            
        self.setStyleSheet(f"""
            QLabel {{
                background-color: {bg_color};
                color: {text_color};
                border: 2px solid #333;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
                padding: 5px;
            }}
        """)
        self.setText(self.name)
        self.setAlignment(Qt.AlignCenter)