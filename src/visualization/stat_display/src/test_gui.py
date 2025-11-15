#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
import tkinter as tk

class ButtonGUI:
    def __init__(self):
        rospy.init_node('button_gui', anonymous=True)
        
        self.button1_pub = rospy.Publisher('/button/button1_click', Bool, queue_size=1)
        self.button2_pub = rospy.Publisher('/button/button2_click', Bool, queue_size=1)
        
        self.button1_state = False
        self.button2_state = False
        
        # GUI 생성
        self.root = tk.Tk()
        self.root.title("Control Buttons")
        self.root.geometry("300x200")
        
        # 버튼1
        self.btn1 = tk.Button(
            self.root, 
            text="Button 1: OFF", 
            command=self.toggle_button1,
            width=20,
            height=3,
            bg='gray'
        )
        self.btn1.pack(pady=20)
        
        # 버튼2
        self.btn2 = tk.Button(
            self.root, 
            text="Button 2: OFF", 
            command=self.toggle_button2,
            width=20,
            height=3,
            bg='gray'
        )
        self.btn2.pack(pady=20)
        
    def toggle_button1(self):
        self.button1_state = not self.button1_state
        msg = Bool()
        msg.data = self.button1_state
        self.button1_pub.publish(msg)
        
        # 버튼 색상 변경
        if self.button1_state:
            self.btn1.config(text="Button 1: ON", bg='green')
        else:
            self.btn1.config(text="Button 1: OFF", bg='gray')
            
    def toggle_button2(self):
        self.button2_state = not self.button2_state
        msg = Bool()
        msg.data = self.button2_state
        self.button2_pub.publish(msg)
        
        # 버튼 색상 변경
        if self.button2_state:
            self.btn2.config(text="Button 2: ON", bg='blue')
        else:
            self.btn2.config(text="Button 2: OFF", bg='gray')
    
    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    try:
        gui = ButtonGUI()
        gui.run()
    except rospy.ROSInterruptException:
        pass