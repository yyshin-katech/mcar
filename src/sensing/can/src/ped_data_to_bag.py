#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import csv
from katech_custom_msgs.msg import ped_crosswalk_check_msg, ped_crosswalk_check_array_msg

class PedCrosswalkLogger:
    def __init__(self):
        # CSV 파일 설정
        self.csv_file = open('/home/yuyeong/ped_crosswalk_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV 헤더 작성
        self.csv_writer.writerow(['timestamp', 'id', 'status', 'on_crosswalk', 'rel_pos_x', 'rel_pos_y'])
        
        # Subscriber 설정
        rospy.Subscriber('/katech_msg/crosswalk_detection', ped_crosswalk_check_array_msg, self.callback)
        
        rospy.loginfo("Pedestrian crosswalk logger started")
    
    def callback(self, msg):
        timestamp = msg.time.to_sec()  # 또는 rospy.Time.now().to_sec()
        
        # 배열의 각 요소를 개별 행으로 저장
        for ped in msg.data:
            row = [
                timestamp,
                ped.id,
                ped.status,
                ped.on_crosswalk,
                ped.rel_pos_x,
                ped.rel_pos_y
            ]
            self.csv_writer.writerow(row)
        
        self.csv_file.flush()  # 즉시 파일에 쓰기
    
    def shutdown(self):
        self.csv_file.close()
        rospy.loginfo("CSV file closed")

if __name__ == '__main__':
    rospy.init_node('ped_crosswalk_logger')
    
    logger = PedCrosswalkLogger()
    rospy.on_shutdown(logger.shutdown)
    
    rospy.spin()