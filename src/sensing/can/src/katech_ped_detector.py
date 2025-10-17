#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from typing import List, Tuple, Dict, Optional

from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg
from perception_ros_msg.msg import object_array_msg, object_msg
from sensor_msgs.msg import NavSatFix
from katech_custom_msgs.msg import ped_crosswalk_check_msg, ped_crosswalk_check_array_msg

class ObjectArray:
    def __init__(self):
        self.header = Header()
        self.data = []  # ObjectMsg 리스트

class Crosswalk:
    """단일 횡단보도 클래스"""
    def __init__(self, crosswalk_id, coords):
        self.id = crosswalk_id
        self.coords = coords
        
        # 경계 상자 계산
        x_coords = [coord[0] for coord in coords]
        y_coords = [coord[1] for coord in coords]
        
        self.min_x = min(x_coords)
        self.max_x = max(x_coords)
        self.min_y = min(y_coords)
        self.max_y = max(y_coords)
        
        # 중심점 계산
        self.center_x = (self.min_x + self.max_x) / 2
        self.center_y = (self.min_y + self.max_y) / 2
        self.width = self.max_x - self.min_x
        self.height = self.max_y - self.min_y
    
    def point_in_rectangle(self, point_x, point_y):
        """점이 횡단보도 내부에 있는지 판단"""
        return (self.min_x <= point_x <= self.max_x and 
                self.min_y <= point_y <= self.max_y)
    
    def distance_to_point(self, point_x, point_y):
        """점과 횡단보도 중심 간의 거리"""
        return math.sqrt((point_x - self.center_x)**2 + (point_y - self.center_y)**2)

class ROSCrosswalkDetector:
    """ROS1 기반 다중 횡단보도 검출 시스템"""
    
    def __init__(self):
        # ROS1 노드 초기화
        rospy.init_node('crosswalk_detector', anonymous=True)
        
        # 횡단보도 관리
        self.crosswalks = {}
        self.initialize_crosswalks()
        
        # 자차 상태 변수
        self.host_east = 0.0      # global X [m]
        self.host_north = 0.0     # global Y [m] 
        self.host_yaw = 0.0       # global yaw [rad]
        self.host_data_updated = False
        
        # ROS1 구독자 생성
        self.host_subscriber = rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, self.host_status_callback, queue_size=10)
        self.object_subscriber = rospy.Subscriber('/track_Multi_RS', object_array_msg, self.object_array_callback, queue_size=10)

        self.target_pub = rospy.Publisher('/katech_msg/crosswalk_detection', ped_crosswalk_check_array_msg, queue_size=10)

        # 결과 발행자
        # self.result_publisher = rospy.Publisher(
        #     '/crosswalk_detection_result',
        #     String,
        #     queue_size=10
        # )
        
        # rospy.loginfo('횡단보도 검출 시스템이 시작되었습니다.')
        # rospy.loginfo('등록된 횡단보도 수: %d개', len(self.crosswalks))
    
    def initialize_crosswalks(self):
        """횡단보도 초기화 (1번~9번)"""
        crosswalk_data = {
            1: [(935600.0379, 1916121.555), (935600.2985, 1916108.973), (935594.0335, 1916121.555), (935594.2717, 1916108.973)],
            2: [(935581.0876, 1916230.353), (935588.4297, 1916230.578), (935581.0876, 1916224.326), (935588.4714, 1916224.562)],
            3: [(935580.6891, 1916240.245), (935587.546, 1916240.337), (935580.6265, 1916246.25), (935587.546, 1916246.313)],
            4: [(935578.4427, 1915927.589), (935571.9538, 1915931.057), (935580.7854, 1915933.171), (935573.2702, 1915937.154)],
            5: [(935599.946, 1915939.65), (935597.5507, 1915935.164), (935588.9097, 1915936.14), (935594.1919, 1915945.875)],
            6: [(935613.0087, 1915982.558), (935615.2914, 1915987.105), (935618.5076, 1915980.324), (935620.8349, 1915984.809)],
            7: [(935607.1913, 1916003.465), (935615.7084, 1915998.892), (935603.9119, 1915996.147), (935612.4462, 1915991.533)],
            8: [(935573.2702, 1915937.154), (935574.2511, 1915938.917), (935582.9325, 1915934.242), (935582.0232, 1915932.518)],
            9: [(935590.6614, 1915935.176), (935588.9097, 1915936.14), (935594.1919, 1915945.875), (935595.952, 1915944.927)]
        }
        
        for crosswalk_id, coords in crosswalk_data.items():
            self.add_crosswalk(crosswalk_id, coords)
    
    def add_crosswalk(self, crosswalk_id, coords):
        """횡단보도 추가"""
        crosswalk = Crosswalk(crosswalk_id, coords)
        self.crosswalks[crosswalk_id] = crosswalk
        # rospy.loginfo(
        #     '횡단보도 %d번 등록: 중심(%.1f, %.1f)', 
        #     crosswalk_id, crosswalk.center_x, crosswalk.center_y
        # )
    
    def host_status_callback(self, msg):
        """자차 상태 토픽 콜백"""
        self.host_east = msg.host_east
        self.host_north = msg.host_north
        self.host_yaw = msg.host_yaw
        self.host_data_updated = True
        
        # 디버그 로그 (필요시 주석 해제)
        # rospy.logdebug(
        #     '자차 위치 업데이트: (%.1f, %.1f), 방향: %.1f도',
        #     self.host_east, self.host_north, math.degrees(self.host_yaw)
        # )
    
    def object_array_callback(self, msg):
        ped_msg = ped_crosswalk_check_msg()
        pub_msg_ar = ped_crosswalk_check_array_msg()

        """오브젝트 배열 토픽 콜백"""
        if not self.host_data_updated:
            rospy.logwarn('자차 위치 정보가 아직 수신되지 않았습니다.')
            return
        
        # status가 1인 오브젝트만 처리
        # active_objects = [obj for obj in msg.data if obj.status == 1]
        active_objects = [obj for obj in msg.data if obj.status in [1, 2]]
        
        if not active_objects:
            ped_msg.id = 0
            ped_msg.status = 0
            ped_msg.on_crosswalk = 0
            ped_msg.rel_pos_x = 0
            ped_msg.rel_pos_y = 0

            pub_msg_ar.data.append(ped_msg)
        
        # rospy.loginfo('활성 오브젝트 %d개 처리 중...', len(active_objects))
        
        # detection_results = []
        else:
            for obj in active_objects:
                # 오브젝트가 위치한 횡단보도들 찾기
                crosswalk_ids, abs_pos = self.find_crosswalks_containing_object(
                    obj.x, obj.y
                )
                
                # 로깅
                if crosswalk_ids:
                    # rospy.loginfo(
                    #     '오브젝트 %d: 횡단보도 %s번 위에 있음 (절대위치: %.1f, %.1f)',
                    #     obj.id, str(crosswalk_ids), abs_pos[0], abs_pos[1]
                    # )
                    ped_msg.id = obj.id
                    ped_msg.status = obj.status
                    ped_msg.on_crosswalk = 1
                    ped_msg.rel_pos_x = obj.x
                    ped_msg.rel_pos_y = obj.y

                    pub_msg_ar.data.append(ped_msg)

                else:
                    # 가장 가까운 횡단보도 찾기
                    nearest_id, distance = self.find_nearest_crosswalk(
                        obj.x, obj.y
                    )
        
        # 결과 발행
        pub_msg_ar.time = rospy.Time.now()
        self.target_pub.publish(pub_msg_ar)
        

        # if detection_results:
            # self.publish_detection_results(detection_results)
        # rospy.loginfo(pub_msg_ar)

        pub_msg_ar.data.clear()

    def rotate_point(self, x, y, yaw_rad):
        """점을 yaw각도만큼 회전"""
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        
        x_rot = x * cos_yaw - y * sin_yaw
        y_rot = x * sin_yaw + y * cos_yaw
        
        return x_rot, y_rot
    
    def relative_to_absolute(self, relative_x, relative_y):
        """자차 기준 상대좌표를 절대좌표로 변환"""
        # 상대좌표를 자차의 yaw만큼 회전
        rotated_x, rotated_y = self.rotate_point(relative_x, relative_y, self.host_yaw)
        
        # 자차 위치만큼 평행이동
        absolute_x = self.host_east + rotated_x
        absolute_y = self.host_north + rotated_y
        
        return absolute_x, absolute_y
    
    def find_crosswalks_containing_object(self, relative_x, relative_y):
        """오브젝트가 위치한 모든 횡단보도 찾기"""
        # 상대좌표를 절대좌표로 변환
        abs_x, abs_y = self.relative_to_absolute(relative_x, relative_y)
        
        # 각 횡단보도에 대해 검사
        containing_crosswalks = []
        
        for crosswalk_id, crosswalk in self.crosswalks.items():
            if crosswalk.point_in_rectangle(abs_x, abs_y):
                containing_crosswalks.append(crosswalk_id)
        
        return containing_crosswalks, (abs_x, abs_y)
    
    def find_nearest_crosswalk(self, relative_x, relative_y):
        """오브젝트에서 가장 가까운 횡단보도 찾기"""
        if not self.crosswalks:
            return None, float('inf')
        
        abs_x, abs_y = self.relative_to_absolute(relative_x, relative_y)
        
        nearest_id = None
        min_distance = float('inf')
        
        for crosswalk_id, crosswalk in self.crosswalks.items():
            distance = crosswalk.distance_to_point(abs_x, abs_y)
            if distance < min_distance:
                min_distance = distance
                nearest_id = crosswalk_id
        
        return nearest_id, min_distance
    
    # def publish_detection_results(self, results):
    #     """검출 결과 발행"""
    #     result_json = json.dumps(results, indent=2)
    #     msg = String()
    #     msg.data = result_json
    #     self.result_publisher.publish(msg)
    
    def run(self):
        """메인 실행 루프"""
        rospy.loginfo('횡단보도 검출 시스템이 실행 중입니다...')
        
        # 시스템 정보 출력
        rospy.loginfo('등록된 횡단보도 목록:')
        for crosswalk_id in sorted(self.crosswalks.keys()):
            crosswalk = self.crosswalks[crosswalk_id]
            rospy.loginfo(
                '  %d번: 중심(%.1f, %.1f), 크기(%.1fx%.1f)',
                crosswalk_id, crosswalk.center_x, crosswalk.center_y,
                crosswalk.width, crosswalk.height
            )
        
        # ROS 스핀
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo('시스템이 종료됩니다.')

def main():
    """메인 함수"""
    try:
        # 노드 생성 및 실행
        crosswalk_detector = ROSCrosswalkDetector()
        crosswalk_detector.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo('프로그램이 중단되었습니다.')
    except Exception as e:
        rospy.logerr('오류 발생: %s', str(e))

if __name__ == '__main__':
    main()
