#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from typing import List, Tuple, Dict, Optional

from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg
from perception_ros_msg.msg import object_array_msg, object_msg
from sensor_msgs.msg import NavSatFix
from katech_custom_msgs.msg import ped_crosswalk_check_msg, ped_crosswalk_check_array_msg, crosswalk_occupancy_msg

# 크로스워크 -> 접근 LINK_ID 맵 (SSOT).
# 원천: claude_work_list/crosswalk_position.md §134 와 동기화할 것 (crosswalk_ped_fusion.py 와 동일 dict).
# ego 가 CW_LINKS[N] 에 속한 LINK_ID 에 있을 때만 크로스워크 N 을 검사(게이팅)한다.
CW_LINKS = {
    1: {1239, 1238, 1242, 1241},
    2: {1205},
    3: {465, 467, 463},
    4: {417},
    5: {1029, 1025, 1027},
    6: {1092, 1094, 1090, 1091, 1093, 1089},
    7: {1370, 1368, 1372, 1369, 1367, 1371},
    8: {1326, 1325, 1330, 1331},
    9: {1257, 1258},
}

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
        """점이 횡단보도(다각형) 내부에 있는지 판단.

        횡단보도는 사각형이 아닌 임의 다각형일 수 있으므로,
        바운딩 박스로 빠르게 걸러낸 뒤 ray-casting 으로 다각형 내부를 판정한다.
        """
        # 1) 바운딩 박스 빠른 배제
        if not (self.min_x <= point_x <= self.max_x and
                self.min_y <= point_y <= self.max_y):
            return False

        # 2) ray-casting (짝/홀 교차 판정)
        coords = self.coords
        n = len(coords)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = coords[i]
            xj, yj = coords[j]
            if ((yi > point_y) != (yj > point_y)) and \
               (point_x < (xj - xi) * (point_y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside
    
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
        self.host_link_id = 0     # 현재 주행 LINK_ID (크로스워크 게이팅용)
        self.host_data_updated = False
        
        # ROS1 구독자 생성
        self.host_subscriber = rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, self.host_status_callback, queue_size=10)
        self.object_subscriber = rospy.Subscriber('/track_Multi_RS', object_array_msg, self.object_array_callback, queue_size=10)

        self.target_pub = rospy.Publisher('/katech_msg/crosswalk_detection', ped_crosswalk_check_array_msg, queue_size=10)
        self.occupancy_pub = rospy.Publisher('/katech_msg/crosswalk_occupancy', crosswalk_occupancy_msg, queue_size=10)

        # 결과 발행자
        # self.result_publisher = rospy.Publisher(
        #     '/crosswalk_detection_result',
        #     String,
        #     queue_size=10
        # )
        
        # rospy.loginfo('횡단보도 검출 시스템이 시작되었습니다.')
        # rospy.loginfo('등록된 횡단보도 수: %d개', len(self.crosswalks))
    
    def initialize_crosswalks(self):
        """횡단보도 초기화 (senario 260514c, EPSG:5179 다각형)"""
        # WGS84 원좌표(_work_item/crosswalk_position.md)를 EPSG:5179(east, north)로 변환한 값
        crosswalk_data = {
            1: [(930819.312725, 1929593.158143), (930807.530293, 1929580.608639), (930804.537889, 1929582.883314), (930803.754045, 1929582.693963), (930800.736230, 1929585.001303), (930798.147151, 1929585.252169), (930814.166390, 1929602.326491), (930814.430028, 1929599.606956), (930813.356668, 1929598.462133), (930816.299566, 1929596.210116), (930816.286256, 1929595.495250)],
            2: [(930819.933061, 1929617.498679), (930817.484334, 1929614.491229), (930817.695865, 1929613.842349), (930815.283504, 1929610.895521), (930813.983369, 1929609.298809), (930788.698959, 1929628.204441), (930789.932348, 1929629.623349), (930790.729092, 1929629.069490), (930793.150823, 1929631.994565), (930792.937083, 1929632.723870), (930795.460264, 1929635.651967)],
            3: [(931618.480129, 1928668.245223), (931602.430990, 1928680.837104), (931604.756398, 1928683.910106), (931604.535801, 1928684.329460), (931606.966005, 1928687.602070), (931606.359503, 1928688.086279), (931607.641929, 1928689.732141), (931625.739121, 1928675.649309), (931624.307032, 1928673.985063), (931623.131051, 1928674.888081), (931620.632328, 1928671.712353), (931620.838390, 1928671.200786)],
            4: [(931746.802920, 1928832.583676), (931731.083283, 1928844.947864), (931732.751645, 1928847.093707), (931731.718782, 1928848.551824), (931733.372971, 1928850.708445), (931731.845245, 1928851.916781), (931731.955025, 1928853.249105), (931733.455832, 1928853.860780), (931753.293170, 1928838.226088), (931753.572679, 1928835.924257), (931752.549467, 1928835.643084), (931750.919243, 1928836.947748), (931749.116990, 1928834.923729), (931748.438488, 1928834.717009)],
            5: [(931584.965262, 1928973.697114), (931581.771173, 1928976.155563), (931581.323971, 1928975.925517), (931578.234135, 1928978.366626), (931585.070821, 1928987.307505), (931587.654537, 1928990.733507), (931592.420497, 1928996.805367), (931595.503240, 1928994.370156), (931596.434902, 1928995.316672), (931599.639435, 1928992.892383), (931593.808205, 1928985.349048), (931591.235266, 1928981.811252)],
            6: [(931483.574338, 1929056.144024), (931480.594260, 1929058.534888), (931479.977918, 1929058.443113), (931477.062238, 1929060.808009), (931476.218800, 1929059.782930), (931473.932140, 1929060.184437), (931490.058985, 1929080.188544), (931491.167411, 1929078.441717), (931490.336670, 1929077.309865), (931493.266962, 1929074.906220), (931493.902490, 1929075.049275), (931496.819689, 1929072.651420)],
            7: [(931280.158279, 1929216.475039), (931277.153107, 1929218.935474), (931276.793565, 1929218.787803), (931273.626088, 1929221.207678), (931272.885465, 1929220.381162), (931270.311425, 1929220.938055), (931285.571652, 1929240.179616), (931287.512710, 1929238.690955), (931286.833061, 1929237.801428), (931289.877390, 1929235.393598), (931290.379072, 1929235.592503), (931293.393199, 1929233.197194)],
            8: [(931109.745208, 1929350.398237), (931107.517502, 1929352.183155), (931106.537442, 1929352.114020), (931104.362517, 1929353.968414), (931102.217096, 1929355.490024), (931123.572270, 1929370.300755), (931122.889192, 1929366.812558), (931125.221841, 1929364.919769), (931126.143806, 1929365.194742), (931128.402187, 1929363.248166)],
            9: [(930949.468998, 1929473.449621), (930947.294577, 1929475.114827), (930945.901475, 1929474.049849), (930943.694736, 1929475.722102), (930941.696300, 1929476.343399), (930953.418371, 1929492.148083), (930954.726980, 1929490.412941), (930956.895444, 1929488.757050), (930957.668540, 1929488.887335), (930959.876393, 1929487.248217)]
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
        self.host_link_id = int(msg.LINK_ID)
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

        # --- additive (Option 1, CAN 무관): 자체 횡단보도 점유를 독립 계산·발행 ---
        # 기존 crosswalk_detection 배열/136행 참조버그와 분리 — active_objects 에서 재계산.
        occ_ids = set()
        for obj in active_objects:
            ids, _ = self.find_crosswalks_containing_object(obj.x, obj.y)
            occ_ids.update(ids)
        occ_msg = crosswalk_occupancy_msg()
        occ_msg.time = rospy.Time.now()
        occ_msg.crosswalk1_occupied = (1 in occ_ids)
        occ_msg.crosswalk2_occupied = (2 in occ_ids)
        occ_msg.occupied_ids = sorted(occ_ids)
        self.occupancy_pub.publish(occ_msg)

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
            # LINK 게이팅: ego 가 이 크로스워크의 접근 링크(CW_LINKS[N])에 있을 때만 검사.
            # host_link_id 가 어떤 매핑 링크에도 없으면(0 포함) 검사 대상 없음.
            if self.host_link_id not in CW_LINKS.get(crosswalk_id, set()):
                continue
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
