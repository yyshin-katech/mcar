#!/usr/bin/env python3
"""
LiDAR Object Detection ROS Publisher (실제 메시지 사용)
perception_ros_msg 패키지의 메시지 타입 사용
"""

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# 실제 메시지 import (패키지가 설치되어 있어야 함)
try:
    from perception_ros_msg.msg import object_array_msg, object_msg
    HAS_PERCEPTION_MSG = True
except ImportError:
    rospy.logwarn("perception_ros_msg not found. Using dummy messages.")
    HAS_PERCEPTION_MSG = False
    
    # 더미 메시지 정의
    class object_msg:
        def __init__(self):
            self.id = 0
            self.status = 0
            self.valid_level = ""
            self.x = 0.0
            self.y = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.ax = 0.0
            self.ay = 0.0
            self.size_x = 0.0
            self.size_y = 0.0
            self.orientation = 0.0
            self.orientation_v = 0.0
            self.confidence = 0.0
            self.nearest_point_x = 0.0
            self.nearest_point_y = 0.0
            self.nearest_point_z = 0.0
    
    class object_array_msg:
        def __init__(self):
            self.time = rospy.Time.now()
            self.data = []

class LidarObjectDetector:
    """
    LiDAR 객체 탐지 및 ROS 메시지 퍼블리시
    """
    
    def __init__(self):
        rospy.init_node('lidar_object_detector', anonymous=True)
        
        # 파라미터 로드
        self.voxel_size = rospy.get_param('~voxel_size', 0.1)
        self.ground_threshold = rospy.get_param('~ground_threshold', 0.2)
        self.cluster_tolerance = rospy.get_param('~cluster_tolerance', 0.5)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 10)
        self.max_cluster_size = rospy.get_param('~max_cluster_size', 10000)
        self.max_objects = rospy.get_param('~max_objects', 14)
        self.min_confidence = rospy.get_param('~min_confidence', 0.90)
        
        # CAN ID 관리 (C++ 코드와 동일)
        self.object_to_can_id = {}
        self.can_id_to_object = {}
        self.used_can_ids = set()
        self.absence_count = {}
        self.max_absence_threshold = 20
        
        # 객체 추적
        self.current_objects = {}
        self.next_tracker_id = 1
        self.frame_count = 0
        
        # Publisher
        output_topic = rospy.get_param('~output_topic', '/track_Multi_RS')
        self.pub = rospy.Publisher(output_topic, object_array_msg, queue_size=10)
        
        # Subscriber
        input_topic = rospy.get_param('~pointcloud_topic', '/rslidar_points/center')
        rospy.Subscriber(input_topic, PointCloud2, self.pointcloud_callback)
        
        rospy.loginfo("="*60)
        rospy.loginfo("LiDAR Object Detector Node Started")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Input topic:  {input_topic}")
        rospy.loginfo(f"Output topic: {output_topic}")
        rospy.loginfo(f"Max objects:  {self.max_objects}")
        rospy.loginfo(f"Min confidence: {self.min_confidence}")
        rospy.loginfo(f"Voxel size:   {self.voxel_size}m")
        rospy.loginfo("="*60)
    
    def assign_can_id(self, object_id):
        """
        CAN ID 할당 (1~254) - C++ 코드와 동일
        """
        if object_id in self.object_to_can_id:
            self.absence_count[object_id] = 0
            return self.object_to_can_id[object_id]
        
        # 빈 CAN ID 찾기
        for i in range(1, 255):
            if i not in self.used_can_ids:
                self.object_to_can_id[object_id] = i
                self.can_id_to_object[i] = object_id
                self.used_can_ids.add(i)
                self.absence_count[object_id] = 0
                return i
        
        return 0  # 가용 ID 없음
    
    def cleanup_stale_ids(self):
        """
        부재 객체 회수 - C++ 코드와 동일
        """
        expired_ids = []
        
        # 모든 absence_count 1 증가
        for obj_id in list(self.absence_count.keys()):
            self.absence_count[obj_id] += 1
            if self.absence_count[obj_id] >= self.max_absence_threshold:
                expired_ids.append(obj_id)
        
        # 만료된 객체 제거
        for obj_id in expired_ids:
            if obj_id in self.object_to_can_id:
                can_id = self.object_to_can_id[obj_id]
                self.used_can_ids.discard(can_id)
                if can_id in self.can_id_to_object:
                    del self.can_id_to_object[can_id]
                del self.object_to_can_id[obj_id]
            
            if obj_id in self.current_objects:
                del self.current_objects[obj_id]
            
            if obj_id in self.absence_count:
                del self.absence_count[obj_id]
    
    def pointcloud2_to_array(self, cloud_msg):
        """PointCloud2 → numpy array"""
        points_list = []
        for point in point_cloud2.read_points(cloud_msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)
    
    def preprocess_pointcloud(self, points):
        """전처리"""
        distances = np.linalg.norm(points[:, :2], axis=1)
        valid_mask = (distances > 1.0) & (distances < 50.0)
        points_filtered = points[valid_mask]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_filtered)
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        return np.asarray(pcd_down.points)
    
    def remove_ground(self, points):
        """지면 제거"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.ground_threshold,
            ransac_n=3,
            num_iterations=1000
        )
        
        pcd_objects = pcd.select_by_index(inliers, invert=True)
        return np.asarray(pcd_objects.points)
    
    def cluster_objects(self, points):
        """DBSCAN 클러스터링"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        labels = np.array(pcd.cluster_dbscan(
            eps=self.cluster_tolerance,
            min_points=self.min_cluster_size,
            print_progress=False
        ))
        
        clusters = []
        max_label = labels.max()
        
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            
            if self.min_cluster_size <= len(cluster_indices) <= self.max_cluster_size:
                cluster_points = points[cluster_indices]
                clusters.append(cluster_points)
        
        return clusters
    
    def classify_object(self, size):
        """객체 분류"""
        height = size[2]
        width = max(size[0], size[1])
        
        if height < 0.5:
            return 0  # low_object
        elif height < 1.0 and width < 1.0:
            return 1  # pedestrian
        elif height < 2.0 and width > 1.5:
            return 2  # vehicle
        elif height > 2.0:
            return 3  # large_object
        else:
            return 4  # unknown
    
    def compute_object_features(self, cluster, tracker_id):
        """객체 특징 계산"""
        # 바운딩 박스
        min_bound = cluster.min(axis=0)
        max_bound = cluster.max(axis=0)
        center = (min_bound + max_bound) / 2
        size = max_bound - min_bound
        
        # 가장 가까운 점
        distances = np.linalg.norm(cluster, axis=1)
        nearest_idx = np.argmin(distances)
        nearest_point = cluster[nearest_idx]
        
        # 객체 타입
        obj_type = self.classify_object(size)
        
        # 신뢰도 (포인트 수 기반)
        confidence = min(1.0, len(cluster) / 200.0)
        
        # 우선순위 (거리 기반 - 가까울수록 낮은 숫자)
        distance = np.linalg.norm(center[:2])
        priority = int(distance * 10)
        
        return {
            'tracker_id': tracker_id,
            'center': center,
            'size': size,
            'nearest_point': nearest_point,
            'obj_type': obj_type,
            'confidence': confidence,
            'priority': priority,
            'num_points': len(cluster),
            'distance': distance
        }
    
    def estimate_velocity_acceleration(self, tracker_id, current_pos, current_time):
        """속도 및 가속도 추정"""
        vx, vy = 0.0, 0.0
        ax, ay = 0.0, 0.0
        
        if tracker_id not in self.current_objects:
            return vx, vy, ax, ay
        
        prev_state = self.current_objects[tracker_id]
        
        # 속도 계산
        dt = (current_time - prev_state['time']).to_sec()
        if dt > 0.001:
            dx = current_pos[0] - prev_state['x']
            dy = current_pos[1] - prev_state['y']
            vx = dx / dt
            vy = dy / dt
            
            # 가속도 계산
            if 'vx' in prev_state and 'vy' in prev_state:
                dvx = vx - prev_state['vx']
                dvy = vy - prev_state['vy']
                ax = dvx / dt
                ay = dvy / dt
        
        return vx, vy, ax, ay
    
    def create_object_msg(self, features, can_id, vx, vy, ax, ay, total_objects):
        """
        object_msg 생성 (C++ 코드와 동일한 구조)
        """
        msg = object_msg()
        
        # C++ 코드의 ads_obj와 동일
        msg.id = can_id
        msg.status = features['obj_type']
        
        # valid_level: "tracker_id, total_tracks, priority_id"
        msg.valid_level = f"{features['tracker_id']}, {total_objects}, {features['priority']}"
        
        # 위치
        msg.x = float(features['center'][0])
        msg.y = float(features['center'][1])
        
        # 속도
        msg.vx = float(vx)
        msg.vy = float(vy)
        
        # 가속도
        msg.ax = float(ax)
        msg.ay = float(ay)
        
        # 크기
        msg.size_x = float(features['size'][0])
        msg.size_y = float(features['size'][1])
        
        # 방향
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            msg.orientation = float(np.arctan2(vy, vx))
        else:
            msg.orientation = 0.0
        msg.orientation_v = msg.orientation
        
        # 신뢰도
        msg.confidence = float(features['confidence'])
        
        # 최근접점
        msg.nearest_point_x = float(features['nearest_point'][0])
        msg.nearest_point_y = float(features['nearest_point'][1])
        msg.nearest_point_z = float(features['nearest_point'][2])
        
        return msg
    
    def pointcloud_callback(self, cloud_msg):
        """
        PointCloud2 콜백 - C++ 코드의 callback과 동일한 로직
        """
        self.frame_count += 1
        current_time = rospy.Time.now()
        
        try:
            # 1. 부재 객체 회수
            self.cleanup_stale_ids()
            
            # 2. 데이터 변환
            points = self.pointcloud2_to_array(cloud_msg)
            if len(points) == 0:
                return
            
            # 3. 전처리
            points_preprocessed = self.preprocess_pointcloud(points)
            
            # 4. 지면 제거
            points_no_ground = self.remove_ground(points_preprocessed)
            
            # 5. 클러스터링
            clusters = self.cluster_objects(points_no_ground)
            
            total_tracks = len(clusters)
            
            if total_tracks == 0:
                return
            
            # 6. 객체 처리
            current_frame_ids = set()
            detected_objects = []
            
            for i, cluster in enumerate(clusters):
                # 새 tracker_id 할당
                tracker_id = self.next_tracker_id
                self.next_tracker_id += 1
                
                # 특징 계산
                features = self.compute_object_features(cluster, tracker_id)
                
                # confidence 필터링
                if features['confidence'] <= self.min_confidence:
                    continue
                
                # 속도/가속도 추정
                vx, vy, ax, ay = self.estimate_velocity_acceleration(
                    tracker_id,
                    features['center'][:2],
                    current_time
                )
                
                # CAN ID 할당
                can_id = self.assign_can_id(tracker_id)
                
                # 현재 프레임 ID 기록
                current_frame_ids.add(tracker_id)
                
                # 상태 저장
                self.current_objects[tracker_id] = {
                    'x': features['center'][0],
                    'y': features['center'][1],
                    'vx': vx,
                    'vy': vy,
                    'time': current_time
                }
                
                detected_objects.append({
                    'features': features,
                    'can_id': can_id,
                    'vx': vx,
                    'vy': vy,
                    'ax': ax,
                    'ay': ay
                })
            
            # 7. 이번 프레임에서 관측된 객체는 부재 카운트 0
            for obj_id in current_frame_ids:
                self.absence_count[obj_id] = 0
            
            # 8. priority_id로 정렬
            detected_objects.sort(key=lambda x: x['features']['priority'])
            
            # 9. 최대 14개만 선택
            objects_to_publish = detected_objects[:self.max_objects]
            
            # 10. ROS 메시지 생성 및 퍼블리시
            array_msg = object_array_msg()
            array_msg.time = current_time
            
            for obj in objects_to_publish:
                obj_msg = self.create_object_msg(
                    obj['features'],
                    obj['can_id'],
                    obj['vx'],
                    obj['vy'],
                    obj['ax'],
                    obj['ay'],
                    total_tracks
                )
                array_msg.data.append(obj_msg)
            
            self.pub.publish(array_msg)
            
            # 11. 로그 출력 (C++ 코드와 유사)
            rospy.loginfo("---")
            rospy.loginfo(f"Frame {self.frame_count}:")
            rospy.loginfo(f"Total detected: {total_tracks}, "
                         f"Passed filter: {len(detected_objects)}, "
                         f"Published: {len(objects_to_publish)}")
            
            for i, obj in enumerate(objects_to_publish):
                feat = obj['features']
                rospy.loginfo(
                    f"  [{i}] ID={obj['can_id']}, "
                    f"Type={feat['obj_type']}, "
                    f"Priority={feat['priority']}, "
                    f"Pos=({feat['center'][0]:.2f}, {feat['center'][1]:.2f}), "
                    f"Vel=({obj['vx']:.2f}, {obj['vy']:.2f}), "
                    f"Conf={feat['confidence']:.2f}"
                )
        
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")
            import traceback
            traceback.print_exc()
    
    def run(self):
        """노드 실행"""
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = LidarObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass