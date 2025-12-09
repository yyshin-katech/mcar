#!/usr/bin/env python3
"""
LiDAR Object Detection with TF Transform
TF를 사용하여 좌표계 변환 지원
"""

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf2_ros
import tf2_sensor_msgs
from tf2_geometry_msgs import PointStamped

try:
    from perception_ros_msg.msg import object_array_msg, object_msg
    HAS_PERCEPTION_MSG = True
except ImportError:
    rospy.logwarn("perception_ros_msg not found.")
    HAS_PERCEPTION_MSG = False
    
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

class LidarObjectDetectorTF:
    """
    TF 변환을 지원하는 LiDAR 객체 탐지
    """
    
    def __init__(self):
        rospy.init_node('lidar_object_detector_tf', anonymous=True)
        
        # TF 관련
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame = rospy.get_param('~target_frame', 'ego_frame')
        self.use_tf = rospy.get_param('~use_tf', True)
        
        # 파라미터 로드
        self.voxel_size = rospy.get_param('~voxel_size', 0.1)
        self.ground_threshold = rospy.get_param('~ground_threshold', 0.2)
        self.cluster_tolerance = rospy.get_param('~cluster_tolerance', 0.5)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 10)
        self.max_cluster_size = rospy.get_param('~max_cluster_size', 10000)
        self.max_objects = 3
        self.min_confidence = rospy.get_param('~min_confidence', 0.70)
        
        # CAN ID 관리
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
        rospy.loginfo("LiDAR Object Detector with TF Transform")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Input topic:  {input_topic}")
        rospy.loginfo(f"Output topic: {output_topic}")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Use TF:       {self.use_tf}")
        rospy.loginfo(f"Max objects:  {self.max_objects} (CLOSEST)")
        rospy.loginfo("="*60)
    
    def transform_pointcloud(self, cloud_msg):
        """
        PointCloud2를 target_frame으로 변환
        """
        if not self.use_tf:
            return cloud_msg
        
        try:
            # TF 변환 대기
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud_msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # PointCloud2 변환
            cloud_transformed = tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)
            
            rospy.loginfo_once(f"Transformed: {cloud_msg.header.frame_id} → {self.target_frame}")
            return cloud_transformed
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF transform failed: {e}")
            return cloud_msg
    
    def assign_can_id(self, object_id):
        """CAN ID 할당"""
        if object_id in self.object_to_can_id:
            self.absence_count[object_id] = 0
            return self.object_to_can_id[object_id]
        
        for i in range(1, 255):
            if i not in self.used_can_ids:
                self.object_to_can_id[object_id] = i
                self.can_id_to_object[i] = object_id
                self.used_can_ids.add(i)
                self.absence_count[object_id] = 0
                return i
        return 0
    
    def cleanup_stale_ids(self):
        """부재 객체 회수"""
        expired_ids = []
        for obj_id in list(self.absence_count.keys()):
            self.absence_count[obj_id] += 1
            if self.absence_count[obj_id] >= self.max_absence_threshold:
                expired_ids.append(obj_id)
        
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
                clusters.append(points[cluster_indices])
        return clusters
    
    def get_cluster_distance(self, cluster):
        """클러스터 중심 거리"""
        center = cluster.mean(axis=0)
        return np.linalg.norm(center[:2])
    
    def classify_object(self, size):
        """객체 분류"""
        height = size[2]
        width = max(size[0], size[1])
        if height < 0.5:
            return 0
        elif height < 1.0 and width < 1.0:
            return 1
        elif height < 2.0 and width > 1.5:
            return 2
        elif height > 2.0:
            return 3
        return 4
    
    def compute_object_features(self, cluster, tracker_id, distance):
        """객체 특징 계산"""
        min_bound = cluster.min(axis=0)
        max_bound = cluster.max(axis=0)
        center = (min_bound + max_bound) / 2
        size = max_bound - min_bound
        
        distances_to_origin = np.linalg.norm(cluster, axis=1)
        nearest_idx = np.argmin(distances_to_origin)
        nearest_point = cluster[nearest_idx]
        
        obj_type = self.classify_object(size)
        confidence = min(1.0, len(cluster) / 200.0)
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
        """속도/가속도 추정"""
        vx, vy, ax, ay = 0.0, 0.0, 0.0, 0.0
        if tracker_id not in self.current_objects:
            return vx, vy, ax, ay
        
        prev_state = self.current_objects[tracker_id]
        dt = (current_time - prev_state['time']).to_sec()
        if dt > 0.001:
            dx = current_pos[0] - prev_state['x']
            dy = current_pos[1] - prev_state['y']
            vx = dx / dt
            vy = dy / dt
            
            if 'vx' in prev_state and 'vy' in prev_state:
                dvx = vx - prev_state['vx']
                dvy = vy - prev_state['vy']
                ax = dvx / dt
                ay = dvy / dt
        return vx, vy, ax, ay
    
    def create_object_msg(self, features, can_id, vx, vy, ax, ay, total_objects):
        """object_msg 생성"""
        msg = object_msg()
        msg.id = can_id
        msg.status = features['obj_type']
        msg.valid_level = f"{features['tracker_id']}, {total_objects}, {features['priority']}"
        msg.x = float(features['center'][0])
        msg.y = float(features['center'][1])
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.ax = float(ax)
        msg.ay = float(ay)
        msg.size_x = float(features['size'][0])
        msg.size_y = float(features['size'][1])
        
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            msg.orientation = float(np.arctan2(vy, vx))
        else:
            msg.orientation = 0.0
        msg.orientation_v = msg.orientation
        msg.confidence = float(features['confidence'])
        msg.nearest_point_x = float(features['nearest_point'][0])
        msg.nearest_point_y = float(features['nearest_point'][1])
        msg.nearest_point_z = float(features['nearest_point'][2])
        return msg
    
    def pointcloud_callback(self, cloud_msg):
        """PointCloud2 콜백"""
        self.frame_count += 1
        current_time = rospy.Time.now()
        
        try:
            # 1. TF 변환 (필요시)
            cloud_transformed = self.transform_pointcloud(cloud_msg)
            
            # 2. 부재 객체 회수
            self.cleanup_stale_ids()
            
            # 3. 데이터 변환
            points = self.pointcloud2_to_array(cloud_transformed)
            if len(points) == 0:
                return
            
            # 4. 전처리
            points_preprocessed = self.preprocess_pointcloud(points)
            points_no_ground = self.remove_ground(points_preprocessed)
            
            # 5. 클러스터링
            clusters = self.cluster_objects(points_no_ground)
            total_tracks = len(clusters)
            if total_tracks == 0:
                return
            
            # 6. 거리 정렬
            cluster_distances = []
            for cluster in clusters:
                distance = self.get_cluster_distance(cluster)
                cluster_distances.append((distance, cluster))
            cluster_distances.sort(key=lambda x: x[0])
            closest_clusters = cluster_distances[:self.max_objects]
            
            # 7. 객체 처리
            current_frame_ids = set()
            detected_objects = []
            
            for distance, cluster in closest_clusters:
                tracker_id = self.next_tracker_id
                self.next_tracker_id += 1
                
                features = self.compute_object_features(cluster, tracker_id, distance)
                if features['confidence'] < self.min_confidence:
                    continue
                
                vx, vy, ax, ay = self.estimate_velocity_acceleration(
                    tracker_id, features['center'][:2], current_time
                )
                
                can_id = self.assign_can_id(tracker_id)
                current_frame_ids.add(tracker_id)
                
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
            
            # 8. 부재 카운트 리셋
            for obj_id in current_frame_ids:
                self.absence_count[obj_id] = 0
            
            # 9. 퍼블리시
            array_msg = object_array_msg()
            array_msg.time = current_time
            
            for obj in detected_objects:
                obj_msg = self.create_object_msg(
                    obj['features'], obj['can_id'],
                    obj['vx'], obj['vy'], obj['ax'], obj['ay'],
                    total_tracks
                )
                array_msg.data.append(obj_msg)
            
            self.pub.publish(array_msg)
            
            # 10. 로그
            rospy.loginfo(f"Frame {self.frame_count}: {len(detected_objects)} objects")
            for i, obj in enumerate(detected_objects):
                feat = obj['features']
                rospy.loginfo(
                    f"  [{i+1}] ID={obj['can_id']}, Dist={feat['distance']:.2f}m, "
                    f"Pos=({feat['center'][0]:.2f}, {feat['center'][1]:.2f})"
                )
        
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = LidarObjectDetectorTF()
        detector.run()
    except rospy.ROSInterruptException:
        pass