#!/usr/bin/env python3
"""
Object Array Message Listener
/track_Multi_RS 토픽을 구독하여 객체 정보를 표시
"""

import rospy
import sys

try:
    from perception_ros_msg.msg import object_array_msg
    HAS_MSG = True
except ImportError:
    rospy.logwarn("perception_ros_msg not found. Define message structure manually.")
    HAS_MSG = False

def callback(msg):
    """
    object_array_msg 콜백
    """
    print("\n" + "="*70)
    print(f"Time: {msg.time.secs}.{msg.time.nsecs:09d}")
    print(f"Objects: {len(msg.data)}")
    print("="*70)
    
    for i, obj in enumerate(msg.data):
        # valid_level 파싱
        try:
            parts = obj.valid_level.split(', ')
            if len(parts) >= 3:
                tracker_id = parts[0]
                total_tracks = parts[1]
                priority = parts[2]
            else:
                tracker_id, total_tracks, priority = "?", "?", "?"
        except:
            tracker_id, total_tracks, priority = "?", "?", "?"
        
        # 거리 계산
        distance = (obj.x**2 + obj.y**2)**0.5
        
        # 속도 크기
        speed = (obj.vx**2 + obj.vy**2)**0.5
        
        # 객체 타입 이름
        type_names = {
            0: "low_object",
            1: "pedestrian",
            2: "vehicle",
            3: "large_object",
            4: "unknown"
        }
        type_name = type_names.get(obj.status, "unknown")
        
        print(f"\n[{i}] Object #{obj.id} (Tracker: {tracker_id}, Priority: {priority})")
        print(f"  Type: {type_name} ({obj.status})")
        print(f"  Position: ({obj.x:.2f}, {obj.y:.2f}) m")
        print(f"  Distance: {distance:.2f} m")
        print(f"  Velocity: ({obj.vx:.2f}, {obj.vy:.2f}) m/s, Speed: {speed:.2f} m/s")
        print(f"  Acceleration: ({obj.ax:.2f}, {obj.ay:.2f}) m/s²")
        print(f"  Size: {obj.size_x:.2f} x {obj.size_y:.2f} m")
        print(f"  Orientation: {obj.orientation:.3f} rad ({obj.orientation*57.3:.1f}°)")
        print(f"  Confidence: {obj.confidence:.2f}")
        print(f"  Nearest point: ({obj.nearest_point_x:.2f}, {obj.nearest_point_y:.2f}, {obj.nearest_point_z:.2f})")

def listener():
    """
    ROS 노드 실행
    """
    rospy.init_node('object_listener', anonymous=True)
    
    topic = rospy.get_param('~topic', '/track_Multi_RS')
    
    rospy.loginfo("="*70)
    rospy.loginfo("Object Array Message Listener")
    rospy.loginfo("="*70)
    rospy.loginfo(f"Topic: {topic}")
    rospy.loginfo("Waiting for messages...")
    rospy.loginfo("="*70)
    
    if HAS_MSG:
        rospy.Subscriber(topic, object_array_msg, callback)
    else:
        rospy.logerr("Cannot subscribe - message type not available")
        sys.exit(1)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass