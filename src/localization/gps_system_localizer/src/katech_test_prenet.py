#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import shapefile
import math
from scipy.spatial import cKDTree
from geometry_msgs.msg import PoseStamped
from mmc_msgs.msg import to_control_team_from_local_msg

class FrenetCoordinateCalculator:
    def __init__(self):
        rospy.init_node('frenet_coordinate_calculator')
        
        # Parameters
        self.mapfile_path = rospy.get_param('~mapfile_path', 
                                          '/home/katech/mcar_v11/src/localization/gps_system_localizer/src')
        self.shp_filename = rospy.get_param('~shp_filename', 'A2_LINK_epsg5179.shp')
        
        # Reference coordinates
        self.ref_east = rospy.get_param('~ref_east', 935586.94)
        self.ref_north = rospy.get_param('~ref_north', 1916201.0)
        
        # Load road centerlines
        self.road_segments = []
        self.kdtree = None
        self.load_road_centerlines()
        
        # Publisher for Frenet coordinates only
        self.frenet_pub = rospy.Publisher('/frenet_coordinates', PoseStamped, queue_size=10)
        
        # Subscriber for vehicle position
        # self.vehicle_pos_sub = rospy.Subscriber('/vehicle_position', PoseStamped, self.vehicle_position_callback)
        self.vehicle_pos_sub = rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, self.vehicle_position_callback)

        rospy.loginfo("Frenet Coordinate Calculator initialized")
        rospy.loginfo(f"Loaded {len(self.road_segments)} road segments")
    
    def load_road_centerlines(self):
        """Load road centerline data from shapefile"""
        shp_path = f"{self.mapfile_path}/{self.shp_filename}"
        
        try:
            sf = shapefile.Reader(shp_path)
            shapes = sf.shapes()
            
            rospy.loginfo(f"Loading road centerlines from {shp_path}")
            rospy.loginfo(f"Found {len(shapes)} road segments")
            rospy.loginfo(f"Reference point: East={self.ref_east:.2f}, North={self.ref_north:.2f}")
            
            all_points = []
            
            for idx, shape in enumerate(shapes):
                if len(shape.points) < 2:
                    continue
                
                # Convert coordinates to local frame
                # 표준 좌표계: East → X, North → Y
                local_points = []
                for point in shape.points:
                    east, north = point
                    x = east - self.ref_east    # East → X
                    y = north - self.ref_north  # North → Y
                    local_points.append([x, y])
                
                # Log first segment for debugging
                if idx == 0:
                    rospy.loginfo(f"First segment sample: UTM({shape.points[0][0]:.2f}, {shape.points[0][1]:.2f}) -> Local({local_points[0][0]:.2f}, {local_points[0][1]:.2f})")
                
                # Calculate cumulative distances along the path
                cumulative_distances = [0.0]
                total_length = 0.0
                
                for i in range(1, len(local_points)):
                    dx = local_points[i][0] - local_points[i-1][0]
                    dy = local_points[i][1] - local_points[i-1][1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    total_length += dist
                    cumulative_distances.append(total_length)
                
                segment = {
                    'id': idx,
                    'points': np.array(local_points),
                    'cumulative_distances': cumulative_distances,
                    'total_length': total_length
                }
                
                self.road_segments.append(segment)
                
                # Add all points for KDTree
                for point in local_points:
                    all_points.append(point)
            
            # Build KDTree for fast nearest neighbor search
            if all_points:
                self.kdtree = cKDTree(all_points)
                rospy.loginfo(f"Built KDTree with {len(all_points)} points")
            
            # 도로 범위 출력
            if all_points:
                all_points_array = np.array(all_points)
                x_min, x_max = all_points_array[:, 0].min(), all_points_array[:, 0].max()
                y_min, y_max = all_points_array[:, 1].min(), all_points_array[:, 1].max()
                rospy.loginfo(f"Road network bounds: X[{x_min:.2f}, {x_max:.2f}], Y[{y_min:.2f}, {y_max:.2f}]")
            
        except Exception as e:
            rospy.logerr(f"Error loading road centerlines: {e}")
    
    def find_closest_road_segment(self, vehicle_x, vehicle_y):
        """Find the closest road segment and point to vehicle position"""
        if not self.road_segments:
            return None, None, None, None
        
        min_distance = float('inf')
        closest_segment = None
        closest_point_idx = None
        closest_point = None
        projection_ratio = 0.0
        
        for segment in self.road_segments:
            points = segment['points']
            
            # Check each line segment in the road
            for i in range(len(points) - 1):
                p1 = points[i]
                p2 = points[i + 1]
                
                # Find closest point on line segment
                closest_on_segment, ratio = self.point_to_line_distance(
                    [vehicle_x, vehicle_y], p1, p2)
                
                distance = math.sqrt((vehicle_x - closest_on_segment[0])**2 + 
                                   (vehicle_y - closest_on_segment[1])**2)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_segment = segment
                    closest_point_idx = i
                    closest_point = closest_on_segment
                    projection_ratio = ratio
        
        return closest_segment, closest_point_idx, closest_point, projection_ratio
    
    def point_to_line_distance(self, point, line_start, line_end):
        """Calculate closest point on line segment to given point"""
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # Vector from line_start to line_end
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            # Line segment is actually a point
            return line_start, 0.0
        
        # Parameter t represents position along line segment (0 to 1)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        
        # Clamp t to [0, 1] to stay within line segment
        t = max(0.0, min(1.0, t))
        
        # Calculate closest point
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return [closest_x, closest_y], t
    
    def calculate_frenet_coordinates(self, vehicle_x, vehicle_y):
        """Calculate Frenet coordinates (s, d) for vehicle position"""
        segment, point_idx, closest_point, ratio = self.find_closest_road_segment(vehicle_x, vehicle_y)
        
        if segment is None:
            return None, None, None
        
        # Calculate s coordinate (longitudinal distance)
        if point_idx is not None:
            s_base = segment['cumulative_distances'][point_idx]
            
            if point_idx + 1 < len(segment['cumulative_distances']):
                segment_length = (segment['cumulative_distances'][point_idx + 1] - 
                                segment['cumulative_distances'][point_idx])
                s = s_base + ratio * segment_length
            else:
                s = s_base
        else:
            s = 0.0
        
        # Calculate d coordinate (lateral distance)
        # Positive d means left side of road, negative means right side
        if closest_point is not None and point_idx is not None:
            # Get road direction at closest point
            if point_idx + 1 < len(segment['points']):
                road_point1 = segment['points'][point_idx]
                road_point2 = segment['points'][point_idx + 1]
                
                # Road direction vector
                road_dx = road_point2[0] - road_point1[0]
                road_dy = road_point2[1] - road_point1[1]
                road_length = math.sqrt(road_dx**2 + road_dy**2)
                
                if road_length > 0:
                    # Normalize road direction
                    road_dx /= road_length
                    road_dy /= road_length
                    
                    # Vector from closest point to vehicle
                    to_vehicle_dx = vehicle_x - closest_point[0]
                    to_vehicle_dy = vehicle_y - closest_point[1]
                    
                    # Cross product to determine side (left/right)
                    cross_product = road_dx * to_vehicle_dy - road_dy * to_vehicle_dx
                    
                    # Distance
                    d_distance = math.sqrt(to_vehicle_dx**2 + to_vehicle_dy**2)
                    
                    # Apply sign based on which side of road
                    d = d_distance if cross_product > 0 else -d_distance
                else:
                    d = 0.0
            else:
                d = 0.0
        else:
            d = 0.0
        
        return s, d, segment
    
    def vehicle_position_callback(self, msg):
        """Callback for vehicle position updates"""
        # 차량 위치가 UTM 좌표로 들어오는지 로컬 좌표로 들어오는지 확인
        raw_x = msg.host_east
        raw_y = msg.host_north
        
        # UTM 좌표인지 확인 (값이 매우 크면 UTM)
        if abs(raw_x) > 100000 or abs(raw_y) > 100000:
            # UTM 좌표를 로컬 좌표로 변환
            # 표준: msg.pose.position.x = East, msg.pose.position.y = North
            vehicle_east = raw_x
            vehicle_north = raw_y
            vehicle_x = vehicle_east - self.ref_east    # East → X
            vehicle_y = vehicle_north - self.ref_north  # North → Y
            rospy.logdebug(f"Converted UTM(E:{vehicle_east:.2f}, N:{vehicle_north:.2f}) to Local({vehicle_x:.2f}, {vehicle_y:.2f})")
        else:
            # 이미 로컬 좌표
            vehicle_x = raw_x
            vehicle_y = raw_y
            rospy.logdebug(f"Using local coordinates: ({vehicle_x:.2f}, {vehicle_y:.2f})")
        
        # Calculate Frenet coordinates
        s, d, segment = self.calculate_frenet_coordinates(vehicle_x, vehicle_y)
        
        if s is not None and d is not None:
            # Publish Frenet coordinates
            frenet_msg = PoseStamped()
            # frenet_msg.header = msg.header
            frenet_msg.header.frame_id = "frenet"
            frenet_msg.pose.position.x = s  # Longitudinal distance
            frenet_msg.pose.position.y = d  # Lateral distance
            frenet_msg.pose.position.z = 0.0
            
            # Add segment ID to orientation.w for reference
            if segment:
                frenet_msg.pose.orientation.w = segment['id']
            
            self.frenet_pub.publish(frenet_msg)
            
            rospy.loginfo_throttle(1, f"Vehicle UTM(E:{raw_x:.2f}, N:{raw_y:.2f}) Local({vehicle_x:.2f}, {vehicle_y:.2f}) -> "
                          f"Frenet (s={s:.2f}m, d={d:.2f}m) on segment {segment['id'] if segment else 'None'}")
        else:
            rospy.logwarn_throttle(5, f"Could not calculate Frenet coordinates for vehicle at ({vehicle_x:.2f}, {vehicle_y:.2f})")

def main():
    try:
        calculator = FrenetCoordinateCalculator()
        
        rospy.loginfo("Frenet coordinate calculator ready. Waiting for vehicle position...")
        rospy.loginfo("Publish vehicle position to /vehicle_position topic")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Frenet coordinate calculator shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Frenet coordinate calculator: {e}")

if __name__ == '__main__':
    main()