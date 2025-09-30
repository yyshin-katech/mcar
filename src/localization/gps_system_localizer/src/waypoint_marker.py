#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# import rospy
# import numpy as np

# import glob
# import scipy.io as sio
# import matplotlib.pyplot as plt


# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point
# # MAPFILE_PATH = rospy.get_param("MAPFILE_PATH")
# MAPFILE_PATH = '/home/katech/mcar_v11/src/localization/gps_system_localizer/mapfiles/K_CITY_20250902'
# # from LaneModels.msg import LaneModels

# def waypoint_data_initialize(dir):
#     matfiles = glob.glob(dir)
#     wps = []
#     j = 0
#     for matfile in matfiles:
#         mat = sio.loadmat(matfile)
#         easts = mat["east"][0]
#         norths = mat["north"][0]
#         # print(easts[0], norths[0])
#         stations = mat["station"][0]
#         wp = {'id': j, 'e': easts, 'n': norths}
#         j = j + 1
#         wps.append(wp)
#     # print("loaded mat file num is " + str(j))
#     return wps

# class waypoint_marker(object):
#     def __init__(self):
#         rospy.init_node('waypoint_marker')

#         # initialize lanelet dataset
#         mat = MAPFILE_PATH + "/link_1.mat"
#         # mat = "/home/mmc-chanuk/Ros_Project/System-Localization/lanelet_waypoint_visualizer/src/waypoint_marker/database/*.mat"
#         self.wps = waypoint_data_initialize(mat)
#         # print(self.wps)
#         # define publisher
#         self.lanelet_strip_pub = rospy.Publisher('/rviz/waypoint_marker', MarkerArray, queue_size=100)

#         # e_ego = 326223.961293405
#         # n_ego = 340777.214972443
#         e_ego, n_ego = 0, 0
#         marker = MarkerArray()
#         for wp in self.wps:
#             marker_single = Marker()
#             marker_single.id = wp['id']
#             marker_single.header.frame_id = "/world"
#             marker_single.type = marker_single.SPHERE_LIST
#             marker_single.action = marker_single.ADD

#             # marker_single scale
#             marker_single.scale.x = 0.2
#             marker_single.scale.y = 0.2
#             marker_single.scale.z = 0.2

#             # marker_single color
#             marker_single.color.r = 210/255.0
#             marker_single.color.g = 215/255.0
#             marker_single.color.b = 211/255.0
#             marker_single.color.a = 0.8

#             # marker_single orientaiton
#             marker_single.pose.orientation.x = 0.0
#             marker_single.pose.orientation.y = 0.0
#             marker_single.pose.orientation.z = 0.0
#             marker_single.pose.orientation.w = 1.0

#             # marker line pointsW
#             marker_single.points = []
#             for i in range(0,len(wp['e']),1):
#                 point = Point()
#                 point.x = wp['e'][i] - e_ego
#                 point.y = wp['n'][i] - n_ego
#                 # print("x is " + str(point.x) + " and y is " + str(point.y))
#                 marker_single.points.append(point)
#             marker.markers.append(marker_single)


#         # Publish the Marker after 5 seconds later
#         rospy.sleep(5)
#         self.lanelet_strip_pub.publish(marker)



# if __name__ == '__main__':
#     try:
#         waypoint_marker = waypoint_marker()

#     except rospy.ROSInterruptException:
#         rospy.logerr('Could not start waypoint_marker() node.')


# import rospy
# import numpy as np
# import os
# import glob
# import scipy.io as sio
# import matplotlib.pyplot as plt

# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point

# # MAPFILE_PATH = rospy.get_param("MAPFILE_PATH")
# MAPFILE_PATH = '/home/katech/mcar_v11/src/localization/gps_system_localizer/mapfiles/K_CITY_20250902'

# def waypoint_data_initialize(filepath):
#     """Load waypoint data from a single mat file or multiple mat files"""
#     wps = []
    
#     # Check if it's a single file or a pattern
#     if filepath.endswith('.mat'):
#         # Single file
#         if not os.path.exists(filepath):
#             rospy.logerr(f"Mat file not found: {filepath}")
#             return wps
#         matfiles = [filepath]
#     else:
#         # Pattern for multiple files
#         matfiles = glob.glob(filepath)
#         if not matfiles:
#             rospy.logerr(f"No mat files found with pattern: {filepath}")
#             return wps
    
#     j = 0
#     for matfile in matfiles:
#         try:
#             rospy.loginfo(f"Loading mat file: {matfile}")
#             mat = sio.loadmat(matfile)
            
#             # Check if required keys exist
#             if 'east' not in mat or 'north' not in mat:
#                 rospy.logerr(f"Required keys 'east' or 'north' not found in {matfile}")
#                 continue
                
#             easts = mat["east"][0]
#             norths = mat["north"][0]
            
#             # Optional: load stations if available
#             stations = mat.get("station", [None])[0] if "station" in mat else None
            
#             wp = {'id': j, 'e': easts, 'n': norths}
#             if stations is not None:
#                 wp['stations'] = stations
                
#             wps.append(wp)
#             j += 1
            
#         except Exception as e:
#             rospy.logerr(f"Error loading {matfile}: {str(e)}")
#             continue
    
#     rospy.loginfo(f"Loaded {j} waypoint files successfully")
#     return wps

# class waypoint_marker(object):
#     def __init__(self):
#         rospy.init_node('waypoint_marker')
        
#         # Check if directory exists
#         if not os.path.exists(MAPFILE_PATH):
#             rospy.logerr(f"Directory not found: {MAPFILE_PATH}")
#             return
        
#         # Initialize waypoint dataset
#         mat_file = os.path.join(MAPFILE_PATH, "link_5.mat")
#         rospy.loginfo(f"Looking for mat file: {mat_file}")
        
#         self.wps = waypoint_data_initialize(mat_file)
        
#         if not self.wps:
#             rospy.logerr("No waypoint data loaded. Exiting...")
#             return
        
#         # Define publisher
#         self.lanelet_strip_pub = rospy.Publisher('/rviz/waypoint_marker', MarkerArray, queue_size=100)
        
#         # Create markers
#         self.create_and_publish_markers()
        
#         # Keep the node running
#         rospy.spin()
    
#     def create_and_publish_markers(self):
#         """Create and publish waypoint markers"""
#         # Ego vehicle position (origin)
#         e_ego, n_ego = 935586.94, 1916201.0
        
#         marker_array = MarkerArray()
        
#         for wp in self.wps:
#             marker_single = Marker()
#             marker_single.id = wp['id']
#             marker_single.header.frame_id = "gps"
#             marker_single.header.stamp = rospy.Time.now()
#             marker_single.type = marker_single.SPHERE_LIST
#             marker_single.action = marker_single.ADD
            
#             # Marker scale
#             marker_single.scale.x = 0.2
#             marker_single.scale.y = 0.2
#             marker_single.scale.z = 0.2
            
#             # Marker color
#             marker_single.color.r = 210/255.0
#             marker_single.color.g = 215/255.0
#             marker_single.color.b = 211/255.0
#             marker_single.color.a = 0.8
            
#             # Marker orientation
#             marker_single.pose.orientation.x = 0.0
#             marker_single.pose.orientation.y = 0.0
#             marker_single.pose.orientation.z = 0.0
#             marker_single.pose.orientation.w = 1.0
            
#             # Add waypoint coordinates as points
#             marker_single.points = []
#             for i in range(len(wp['e'])):
#                 point = Point()
#                 point.y = wp['e'][i] - e_ego
#                 point.x = wp['n'][i] - n_ego
#                 point.z = 0.0  # Add z coordinate
#                 marker_single.points.append(point)
            
#             marker_array.markers.append(marker_single)
        
#         # Wait for connections and publish
#         rospy.sleep(2)  # Reduced wait time
        
#         # Publish markers continuously
#         rate = rospy.Rate(1)  # 1 Hz
#         while not rospy.is_shutdown():
#             # Update timestamp
#             for marker in marker_array.markers:
#                 marker.header.stamp = rospy.Time.now()
            
#             self.lanelet_strip_pub.publish(marker_array)
#             rospy.loginfo(f"Published {len(marker_array.markers)} waypoint markers")
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         waypoint_marker_node = waypoint_marker()
#     except rospy.ROSInterruptException:
#         rospy.logerr('Could not start waypoint_marker() node.')
#     except Exception as e:
#         rospy.logerr(f'Error in waypoint_marker() node: {str(e)}')


import rospy
import numpy as np
import os
import glob
import scipy.io as sio
import matplotlib.pyplot as plt

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# MAPFILE_PATH = rospy.get_param("MAPFILE_PATH")
MAPFILE_PATH = '/home/katech/mcar_v11/src/localization/gps_system_localizer/mapfiles/K_CITY_20250902'

def waypoint_data_initialize(pattern):
    """Load waypoint data from multiple mat files matching the pattern"""
    wps = []
    
    # Find all mat files matching the pattern
    matfiles = glob.glob(pattern)
    matfiles.sort()  # Sort files for consistent ordering
    
    if not matfiles:
        rospy.logerr(f"No mat files found with pattern: {pattern}")
        return wps
    
    rospy.loginfo(f"Found {len(matfiles)} mat files to load")
    
    j = 0
    for matfile in matfiles:
        try:
            filename = os.path.basename(matfile)
            rospy.loginfo(f"Loading mat file: {filename}")
            mat = sio.loadmat(matfile)
            
            # Check if required keys exist
            if 'east' not in mat or 'north' not in mat:
                rospy.logerr(f"Required keys 'east' or 'north' not found in {filename}")
                continue
                
            easts = mat["east"][0]
            norths = mat["north"][0]
            
            # Optional: load stations if available
            stations = mat.get("station", [None])[0] if "station" in mat else None
            
            wp = {
                'id': j, 
                'e': easts, 
                'n': norths,
                'filename': filename  # Store filename for identification
            }
            if stations is not None:
                wp['stations'] = stations
                
            wps.append(wp)
            rospy.loginfo(f"  -> Loaded {len(easts)} waypoints from {filename}")
            j += 1
            
        except Exception as e:
            rospy.logerr(f"Error loading {matfile}: {str(e)}")
            continue
    
    rospy.loginfo(f"Successfully loaded {j} waypoint files with total waypoints")
    return wps

def get_color_for_link(link_id, total_links):
    """Generate different colors for each link"""
    # Create a color map for different links
    colors = [
        (1.0, 0.0, 0.0),   # Red
        (0.0, 1.0, 0.0),   # Green
        (0.0, 0.0, 1.0),   # Blue
        (1.0, 1.0, 0.0),   # Yellow
        (1.0, 0.0, 1.0),   # Magenta
        (0.0, 1.0, 1.0),   # Cyan
        (1.0, 0.5, 0.0),   # Orange
        (0.5, 0.0, 1.0),   # Purple
        (0.0, 0.5, 0.0),   # Dark Green
        (0.5, 0.5, 0.5),   # Gray
    ]
    
    if link_id < len(colors):
        return colors[link_id]
    else:
        # Generate colors procedurally for more than 10 links
        hue = (link_id * 137.5) % 360  # Golden angle for good distribution
        sat = 1.0
        val = 1.0
        
        # Convert HSV to RGB
        c = val * sat
        x = c * (1 - abs(((hue / 60) % 2) - 1))
        m = val - c
        
        if 0 <= hue < 60:
            r, g, b = c, x, 0
        elif 60 <= hue < 120:
            r, g, b = x, c, 0
        elif 120 <= hue < 180:
            r, g, b = 0, c, x
        elif 180 <= hue < 240:
            r, g, b = 0, x, c
        elif 240 <= hue < 300:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x
            
        return (r + m, g + m, b + m)

class waypoint_marker(object):
    def __init__(self):
        rospy.init_node('waypoint_marker')
        
        # Check if directory exists
        if not os.path.exists(MAPFILE_PATH):
            rospy.logerr(f"Directory not found: {MAPFILE_PATH}")
            return
        
        # Initialize waypoint dataset - Load ALL link_*.mat files
        mat_pattern = os.path.join(MAPFILE_PATH, "link_*.mat")
        rospy.loginfo(f"Looking for mat files with pattern: {mat_pattern}")
        
        self.wps = waypoint_data_initialize(mat_pattern)
        
        if not self.wps:
            rospy.logerr("No waypoint data loaded. Exiting...")
            return
        
        rospy.loginfo(f"Total loaded: {len(self.wps)} link files")
        
        # Define publisher
        self.lanelet_strip_pub = rospy.Publisher('/rviz/waypoint_marker', MarkerArray, queue_size=100)
        
        # Create markers
        self.create_and_publish_markers()
        
        # Keep the node running
        rospy.spin()
    
    def create_and_publish_markers(self):
        """Create and publish waypoint markers"""
        # Ego vehicle position (origin)
        e_ego, n_ego = 935586.94, 1916201.0
        
        marker_array = MarkerArray()
        
        for wp_idx, wp in enumerate(self.wps):
            marker_single = Marker()
            marker_single.id = wp['id']
            marker_single.header.frame_id = "gps"
            marker_single.header.stamp = rospy.Time.now()
            marker_single.type = marker_single.LINE_STRIP
            marker_single.action = marker_single.ADD
            marker_single.ns = f"waypoints_{wp['filename']}"  # Use namespace to identify different files
            
            # 선의 두께 설정 (LINE_STRIP에서는 scale.x가 선 두께)
            marker_single.scale.x = 0.1  # 선 두께
            marker_single.scale.y = 0.0  # LINE_STRIP에서는 사용되지 않음
            marker_single.scale.z = 0.0  # LINE_STRIP에서는 사용되지 않음
            
            # 모든 링크를 녹색으로 통일
            marker_single.color.r = 0.0
            marker_single.color.g = 1.0  # 녹색
            marker_single.color.b = 0.0
            marker_single.color.a = 0.8
            
            # Marker orientation
            marker_single.pose.orientation.x = 0.0
            marker_single.pose.orientation.y = 0.0
            marker_single.pose.orientation.z = 0.0
            marker_single.pose.orientation.w = 1.0
            
            # Add waypoint coordinates as points
            marker_single.points = []
            
            # Sample points to avoid too many (for performance)
            step = max(1, len(wp['e']) // 200)  # Max 200 points per link
            point_count = 0
            
            for i in range(0, len(wp['e']), step):
                point = Point()
                # 반시계방향 90도 회전: (x, y) → (-y, x)
                point.x = (wp['n'][i] - n_ego)
                point.y = -1*(wp['e'][i] - e_ego)
                point.z = 0.0
                marker_single.points.append(point)
                point_count += 1
            
            rospy.loginfo(f"Added {point_count} points from {wp['filename']} as green line")
            marker_array.markers.append(marker_single)
        
        # Wait for connections and publish
        rospy.sleep(2)
        
        # Publish markers continuously
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Update timestamp
            for marker in marker_array.markers:
                marker.header.stamp = rospy.Time.now()
            
            self.lanelet_strip_pub.publish(marker_array)
            rospy.loginfo_throttle(10, f"Published {len(marker_array.markers)} link markers (from {len(self.wps)} files)")
            rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_marker_node = waypoint_marker()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint_marker() node.')
    except Exception as e:
        rospy.logerr(f'Error in waypoint_marker() node: {str(e)}')