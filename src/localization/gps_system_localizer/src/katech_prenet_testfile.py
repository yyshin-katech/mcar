#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class VehiclePositionPublisher:
    def __init__(self):
        rospy.init_node('vehicle_position_publisher')
        
        # Publisher for vehicle position
        self.position_pub = rospy.Publisher('/vehicle_position', PoseStamped, queue_size=10)
        
        # Parameters for movement simulation
        self.start_x = rospy.get_param('~start_x', 0.0)
        self.start_y = rospy.get_param('~start_y', 0.0)
        self.speed = rospy.get_param('~speed', 2.0)  # m/s
        self.radius = rospy.get_param('~radius', 50.0)  # meters
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        
        # Movement pattern
        self.pattern = rospy.get_param('~pattern', 'circle')  # 'circle', 'straight', 'figure8'
        
        self.start_time = rospy.Time.now()
        
        rospy.loginfo(f"Vehicle position publisher initialized")
        rospy.loginfo(f"Pattern: {self.pattern}, Speed: {self.speed} m/s")
        rospy.loginfo(f"Starting position: ({self.start_x}, {self.start_y})")
    
    def calculate_position(self):
        """Calculate vehicle position based on movement pattern"""
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()
        
        if self.pattern == 'circle':
            # Circular motion
            angular_velocity = self.speed / self.radius
            angle = angular_velocity * elapsed_time
            
            x = self.start_x + self.radius * math.cos(angle)
            y = self.start_y + self.radius * math.sin(angle)
            
        elif self.pattern == 'straight':
            # Straight line motion
            x = self.start_x + self.speed * elapsed_time
            y = self.start_y
            
        elif self.pattern == 'figure8':
            # Figure-8 pattern
            t = elapsed_time * 0.5  # Slow down for better visualization
            x = self.start_x + self.radius * math.sin(t)
            y = self.start_y + self.radius * math.sin(t) * math.cos(t)
            
        elif self.pattern == 'sine':
            # Sine wave pattern
            x = self.start_x + self.speed * elapsed_time
            y = self.start_y + 10.0 * math.sin(0.1 * x)  # 10m amplitude, wavelength based on x
            
        elif self.pattern == 'manual':
            # Static position (for manual testing)
            x = self.start_x
            y = self.start_y
            
        else:
            rospy.logwarn(f"Unknown pattern: {self.pattern}, using straight line")
            x = self.start_x + self.speed * elapsed_time
            y = self.start_y
        
        return x, y
    
    def publish_position(self):
        """Publish current vehicle position"""
        x, y = self.calculate_position()
        
        msg = PoseStamped()
        msg.header.frame_id = "gps"
        msg.header.stamp = rospy.Time.now()
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Simple orientation (could be improved with actual heading calculation)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.position_pub.publish(msg)
        
        rospy.logdebug(f"Published position: ({x:.2f}, {y:.2f})")
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Starting vehicle position publishing...")
        rospy.loginfo("Vehicle will move in '{}' pattern".format(self.pattern))
        
        while not rospy.is_shutdown():
            self.publish_position()
            rate.sleep()

def main():
    try:
        publisher = VehiclePositionPublisher()
        publisher.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Vehicle position publisher shutting down")
    except Exception as e:
        rospy.logerr(f"Error in vehicle position publisher: {e}")

if __name__ == '__main__':
    main()