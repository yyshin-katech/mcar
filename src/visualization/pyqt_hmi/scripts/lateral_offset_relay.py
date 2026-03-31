#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from mmc_msgs.msg import to_control_team_from_local_msg

pub = None

def callback(msg):
    pub.publish(Float32(data=msg.lateral_offset))

if __name__ == '__main__':
    rospy.init_node('lateral_offset_relay')
    pub = rospy.Publisher('/rviz/lateral_offset', Float32, queue_size=1)
    rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, callback)
    rospy.spin()
