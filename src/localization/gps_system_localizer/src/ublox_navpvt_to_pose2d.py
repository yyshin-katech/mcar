#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# /ublox/navpvt (ublox_msgs/NavPVT) -> /localization/pose_2d_gps (localization2D_msg)
# 좌표계: EPSG:4326 (lat/lon) -> EPSG:5179 (Korea 2000 / Unified CS)
# gps_world_tf_node_send_host_gps.cpp 와 동일 좌표계 사용

import rospy
import math
import pyproj

from ublox_msgs.msg import NavPVT
from mmc_msgs.msg import localization2D_msg

EPSG_SRC = 'EPSG:4326'
EPSG_DST = 'EPSG:5179'  # Korea 2000 / Unified CS

transformer = pyproj.Transformer.from_crs(EPSG_SRC, EPSG_DST, always_xy=True)


def navpvt_cb(msg, pub):
    lat = msg.lat * 1e-7   # deg
    lon = msg.lon * 1e-7   # deg
    heading = msg.heading * 1e-5  # deg, 0=North 시계방향

    east, north = transformer.transform(lon, lat)

    # yaw: ROS 기준 (0=East, CCW positive)
    yaw = math.pi / 2.0 - math.radians(heading)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # -pi ~ pi 정규화

    out = localization2D_msg()
    out.time = rospy.Time.now()
    out.EPSG = 5179
    out.east = east
    out.north = north
    out.yaw = yaw

    pub.publish(out)


def main():
    rospy.init_node('ublox_navpvt_to_pose2d')
    pub = rospy.Publisher('/localization/pose_2d_gps', localization2D_msg, queue_size=1)
    rospy.Subscriber('/ublox/navpvt', NavPVT, navpvt_cb, callback_args=pub, queue_size=1)
    rospy.loginfo("ublox_navpvt_to_pose2d: ready (EPSG:5179)")
    rospy.spin()


if __name__ == '__main__':
    main()
