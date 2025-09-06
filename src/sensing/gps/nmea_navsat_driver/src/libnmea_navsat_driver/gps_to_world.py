#!/usr/bin/env python
#-*- coding: utf-8 -*-
"""
시각화를 위해 world 대비 gps의 위치를 tf로 publish
"""


import rospy
import tf
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

import numpy as np
import pyproj

class TF_GPS_TO_WORLD(object):
    def __init__(self):
        rospy.init_node('gps_to_world')
        print (" gps_to_world node starts")
        br = tf.TransformBroadcaster()

        # measurements
        self.lat = 0.0
        self.lon = 0.0

        self.e_measured = 0.0
        self.n_measured = 0.0
        self.theta_measured = 0.0

        # for logfile visualization
        self._t = None

        # converter
        tm_central_epsg = 'epsg:5186'
        wsg84_epsg = 'epsg:4326'
        self.tm_central = pyproj.Proj(init=tm_central_epsg)
        self.wsg84 = pyproj.Proj(init=wsg84_epsg)

        sub1 = rospy.Subscriber('/sensing/gps/fix', NavSatFix, self.gps_fix_cb)
        sub2 = rospy.Subscriber('/sensing/gps/vel', TwistStamped, self.gps_vel_cb)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # br.sendTransform((self.e_measured, self.n_measured ,0),
            #                 tf.transformations.quaternion_from_euler(0, 0, self.theta_measured),
            #                 rospy.Time.now(),
            #                 "gps",
            #                 "world")
            if self._t != None:
                br.sendTransform((self.e_measured, self.n_measured ,0),
                                tf.transformations.quaternion_from_euler(0, 0, self.theta_measured),
                                self._t,
                                "base_link",
                                "world")
                # print(self.e_measured, self.n_measured)
            r.sleep()


    def gps_fix_cb(self, msg):
        '''
        GPS lat, lon 정보 받아오기 (deg)
        lat, lon을 North, East로 변환 (m)
        fix : GPS가 위성을 잡았는지 안잡았는지
        '''
        self.lat = msg.latitude
        self.lon = msg.longitude

        self.fix = msg.status.status
        self._t = msg.header.stamp
        if self.fix >= 0:
            self.e_measured, self.n_measured = pyproj.transform(self.wsg84, self.tm_central, self.lon, self.lat)


    def gps_vel_cb(self, msg):
        '''
        GPS Heading 정보 받아오기 (rad)
        x : East (m)
        y : North (m)
        theta : E에서 N으로 가는게 (+)
        '''
        vn = msg.twist.linear.y
        ve = msg.twist.linear.x
        self.theta_measured = np.arctan2(vn, ve)


if __name__ == '__main__':
    try:
        TF_GPS_TO_WORLD()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start gps_to_world() node.')
