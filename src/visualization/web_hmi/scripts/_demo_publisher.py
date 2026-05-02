#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_demo_publisher — emits fake /hmi/* JSON for browser-only debugging.

Run with: ``rosrun web_hmi _demo_publisher.py``. Useful when no bag or
real sensors are available — the React UI gets a moving speed gauge,
P/R/N/D cycle, orbiting objects, and traffic-light cycle.
"""
import json
import math
import time

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('web_hmi_demo_publisher', anonymous=True)

    pub_state = rospy.Publisher('/hmi/state', String, queue_size=2)
    pub_diag = rospy.Publisher('/hmi/diagnostics', String, queue_size=2)
    pub_objs = rospy.Publisher('/hmi/objects', String, queue_size=2)
    pub_popup = rospy.Publisher('/hmi/popup', String, queue_size=4, latch=True)
    pub_traffic = rospy.Publisher('/hmi/traffic', String, queue_size=4, latch=True)
    pub_bag = rospy.Publisher('/hmi/bag', String, queue_size=4, latch=True)
    pub_hz = rospy.Publisher('/hmi/topic_hz', String, queue_size=2)

    gear_cycle = [1, 2, 3, 4]

    pub_bag.publish(String(data=json.dumps({
        'recording': False, 'info': 'demo'})))

    rate = rospy.Rate(10)
    t0 = time.monotonic()
    last_traffic = -1

    while not rospy.is_shutdown():
        t = time.monotonic() - t0
        speed = 40 + 30 * math.sin(t / 4.0)
        gear = gear_cycle[int(t / 5) % 4]
        steering = 12 * math.sin(t / 2.0)
        autonomous = 1 if int(t) % 10 < 7 else 0

        pub_state.publish(String(data=json.dumps({
            'speed': round(speed, 2),
            'gear': gear,
            'mode': autonomous,
            'aeb': False,
            'steering': round(steering, 2),
            'ego': {'east': 200000.0, 'north': 500000.0,
                    'yaw': 0.5 * math.sin(t / 6.0)},
            'gps': {'rtk': 2, 'lon_std': 0.012, 'lat_std': 0.011},
            'speed_limit': 50,
            'link_id': 1234567,
            'lane_label': '1',
            'on_odd': 0,
            'road_state': 0,
            'selected_mode': 0,
        }, ensure_ascii=False)))

        pub_diag.publish(String(data=json.dumps({
            'status': {'gps': 0, 'adcu': 0, 'lidar': 0, 'radar': 0,
                       'v2x': 0, 'hmi': 0, 'vcu': 0, 'cam': 1, 'ipc': 0},
        })))

        # 3 fake orbiting objects (m, ego frame)
        objs = []
        for i, phase in enumerate((0, 2 * math.pi / 3, 4 * math.pi / 3)):
            r = 22 + 4 * math.sin(t + i)
            ang = t / 3.0 + phase
            objs.append({
                'id': i,
                'x': r * math.cos(ang),
                'y': r * math.sin(ang),
                'width': 1.8,
                'length': 4.4,
                'vx': -r * math.sin(ang) / 3.0,
                'vy':  r * math.cos(ang) / 3.0,
                'orientation': ang + math.pi / 2,
                'type': 'pedestrian' if i == 0 else 'car',
            })
        pub_objs.publish(String(data=json.dumps({'count': len(objs), 'data': objs})))

        # Traffic light cycle 5/2/3 s green/amber/red
        cyc = int(t) % 10
        color = 1 if cyc < 5 else (2 if cyc < 7 else 3)
        if color != last_traffic:
            last_traffic = color
            pub_traffic.publish(String(data=json.dumps({
                'color': color,
                'time_decisec': (10 - cyc) * 10,
                'look_at': {'intersection_id': 1, 'signal_group_id': 1},
            })))

        # Hz table — fake plausible values
        pub_hz.publish(String(data=json.dumps({
            'gps': 10.0, 'adcu': 50.0, 'lidar': 10.0, 'radar': 20.0,
            'v2x': 1.0, 'hmi': 5.0, 'vcu': 50.0, 'cam': 30.0, 'ipc': 5.0,
        })))

        # Popup demo — single CAM warning
        pub_popup.publish(String(data=json.dumps({
            'text': 'CAM 센서 고장', 'severity': 'warn',
        }, ensure_ascii=False)))

        rate.sleep()


if __name__ == '__main__':
    main()
