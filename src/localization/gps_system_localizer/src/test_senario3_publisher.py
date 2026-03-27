#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# senario3 mapfiles(mat) 기반 1회 주행 테스트 노드
# - 경로: 50링크, 2322m (동→서, start=link_273)
# - 갭 없음, LC 플래그 없음
# - 속도: 30 km/h, 주기: 20 Hz

import rospy
import numpy as np
import os
import scipy.io as sio

from mmc_msgs.msg import localization2D_msg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAPFILE_PATH = os.path.join(SCRIPT_DIR, '..', 'mapfiles', 'senario3')

ROUTE_LINK_IDS = [
    273, 427, 123, 281, 286, 131, 313, 299, 324, 330,
    139, 340, 336, 452, 351, 355, 447, 366, 376, 385,
    51, 389, 396, 401, 408, 146, 639, 788, 784, 790,
    795, 1283, 812, 1289, 628, 1290, 821, 828, 835, 816,
    862, 1301, 1142, 1304, 1146, 1147, 1154, 1160, 1172, 1195,
]

SPEED_KMH = 30.0
RATE_HZ   = 20.0
GAP_INTERP_MIN = 0.5   # 이 거리 이상 갭이면 선형 보간 적용


def load_route_waypoints():
    """링크 순서대로 웨이포인트를 이어붙임.
    갭은 선형 보간으로 부드럽게 연결.
    have_to_LangeChange_left/right=1 인 링크 목록도 반환."""

    east_all, north_all = [], []
    lc_links = []

    for lid in ROUTE_LINK_IDS:
        mat_path = os.path.join(MAPFILE_PATH, f'link_{lid}.mat')
        mat = sio.loadmat(mat_path)
        e = mat['east'][0]
        n = mat['north'][0]
        hl = int(mat['have_to_LangeChange_left'][0][0])
        hr = int(mat['have_to_LangeChange_right'][0][0])

        if not east_all:
            east_all.extend(e.tolist())
            north_all.extend(n.tolist())
        else:
            prev_e = east_all[-1]
            prev_n = north_all[-1]
            gap = np.sqrt((e[0] - prev_e) ** 2 + (n[0] - prev_n) ** 2)

            if gap > GAP_INTERP_MIN:
                n_pts = max(2, int(np.ceil(gap)))
                rospy.logwarn(f"link_{lid}: gap {gap:.1f}m → linear interp {n_pts} pts")
                for t in np.linspace(0, 1, n_pts + 1)[1:]:
                    east_all.append(prev_e + t * (e[0] - prev_e))
                    north_all.append(prev_n + t * (n[0] - prev_n))
                east_all.extend(e[1:].tolist())
                north_all.extend(n[1:].tolist())
            else:
                east_all.extend(e[1:].tolist())
                north_all.extend(n[1:].tolist())

        if hl or hr:
            lc_links.append((lid, hl, hr))

    return np.array(east_all), np.array(north_all), lc_links


def build_station(east, north):
    """누적 거리(station) 배열 계산"""
    ds = np.sqrt(np.diff(east) ** 2 + np.diff(north) ** 2)
    station = np.zeros(len(east))
    station[1:] = np.cumsum(ds)
    return station


def interpolate_position(east, north, station, s):
    """station s에 해당하는 (e, n, yaw) 선형 보간"""
    idx = np.searchsorted(station, s, side='right') - 1
    idx = np.clip(idx, 0, len(station) - 2)
    seg_len = station[idx + 1] - station[idx]
    ratio = 0.0 if seg_len < 1e-9 else (s - station[idx]) / seg_len

    e   = east[idx]  + ratio * (east[idx + 1]  - east[idx])
    n   = north[idx] + ratio * (north[idx + 1] - north[idx])
    dx  = east[idx + 1]  - east[idx]
    dy  = north[idx + 1] - north[idx]
    yaw = np.arctan2(dy, dx)
    return e, n, yaw


def main():
    rospy.init_node('test_senario3_publisher')
    pub = rospy.Publisher('/localization/pose_2d_gps', localization2D_msg, queue_size=1)

    rospy.loginfo("test_senario3_publisher: loading senario3 waypoints...")
    east, north, lc_links = load_route_waypoints()
    station = build_station(east, north)
    total_length = station[-1]

    if lc_links:
        lc_str = ', '.join(
            f"link_{lid}({'L' if hl else ''}{'R' if hr else ''})"
            for lid, hl, hr in lc_links
        )
        rospy.loginfo(f"LC 강제 링크: {lc_str}")

    speed_ms    = SPEED_KMH / 3.6
    dt          = 1.0 / RATE_HZ
    ds_per_step = speed_ms * dt

    rospy.loginfo(
        f"Route: {len(ROUTE_LINK_IDS)} links, {total_length:.1f} m, "
        f"ETA {total_length / speed_ms:.0f} s at {SPEED_KMH} km/h"
    )

    rate = rospy.Rate(RATE_HZ)
    s = 0.0

    while not rospy.is_shutdown():
        if s >= total_length:
            rospy.loginfo("Route complete. Shutting down.")
            break

        e, n, yaw = interpolate_position(east, north, station, s)

        msg       = localization2D_msg()
        msg.time  = rospy.Time.now()
        msg.EPSG  = 5179
        msg.east  = float(e)
        msg.north = float(n)
        msg.yaw   = float(yaw)
        pub.publish(msg)

        s += ds_per_step
        rate.sleep()


if __name__ == '__main__':
    main()
