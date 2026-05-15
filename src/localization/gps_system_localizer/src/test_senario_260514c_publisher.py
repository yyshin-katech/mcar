#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# senario_260514c mapfiles(mat) 기반 1회 주행 테스트 노드
# - 경로: 105링크, 4854.8m (12 waypoint anchors, Dijkstra-resolved)
# - 갭 일부 존재(최대 37.4m, idx 103 link_2534 진입), 선형 보간 처리
# - 속도: 40 km/h, 주기: 20 Hz

import rospy
import numpy as np
import os
import scipy.io as sio

from mmc_msgs.msg import localization2D_msg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAPFILE_PATH = os.path.join(SCRIPT_DIR, '..', 'mapfiles', 'senario')

ROUTE_LINK_IDS = [
    900, 901, 714, 2538, 2540, 2541, 499, 452, 462, 463,
    469, 547, 548, 417, 442, 951, 2542, 1033, 1017, 1026,
    1027, 1020, 1089, 1090, 1092, 1391, 1401, 1402, 1956, 830,
    1367, 1368, 1342, 1341, 1354, 1949, 1340, 1330, 1326, 1276,
    1274, 1267, 1261, 1258, 1256, 1243, 1242, 1239, 1205, 1204,
    1917, 1918, 913, 871, 1806, 1172, 1167, 1161, 1974, 1973,
    1143, 1140, 1111, 1110, 1101, 1903, 1899, 1085, 1086, 1075,
    1074, 1885, 876, 953, 1878, 973, 946, 800, 680, 679,
    688, 1848, 574, 775, 668, 663, 662, 787, 568, 654,
    2199, 2080, 2399, 2393, 2183, 2376, 2184, 2373, 2374, 2206,
    2503, 2537, 2536, 2534, 2529,
]

SPEED_KMH = 40.0
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
    rospy.init_node('test_senario_260514c_publisher')
    pub = rospy.Publisher('/localization/pose_2d_gps', localization2D_msg, queue_size=1)

    rospy.loginfo("test_senario_260514c_publisher: loading senario waypoints...")
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
