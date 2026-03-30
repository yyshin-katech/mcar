#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# TB_senario_map.shp에서 루프 경로를 따라 /localization/pose_2d_gps를 퍼블리시하는 테스트 노드
# 속도: 30 km/h, 주기: 20 Hz
# 경로: fid 순서 61,219,233,234,241,246,250,261,272,428,279,282,287,300,314,321,325,331,337,339,
#              348,451,352,356,448,367,377,386,388,393,397,400,409,52,652 -> (반복)

import rospy
import numpy as np
import os
import fiona
import pyproj

from mmc_msgs.msg import localization2D_msg

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SHP_FILE_PATH = os.path.join(SCRIPT_DIR, 'shp_map', 'TB_senario_map.shp')

LOOP_LINK_IDS = [
    'A2209W000073',   # fid=61
    'A2209W000234',   # fid=219
    'A2209W000248',   # fid=233
    'A2209W000249',   # fid=234
    'A2209W000256',   # fid=241
    'A2209W000261',   # fid=246
    'A2209W000265',   # fid=250
    'A2209W000276',   # fid=261
    'A2209W000287',   # fid=272
    'A2209W000472',   # fid=428
    'A2209W000294',   # fid=279
    'A2209W000297',   # fid=282
    'A2209W000302',   # fid=287
    'A2209W000315',   # fid=300
    'A2209W000329',   # fid=314
    'A2209W000336',   # fid=321
    'A2209W000340',   # fid=325
    'A2209W000346',   # fid=331
    'A2209W000352',   # fid=337
    'A2209W000354',   # fid=339
    'A2209W000363',   # fid=348
    'A2209W000506',   # fid=451
    'A2209W000368',   # fid=352
    'A2209W000385',   # fid=356
    'A2209W000503',   # fid=448
    'A2209W000405',   # fid=367
    'A2209W000415',   # fid=377
    'A2209W000424',   # fid=386
    'A2209W000426',   # fid=388
    'A2209W000437',   # fid=393
    'A2209W000441',   # fid=397
    'A2209W000444',   # fid=400
    'A2209W000453',   # fid=409
    'A2209W000064',   # fid=52
    'A2209W000991',   # fid=652
]

SPEED_KMH = 30.0
RATE_HZ   = 20.0

_shp_to_5179 = pyproj.Transformer.from_crs('EPSG:32652', 'EPSG:5179', always_xy=True)


def load_loop_waypoints():
    """루프 링크의 웨이포인트를 순서대로 이어붙여 반환 (east, north 배열)"""
    id_to_coords = {}
    os.environ['SHAPE_RESTORE_SHX'] = 'YES'
    with fiona.open(SHP_FILE_PATH) as shp:
        for feat in shp:
            lid = feat['properties']['ID']
            if lid in LOOP_LINK_IDS:
                coords = np.array(feat['geometry']['coordinates'])
                e, n = _shp_to_5179.transform(coords[:, 0], coords[:, 1])
                id_to_coords[lid] = (e, n)

    east_all, north_all = [], []
    for lid in LOOP_LINK_IDS:
        e, n = id_to_coords[lid]
        if east_all:
            # 이전 링크 끝점과 중복 방지
            east_all.extend(e[1:].tolist())
            north_all.extend(n[1:].tolist())
        else:
            east_all.extend(e.tolist())
            north_all.extend(n.tolist())

    return np.array(east_all), np.array(north_all)


def build_station(east, north):
    """누적 거리(station) 배열 계산"""
    ds = np.sqrt(np.diff(east)**2 + np.diff(north)**2)
    station = np.zeros(len(east))
    station[1:] = np.cumsum(ds)
    return station


def interpolate_position(east, north, station, s):
    """station s에 해당하는 (e, n, yaw) 선형 보간"""
    s = s % station[-1]
    idx = np.searchsorted(station, s, side='right') - 1
    idx = np.clip(idx, 0, len(station) - 2)
    seg_len = station[idx + 1] - station[idx]
    if seg_len < 1e-9:
        ratio = 0.0
    else:
        ratio = (s - station[idx]) / seg_len

    e = east[idx] + ratio * (east[idx + 1] - east[idx])
    n = north[idx] + ratio * (north[idx + 1] - north[idx])

    dx = east[idx + 1] - east[idx]
    dy = north[idx + 1] - north[idx]
    yaw = np.arctan2(dy, dx)

    return e, n, yaw


def main():
    rospy.init_node('test_loop_publisher')
    pub = rospy.Publisher('/localization/pose_2d_gps', localization2D_msg, queue_size=1)

    rospy.loginfo("test_loop_publisher: loading loop waypoints...")
    east, north = load_loop_waypoints()
    station = build_station(east, north)
    total_length = station[-1]

    speed_ms = SPEED_KMH / 3.6          # m/s
    dt = 1.0 / RATE_HZ                   # 초/스텝
    ds_per_step = speed_ms * dt          # 스텝당 이동 거리

    rospy.loginfo(f"Loop length: {total_length:.1f} m, {total_length / speed_ms:.1f} s/lap at {SPEED_KMH} km/h")
    rospy.loginfo(f"Publishing at {RATE_HZ} Hz, step: {ds_per_step:.4f} m")

    rate = rospy.Rate(RATE_HZ)
    s = 0.0

    while not rospy.is_shutdown():
        e, n, yaw = interpolate_position(east, north, station, s)

        msg = localization2D_msg()
        msg.time  = rospy.Time.now()
        msg.EPSG  = 5179
        msg.east  = float(e)
        msg.north = float(n)
        msg.yaw   = float(yaw)
        pub.publish(msg)

        s = (s + ds_per_step) % total_length
        rate.sleep()


if __name__ == '__main__':
    main()
