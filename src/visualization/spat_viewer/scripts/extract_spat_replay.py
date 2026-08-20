#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag → SPaT 방향매칭 리플레이 타임라인 추출 (offline).

~/bag_data 의 SPaT 주행 bag 들을 시간순으로 읽어, 각 시점 ego 위치(host_east/north
→ WGS84)와 ego 진행방향(MANUAVER)에 **수정된 매칭 로직**으로 고른 /spat_merged 신호를
JSON 타임라인으로 뽑는다. spat_viewer/web/replay.html 이 이 JSON 을 재생한다.

매칭 로직은 pyqt_hmi/utils/hmi_state.py 의 수정본과 동일하다(아래 복제):
  _DIR_NAMES = {-1:("LEFT",), 0:("STR","STRAIGHT"), 1:("RIGHT",)}
  movement_matches_manuaver(name, man) := name in _DIR_NAMES[man]
PyQt5/rospy 를 module-top 에서 import 하는 hmi_state 를 헤드리스로 import 하지 않기 위해
동일 2줄 로직만 복제(verifier 로 등가 입증됨).

사용:
  source devel/setup.bash
  python3 extract_spat_replay.py                 # ~/bag_data 의 2026-06-24 3개 bag
  python3 extract_spat_replay.py <bag1> <bag2>   # 명시
출력: spat_viewer/web/data/replay_timeline.json
"""
import os
import sys
import glob
import json

import rosbag
from pyproj import Transformer

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'web', 'data', 'replay_timeline.json')

TOPIC_LOC = '/localization/to_control_team'
TOPIC_SPAT = '/spat_merged'

SAMPLE_DT = 0.2          # ego 다운샘플 주기(s) → 5 Hz
EPSG_SRC = 'EPSG:5179'
EPSG_DST = 'EPSG:4326'

# ── 수정된 방향 매칭 (hmi_state.py 복제) ──────────────────────────
_DIR_NAMES = {-1: ("LEFT",), 0: ("STR", "STRAIGHT"), 1: ("RIGHT",)}
def movement_matches_manuaver(name, manuaver):
    return name in _DIR_NAMES.get(manuaver, ())

# J2735 MovementPhaseState → 표시 색 (hmi_state.py 와 동일): 1=GREEN 2=AMBER 3=RED 0=off
def phase_to_color(phase):
    if phase == 5 or phase == 6:
        return 1
    if phase == 7 or phase == 8:
        return 2
    if phase == 3:
        return 3
    return 0

DIR_LABEL = {-1: 'LEFT', 0: 'STR', 1: 'RIGHT'}


def select_signal(spat, iid, sg, man):
    """수정된 소비자 로직과 동일한 방향 매칭. 성공 dict / 실패 None."""
    if spat is None or iid == 0:
        return None
    for d in spat.data:
        if int(d.IntersectionID) != iid:
            continue
        if sg and int(d.Movements.SignalGroupID) != sg:   # sg==0 이면 SG 필터 생략
            continue
        if not movement_matches_manuaver(d.Movements.MovementStateName, man):
            continue
        phase = int(d.Movements.MovementPhaseStatus)
        return {
            'color': phase_to_color(phase),
            'phase': phase,
            'tcd': int(d.Movements.TimeChangeDetails),   # J2735 minEndTime (0.1s)
            'move': d.Movements.MovementStateName,
            'm_sg': int(d.Movements.SignalGroupID),
        }
    return None


def find_default_bags():
    pat = os.path.expanduser('~/bag_data/2026-06-24-14-37-33_2026-06-24*.bag')
    return sorted(glob.glob(pat))


def main():
    bags = sys.argv[1:] or find_default_bags()
    if not bags:
        print('[extract] no bags found (~/bag_data/2026-06-24-...)', file=sys.stderr)
        return 1
    print('[extract] bags:')
    for b in bags:
        print('   ', b)

    tf = Transformer.from_crs(EPSG_SRC, EPSG_DST, always_xy=True)

    samples = []
    latest_spat = None
    t0 = None
    last_kept = None
    n_loc = 0

    for b in bags:
        with rosbag.Bag(b) as bag:
            for topic, msg, t in bag.read_messages(topics=[TOPIC_LOC, TOPIC_SPAT]):
                ts = t.to_sec()
                if t0 is None:
                    t0 = ts
                if topic == TOPIC_SPAT:
                    latest_spat = msg
                    continue
                # topic == TOPIC_LOC
                n_loc += 1
                if last_kept is not None and (ts - last_kept) < SAMPLE_DT:
                    continue
                last_kept = ts
                iid = int(msg.look_at_IntersectionID)
                sg = int(msg.look_at_signalGroupID)
                man = int(msg.MANUAVER)
                lon, lat = tf.transform(msg.host_east, msg.host_north)
                sig = select_signal(latest_spat, iid, sg, man)
                samples.append({
                    't': round(ts - t0, 2),
                    'lat': round(float(lat), 7),
                    'lon': round(float(lon), 7),
                    'yaw': round(float(msg.host_yaw), 4),       # ENU rad
                    'link': int(msg.LINK_ID),
                    'spd': int(msg.Speed_Limit),
                    'iid': iid,
                    'sg': sg,
                    'man': man,
                    'man_l': DIR_LABEL.get(man, str(man)),
                    'sig': sig,          # None = ego 방향 신호 매칭 실패(안전 리셋)
                })
        print('[extract]   done %s (loc read=%d, kept=%d)' % (os.path.basename(b), n_loc, len(samples)))

    doc = {
        'schema_version': 1,
        'source_bags': [os.path.basename(b) for b in bags],
        'match_logic': 'hmi_state.movement_matches_manuaver (fixed): -1->LEFT, 0->{STR,STRAIGHT}, 1->RIGHT',
        'epsg_source': EPSG_SRC,
        'sample_hz': round(1.0 / SAMPLE_DT, 1),
        'count': len(samples),
        'duration_s': round(samples[-1]['t'], 2) if samples else 0,
        'samples': samples,
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, separators=(',', ':'))

    matched = sum(1 for s in samples if s['sig'])
    signaled = sum(1 for s in samples if s['iid'] != 0)
    print('[extract] wrote %s' % OUT)
    print('[extract] samples=%d duration=%.1fs  (iid!=0: %d, matched: %d)'
          % (len(samples), doc['duration_s'], signaled, matched))
    return 0


if __name__ == '__main__':
    sys.exit(main())
