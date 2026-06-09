#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
블랙박스 레코더 (이벤트 트리거 lookback 저장)

설계: 경량 모니터(저rate 2토픽만 구독) + C++ `rosbag record` 롤링 버퍼.
  - 자율주행(/sensors/ioniq5_ad_can.autonomous_mode==1) 동안 rosbag record 가
    기본 토픽(LiDAR/인지 EXCLUDE_TOPICS 제외)을 작은 chunk 로 롤링 기록(--split --max-splits).
    → 무거운 토픽 부하는 C++ recorder 가 처리(파이썬 전토픽 버퍼링은 실차 부하에서 막힘).
  - /localization/to_control_team 의 Take_Over_Request 가 0→1 상승하는 순간,
    자율주행 ON(또는 직전 grace초 내 ON)이면 트리거.
  - 트리거 시각 T 기준 [T-PRE, T+POST] 를 롤링 chunk 들에서 잘라 블랙박스 bag 으로 저장.
  - TOR=1 과 함께 자율주행이 풀려도 POST(2s) 캡처는 진행(게이트는 상승엣지 순간만 확인).

rosbag_snapshot 패키지가 없어 직접 구현. 트리거 시 recorder 를 잠시 멈췄다(chunk flush)
재개하므로 ~0.5s 공백이 생긴다.
"""
import os
import re
import glob
import signal
import threading
import datetime
import subprocess

import rospy
import rosbag

from mmc_msgs.msg import to_control_team_from_local_msg
from katech_custom_msgs.msg import ioniq5_ad_can_msg

# REC OFF 모드와 동일하게 제외하는 LiDAR/인지 토픽
EXCLUDE_TOPICS = [
    "/left/rslidar_packets_difop",
    "/middle/rslidar_packets",
    "/middle/rslidar_packets_difop",
    "/percept_background_rviz",
    "/percept_cluster_rviz",
    "/percept_ground_rviz",
    "/percept_non_ground_rviz",
    "/percept_origin_rviz",
    "/percept_sematic_rviz",
    "/percept_topic",
    "/perception_info_rviz",
    "/perception_pre_known_rviz",
    "/right/rslidar_packets_difop",
]


class BlackboxRecorder(object):
    def __init__(self):
        self.pre = float(rospy.get_param("~pre_seconds", 10.0))
        self.post = float(rospy.get_param("~post_seconds", 2.0))
        self.chunk = float(rospy.get_param("~chunk_seconds", 5.0))     # 롤링 chunk 길이
        # 롤링 버퍼가 [T-pre, T+post] 를 덮도록 충분한 chunk 수 유지
        need = self.pre + self.post + self.chunk
        self.max_splits = int(rospy.get_param("~max_splits", max(3, int(need / self.chunk) + 1)))
        self.auto_grace = float(rospy.get_param("~auto_grace_seconds", 1.0))

        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/bag_data/blackbox"))
        self.buf_dir = os.path.join(self.output_dir, ".rollbuf")
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.buf_dir, exist_ok=True)

        self.exclude_regex = "(" + "|".join(re.escape(t) + "$" for t in EXCLUDE_TOPICS) + ")"

        self.lock = threading.Lock()
        self.recorder = None
        self.capturing = False
        self.autonomous_mode = 0
        self.last_auto_on = rospy.Time(0)
        self.prev_tor = 0

        rospy.Subscriber("/localization/to_control_team",
                         to_control_team_from_local_msg, self._cb_local, queue_size=50)
        rospy.Subscriber("/sensors/ioniq5_ad_can",
                         ioniq5_ad_can_msg, self._cb_ad_can, queue_size=50)

        # 롤링 버퍼 recorder 는 항상 가동(연속 임시 버퍼, max-splits 로 크기 제한).
        # 모드에 따라 start/stop 하면 TOR=1과 동시에 모드가 풀릴 때 레이스로 POST 가 잘림.
        # → 버퍼는 항상 채우고, 실제 저장(트리거)만 자율주행으로 게이팅.
        self._start_recorder()
        rospy.on_shutdown(self._stop_recorder)
        rospy.loginfo("blackbox_recorder: pre=%.0fs post=%.0fs chunk=%.0fs max_splits=%d dir=%s",
                      self.pre, self.post, self.chunk, self.max_splits, self.output_dir)

    # ── 롤링 recorder 관리 ──────────────────────────────────────
    def _start_recorder(self):
        with self.lock:
            if self.recorder is not None:
                return
            self._clear_buffer()
            cmd = ["rosbag", "record", "-a", "-x", self.exclude_regex,
                   "--split", "--duration=%d" % int(self.chunk),
                   "--max-splits=%d" % self.max_splits,
                   "-o", os.path.join(self.buf_dir, "roll")]
            self.recorder = subprocess.Popen(cmd, preexec_fn=os.setsid)
            rospy.loginfo("blackbox: 롤링 recorder 시작")

    def _stop_recorder(self):
        with self.lock:
            if self.recorder is None:
                return
            try:
                os.killpg(os.getpgid(self.recorder.pid), signal.SIGINT)
                self.recorder.wait()
            except Exception:
                pass
            self.recorder = None

    def _clear_buffer(self):
        for f in glob.glob(os.path.join(self.buf_dir, "roll*")):
            try:
                os.remove(f)
            except OSError:
                pass

    # ── 트리거 로직 ─────────────────────────────────────────────
    def _cb_ad_can(self, msg):
        self.autonomous_mode = msg.autonomous_mode
        if msg.autonomous_mode == 1:
            self.last_auto_on = rospy.Time.now()

    def _cb_local(self, msg):
        tor = msg.Take_Over_Request
        # 0→1 상승 + (자율주행 ON 또는 직전 grace초 내 ON; TOR과 동시 해제 케이스 포함) + 캡처중 아님
        if self.prev_tor == 0 and tor == 1 and not self.capturing:
            recently_auto = (rospy.Time.now() - self.last_auto_on).to_sec() <= self.auto_grace
            if self.autonomous_mode == 1 or recently_auto:
                self._trigger()
        self.prev_tor = tor

    def _trigger(self):
        self.capturing = True
        trigger_time = rospy.Time.now().to_sec()
        rospy.logwarn("blackbox: TAKE_OVER_REQUEST 트리거 @ %.3f → [-%.0f, +%.0f]s 저장 예약",
                      trigger_time, self.pre, self.post)
        threading.Timer(self.post, self._finalize, args=(trigger_time,)).start()

    def _finalize(self, trigger_time):
        # recorder 정지(활성 chunk flush) 후 롤링 chunk 에서 윈도우 추출
        self._stop_recorder()
        lo, hi = trigger_time - self.pre, trigger_time + self.post
        chunks = sorted(glob.glob(os.path.join(self.buf_dir, "roll*.bag")))
        ts = datetime.datetime.fromtimestamp(trigger_time).strftime("%Y-%m-%d-%H-%M-%S")
        out = os.path.join(self.output_dir, "blackbox_%s.bag" % ts)
        n = 0
        try:
            with rosbag.Bag(out, "w") as ob:
                for ch in chunks:
                    try:
                        with rosbag.Bag(ch) as ib:
                            for topic, msg, t in ib.read_messages(raw=True):
                                if lo <= t.to_sec() <= hi:
                                    ob.write(topic, msg, t, raw=True)
                                    n += 1
                    except Exception as e:
                        rospy.logwarn("blackbox: chunk 읽기 실패 %s: %s", ch, e)
            if n > 0:
                rospy.logwarn("blackbox: 저장 완료 %s (%d msgs)", out, n)
            else:
                rospy.logwarn("blackbox: 윈도우 내 메시지 없음 → 빈 저장")
        except Exception as e:
            rospy.logerr("blackbox: 저장 실패: %s", e)

        self.capturing = False
        self._clear_buffer()
        # 롤링 버퍼 재개 (항상 가동)
        self._start_recorder()


def main():
    rospy.init_node("blackbox_recorder")
    BlackboxRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
