#!/usr/bin/env python3
import rospy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from mmc_msgs.msg import to_control_team_from_local_msg

class LateralOffsetPlot:
    def __init__(self):
        rospy.init_node('lateral_offset_plot', anonymous=True)
        self.max_points = 500
        self.times = deque(maxlen=self.max_points)
        self.offsets = deque(maxlen=self.max_points)
        self.start_time = None

        rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, self.callback)

        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.line, = self.ax.plot([], [], 'b-', linewidth=1.5)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Lateral Offset (m)')
        self.ax.set_title('Lateral Offset')
        self.ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
        self.ax.axhline(y=0.95, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
        self.ax.axhline(y=-0.95, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
        self.ax.set_ylim(-2, 2)
        self.ax.set_yticks([-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2])
        self.ax.grid(True, alpha=0.3)

        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        plt.tight_layout()
        plt.show()

    def callback(self, msg):
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now
        self.times.append(now - self.start_time)
        self.offsets.append(msg.lateral_offset)

    def update_plot(self, frame):
        if len(self.times) > 0:
            self.line.set_data(list(self.times), list(self.offsets))
            self.ax.set_xlim(max(0, self.times[-1] - 30), self.times[-1] + 1)
        return self.line,

if __name__ == '__main__':
    try:
        LateralOffsetPlot()
    except rospy.ROSInterruptException:
        pass
