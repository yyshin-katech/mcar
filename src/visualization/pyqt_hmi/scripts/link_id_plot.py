#!/usr/bin/env python3
import rospy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from mmc_msgs.msg import to_control_team_from_local_msg

class LinkIDPlot:
    def __init__(self):
        rospy.init_node('link_id_plot', anonymous=True)
        self.max_points = 500
        self.times = deque(maxlen=self.max_points)
        self.link_ids = deque(maxlen=self.max_points)
        self.start_time = None

        rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, self.callback)

        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.line, = self.ax.plot([], [], 'g-', linewidth=1.5, marker='.', markersize=2)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('LINK_ID')
        self.ax.set_title('LINK_ID')
        self.ax.set_ylim(0, 1400)
        self.ax.set_yticks(range(0, 1401, 200))
        self.ax.grid(True, alpha=0.3)

        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)
        plt.tight_layout()
        plt.show()

    def callback(self, msg):
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now
        self.times.append(now - self.start_time)
        self.link_ids.append(msg.LINK_ID)

    def update_plot(self, frame):
        if len(self.times) > 0:
            self.line.set_data(list(self.times), list(self.link_ids))
            self.ax.set_xlim(max(0, self.times[-1] - 30), self.times[-1] + 1)
        return self.line,

if __name__ == '__main__':
    try:
        LinkIDPlot()
    except rospy.ROSInterruptException:
        pass
