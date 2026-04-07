#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
from mmc_msgs.msg import to_control_team_from_local_msg

class LateralOffsetRelay:
    def __init__(self):
        rospy.init_node('lateral_offset_relay')
        self.float_pub = rospy.Publisher('/rviz/lateral_offset', Float32, queue_size=1)
        self.lane_end_pub = rospy.Publisher('/rviz/jsk/lane_end_text', OverlayText, queue_size=1)
        self.stopline_pub = rospy.Publisher('/rviz/jsk/stopline_popup', OverlayText, queue_size=1)
        rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg, self.callback)
        rospy.spin()

    def callback(self, msg):
        # lateral offset
        self.float_pub.publish(Float32(data=msg.lateral_offset))

        # distance_to_lane_end
        txt = OverlayText()
        txt.action = txt.ADD
        txt.font = "DejaVu Sans Mono"
        txt.text_size = 18
        txt.width = 300
        txt.height = 50
        txt.left = 10
        txt.top = 375
        txt.text = f"Lane End: {msg.distance_to_lane_end:.1f} m"

        fg = ColorRGBA()
        fg.r, fg.g, fg.b, fg.a = 0.1, 1.0, 0.94, 1.0
        txt.fg_color = fg
        bg = ColorRGBA()
        bg.r, bg.g, bg.b, bg.a = 0.0, 0.0, 0.0, 0.3
        txt.bg_color = bg
        self.lane_end_pub.publish(txt)

        # is_stop_line popup
        popup = OverlayText()
        popup.font = "DejaVu Sans Mono"
        popup.text_size = 20
        popup.width = 250
        popup.height = 50
        popup.left = 10
        popup.top = 710

        if msg.is_stop_line == 1:
            popup.action = popup.ADD
            popup.text = "STOP LINE"
            fg2 = ColorRGBA()
            fg2.r, fg2.g, fg2.b, fg2.a = 1.0, 0.2, 0.2, 1.0
            popup.fg_color = fg2
            bg2 = ColorRGBA()
            bg2.r, bg2.g, bg2.b, bg2.a = 0.0, 0.0, 0.0, 0.7
            popup.bg_color = bg2
        else:
            popup.action = popup.ADD
            popup.text = ""
            fg2 = ColorRGBA()
            fg2.r, fg2.g, fg2.b, fg2.a = 0.0, 0.0, 0.0, 0.0
            popup.fg_color = fg2
            bg2 = ColorRGBA()
            bg2.r, bg2.g, bg2.b, bg2.a = 0.0, 0.0, 0.0, 0.0
            popup.bg_color = bg2

        self.stopline_pub.publish(popup)

if __name__ == '__main__':
    LateralOffsetRelay()
