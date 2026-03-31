#!/usr/bin/env python3
import rospy
from novatel_gps_msgs.msg import NovatelPosition
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA

class GpsStdRelay:
    def __init__(self):
        rospy.init_node('gps_std_relay')
        self.pub = rospy.Publisher('/rviz/jsk/gps_std_text', OverlayText, queue_size=1)
        rospy.Subscriber('/sensors/gps/bestpos', NovatelPosition, self.callback)
        rospy.spin()

    def callback(self, msg):
        txt = OverlayText()
        txt.action = txt.ADD
        txt.font = "DejaVu Sans Mono"
        txt.text_size = 12
        txt.width = 300
        txt.height = 150
        txt.left = 10
        txt.top = 580

        lat_cm = msg.lat_sigma * 100
        lon_cm = msg.lon_sigma * 100

        txt.text = f"GPS Lat: {lat_cm:6.2f} cm\nGPS Lon: {lon_cm:6.2f} cm"

        fg = ColorRGBA()
        fg.r, fg.g, fg.b, fg.a = 0.1, 1.0, 0.94, 1.0
        txt.fg_color = fg

        bg = ColorRGBA()
        bg.r, bg.g, bg.b, bg.a = 0.0, 0.0, 0.0, 0.3
        txt.bg_color = bg

        self.pub.publish(txt)

if __name__ == '__main__':
    GpsStdRelay()
