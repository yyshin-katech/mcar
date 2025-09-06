time time         # rospy.Time, ros::Time

uint8 autonomous_mode # system mode
uint8 OFF = 0
uint8 LONGITUDINAL_ONLY = 1
uint8 LATERAL_ONLY = 2
uint8 AUTONOMOUS = 3

uint8 longitudianl_control_mode # 1: AEB, 0: ACC

float32 speed # desired speed (m/s) (for debugging or simulation)
float32 req_acc # desired acceleration (m/s^2) [-3, 3]
float32 jerk # desired jerk (m/s^3) (for debugging or simulation)

float32 target_steering_angle # desired steering angle (deg) (for debugging or simulation)
float32 steering_torque # desired steering torque (Nm) [-7.5, 7.5]

bool warning_display
