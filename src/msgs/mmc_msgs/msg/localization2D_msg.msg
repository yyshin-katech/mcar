time time         # rospy.Time, ros::Time
string localizer_type     # EKF, UKF, PF
bool gps_initialized      # is gps initialized?
int32 EPSG                # espn of east, north
float32 without_gps       # [s]

float64 east        # east [m]
float64 north       # north [m]
float64 yaw         # yaw [rad]
float64 v           # velocity [m/s]

float64 sig_east    # std of east [m]
float64 sig_north   # std of north [m]
float64 sig_yaw     # std of yaw [rad]
float64 sig_v       # std of velocity [m/s]

int32 NP         # number of particles
float64[] pe     # particle east
float64[] pn     # particle north
float64[] pyaw   # particle yaw
float64[] pv     # particle velocity
