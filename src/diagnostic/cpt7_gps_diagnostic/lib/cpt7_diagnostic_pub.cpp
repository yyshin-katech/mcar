#include <cpt7_diagnostic_pub.h>


CPT7_DIAGNOSTIC_PUB::CPT7_DIAGNOSTIC_PUB()
{
    pub = nh.advertise<katech_diagnostic_msgs::cpt7_gps_diagnostic_msg>("diagnostic/cpt7_gps", 1);
    sub = nh.subscribe("/sensors/gps/bestpos", 1, &CPT7_DIAGNOSTIC_PUB::bestpos_callback, this);
    sub_inspva = nh.subscribe("/sensors/gps/inspva", 1, &CPT7_DIAGNOSTIC_PUB::inspva_callback, this);
    
    timer_ = nh.createTimer(ros::Duration(0.1), &CPT7_DIAGNOSTIC_PUB::timerCallback, this);

    alive_cnt = 0;
    bestpos_cb_cnt = 0;
    inspva_cb_cnt = 0;

    cpt7_msg.IMU_StatCode = 0;
    cpt7_msg.GPS_StatCode = 0;
    cpt7_msg.INS_StatCode = 0;
    cpt7_msg.GPSRTK_StatCode = 0;
    cpt7_msg.GPS_INS_SolutionStat = 0x01;
    cpt7_msg.GPS_INS_AliveCnt = 0;
}

CPT7_DIAGNOSTIC_PUB::~CPT7_DIAGNOSTIC_PUB()
{

}
// solution StatCode
// 0x00: SOL_COMPUTED: Solution computed
// 0x01: INSUFFICIENT_OBS: Insufficient observations
// 0x02: NO_CONVERGENCE: No convergence
// 0x03: SINGULARITY: Singularity at parameters matrix
// 0x04: COV_TRACE: Covariance trace exceeds maximum (trace> 1000m)
// 0x05: TEST_DIST: Test distance exceeded (maximum of 3 rejections if distance >10km)
// 0x06: COLD_START: Not yet converged from cold start
// 0x07: V_H_LIMIT: Height or velocity limits exceeded (in accordance with export licensing restrictions)
// 0x08: VARIANCE: Variance exceeds limits
// 0x09: RESIDUALS: Residuals are too large
// 0x0A ~ 0x0C: Reserved
// 0x0D: INTEGRITY_WARNING: Large residuals make position unreliable
// 0x0E ~ 0x11: Reserved
// 0x12: PENDING
// 0x13: INVALID_FIX: The fixed position, entered using the FIX position command, is not valid
// 0x14: UNAUTHORIZED: Position type is unauthorized
// 0x15: Reserved
// 0x16: INVALID_RATE: The selected logging rate is not supported for this solution type

// gpsrtk StatCode position type
// 0x00: NONE
// 0x01: FIXEDPOS
// 0x02: FIXEDHEIGHT
// 0x03 ~ 0x07: Reserved
// 0x08: DOPPLER_VELOCITY
// 0x09 ~ 0x0F: Reserved
// 0x10: SINGLE
// 0x11: PSRDIFF
// 0x12: WAAS
// 0x13: PROPAGATED
// 0x14 ~ 0x1F: Reserved
// 0x20: L1_FLOAT
// 0x21: Reserved
// 0x22: NARROW_FLOAT
// 0x23 ~ 0x2F: Reserved
// 0x30: L1_INT
// 0x31: WIDE_INT
// 0x32: NARROW_INT
// 0x33: RTK_DIRECT_INS
// 0x34: INS_SBAS
// 0x35: INS_PSRSP
// 0x36: INS_PSRDIFF
// 0x37: INS_RTKFLOAT
// 0x38: INS_RTKFIXED
// 0x39 ~ 0x42: Reserved
// 0x43: EXT_CONSTRAINED
// 0x44: PPP_CONVERGING
// 0x45: PPP
// 0x46: OPERATIONAL
// 0x47: WARNING
// 0x48: OUT_OF_BOUNDS
// 0x49: INS_PPP_CONVERGING
// 0x4A: INS_PPP
// 0x4D: PPP_BASIC_CONERGING
// 0x4E: PPP_BASIC
// 0x4F: INS_PPP_BASIC_CONVERGING
// 0x50: INS_PPP_BASIC
void CPT7_DIAGNOSTIC_PUB::bestpos_callback(const novatel_gps_msgs::NovatelPosition::ConstPtr& msg)
{
    bestpos_cb_cnt++;

    if(msg->position_type == "INS_RTKFIXED")
    {
        // ROS_INFO("Position type is INS_RTKFIXED");
        cpt7_msg.GPSRTK_StatCode = 0x38;            // RTK FIXED
    }
    else
    {
        cpt7_msg.GPSRTK_StatCode = 0;
    }

    if(msg->solution_status == "SOL_COMPUTED")
    {
        cpt7_msg.GPS_INS_SolutionStat = 0x00;
    }
    else
    {
        cpt7_msg.GPS_INS_SolutionStat = 0x01;
    }
}

// INS StatCode
// 0x00: INS_INACTIVE
// 0x01: INS_ALIGNING
// 0x02: INS_HIGH_VARIANCE
// 0x03: INS_SOLUTION_GOOD
// 0x06: INS_SOLUTION_FREE
// 0x07: INS_ALIGNMENT_COMPLETE
// 0x08: DETERMINING_ORIENTATION
// 0x09: WAITING_INITIALPOS
// 0x0A: WAITING_AZIMUTH
// 0x0B: INITIALIZING_BIASES
// 0x0C: MOTION_DETECT
// 0x0E: WAITING_ALIGNMENTORIENTATION

void CPT7_DIAGNOSTIC_PUB::inspva_callback(const novatel_gps_msgs::Inspva::ConstPtr& msg)
{
    inspva_cb_cnt++;
    if(msg->status == "INS_SOLUTION_GOOD")
    {
        cpt7_msg.INS_StatCode = 0x03;
    }
    else if(msg->status == "INS_SOLUTION_FREE")
    {
        cpt7_msg.INS_StatCode = 0x06;
    }
    else
    {
        cpt7_msg.INS_StatCode = 0x00;
    }
}

void CPT7_DIAGNOSTIC_PUB::timerCallback(const ros::TimerEvent&)
{
    if((bestpos_cb_cnt == bestpos_cb_cnt_old) && (inspva_cb_cnt == inspva_cb_cnt_old))
    {

    }
    else
    {
        cpt7_msg.GPS_INS_AliveCnt = alive_cnt++;

        pub.publish(cpt7_msg);

        bestpos_cb_cnt_old = bestpos_cb_cnt;
        inspva_cb_cnt_old = inspva_cb_cnt;
    }

}