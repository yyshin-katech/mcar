time time

int8 is_stop_line
int8 look_at_signalGroupID # 100,200...
uint16 look_at_IntersectionID # None 
int32 NEXT_LINK_ID
int32 LINK_ID
float32 distance_to_lane_end
int8 have_to_LangeChange_right #[0/1]
int8 have_to_LangeChange_left
int8 right_LaneChange_avail 
int8 left_LaneChange_avail
int8 Speed_Limit #[km/h]
int8 guard_zone

int8 GPS_Over
int8 Take_Over_Request
int8 Wrong_Way_Warn

string lane_name
int32 lane_id


float64 host_east   # global X [m]
float64 host_north  # global Y [m]
float64 host_yaw    # global yaw [rad]
float64 host_altitude  # MSL altitude [m] (ublox hMSL)


float64 station          # (s) station in current lane [m]
float64 lateral_offset   # (d) lateral offset in current lane [m]


int32 waypoint_index     # index of waypoint

float32 yaw_error_size # [radian]

int8 MANUAVER # -1=left, 0=straight/none, 1=right

int8 On_ODD
int8 Road_State
float32 distance_out_of_ODD # [m]

# 안전 취약시간대 (KST) — to_control_team_demo 가 navpvt+TimeRangeSetting 으로 중앙계산
uint8 effective_hour          # 판정에 쓴 KST 시(0-23)
uint8 effective_minute        # 판정에 쓴 KST 분(0-59)
uint8 safety_vulnerable_time  # 0/1 (1=취약시간대)
uint8 time_source             # 0=GPS, 1=manual override(TimeRangeSetting)