time time

int8 is_stop_line
int8 look_at_signalGroupID # 100,200...
uint16 look_at_IntersectionID # None 
int8 NEXT_LINK_ID
int8 LINK_ID
float32 distance_to_lane_end
int8 have_to_LangeChange_right #[0/1]
int8 have_to_LangeChange_left
int8 right_LaneChange_avail 
int8 left_LaneChange_avail
int8 Speed_Limit #[km/h]
int8 guard_zone

int8 lane_id  # 1: mainlane1, 2: mainlane2, 3: merge_in, 4: out
string lane_name



float64 host_east   # global X [m]
float64 host_north  # global Y [m]
float64 host_yaw    # global yaw [rad]


float64 station          # (s) station in current lane [m]
float64 lateral_offset   # (d) lateral offset in current lane [m]


int32 waypoint_index     # index of waypoint

float32 yaw_error_size # [radian]

int8 On_ODD
int8 Road_State
float32 distance_out_of_ODD # [m]