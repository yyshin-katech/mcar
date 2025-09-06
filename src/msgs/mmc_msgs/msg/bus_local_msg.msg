time time
int8 lane_id  # 1: mainlane1, 2: mainlane2, 3: merge_in, 4: out
string lane_name

float32 distance_to_entry_end
float32 distance_to_exit_start

float32 host_east   # global X [m]
float32 host_north  # global Y [m]
float32 host_yaw    # global yaw [rad]
float32 host_yawrate    # global yawrate [rad/s]
float32 v           # global_v [m/s]

float32 station          # (s) station in current lane [m]
float32 lateral_offset   # (d) lateral offset in current lane [m]

int32 waypoint_index     # index of waypoint
