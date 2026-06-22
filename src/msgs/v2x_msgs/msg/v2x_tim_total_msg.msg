std_msgs/Header header

# J2735 MessageFrame / TravelerInformation
uint16 message_id
uint8 msg_cnt
bool time_stamp_present
uint32 time_stamp
bool packet_id_present
string packet_id
bool url_b_present
string url_b

v2x_tim_dataframe_msg[] data_frames
v2x_tim_regional_msg[] regional

# Parsed local extension values found inside regional OCTET STRING JSON.
bool standard_time_present
string standard_time
bool rsu_latitude_present
float64 rsu_latitude
bool rsu_longitude_present
float64 rsu_longitude
bool north_pedes_present
bool north_pedes
bool east_pedes_present
bool east_pedes
bool south_pedes_present
bool south_pedes
bool west_pedes_present
bool west_pedes
