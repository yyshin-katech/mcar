uint32 frame_index
uint8 not_used
int32 frame_type

# TravelerDataFrame.msgId CHOICE
uint8 msg_id_choice
string further_info_id
bool road_sign_id_present
int32 road_sign_lat
int32 road_sign_long
bool road_sign_elevation_present
int32 road_sign_elevation
string road_sign_view_angle_hex
uint32 road_sign_view_angle_bits
bool road_sign_mutcd_code_present
int32 road_sign_mutcd_code

bool start_year_present
uint16 start_year
uint32 start_time
uint16 duration_time
int32 priority
uint8 not_used1
uint8 not_used2
uint8 not_used3

int32 content_choice
bool url_present
string url

v2x_tim_region_msg[] regions
v2x_tim_content_msg[] contents
