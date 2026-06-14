# GPS UTC 시각 (u-blox NavPVT 분해). CAN GPSTimestamp 송신 / HMI 시각 표시용.
# year 는 UTC 4자리(예: 2026). CAN 인코딩(year-2000)은 소비 측에서 수행.
time stamp          # 수신 ROS 시각
uint16 year         # UTC 연 (4자리)
uint8 month         # 1-12
uint8 day           # 1-31
uint8 hour          # 0-23 (UTC)
uint8 minute        # 0-59
uint8 second        # 0-60
uint16 millisecond  # 0-999
bool valid          # NavPVT VALID_TIME (시각 유효 여부)
