# GearInfo (ID 117)
uint8 gear_status

# TurnSignalInfo (ID 116)
uint8 turn_signal_status

# LongitudinalInfo (ID 115)
float64 motor_rpm
float64 acceleration_pedal_pos
float64 brake_pedal_pos
float64 brake_pressure

# SteeringInfo (ID 114)
float64 steering_angle
float64 steering_torque
float64 steering_angle_rate

# WheelInfo (ID 113, CAN FD)
uint8 wheel_dir_fl
uint8 wheel_dir_fr
uint8 wheel_dir_rl
uint8 wheel_dir_rr
uint8 wheel_pulse_fl
uint8 wheel_pulse_fr
uint8 wheel_pulse_rl
uint8 wheel_pulse_rr
float64 wheel_speed_fl
float64 wheel_speed_fr
float64 wheel_speed_rl
float64 wheel_speed_rr

# DynamicInfo (ID 112, CAN FD)
float64 long_acceleration
float64 lat_acceleration
float64 roll_rate
float64 pitch_rate
float64 yaw_rate

# TurnSignalControl (ID 84)
uint8 turn_signal_control_mode
uint8 target_turn_signal

# LateralControl (ID 83)
uint8 lat_steering_control_mode
float64 lat_target_steering_angle

# GearControl (ID 82)
uint8 gear_control_mode
uint8 target_gear

# LongitudinalControl (ID 81)
uint8 longitudinal_control_mode
float64 target_acceleration

# GearState (ID 21)
uint8 gear_ctrl_state
uint8 gear_override
uint8 gear_error_code

# TurnSignalState (ID 20)
uint8 turnsignal_ctrl_status
uint8 turnsignal_override
uint8 turnsignal_error_code

# LongitudinalState (ID 19)
uint8 longitudinal_ctrl_state
uint8 longitudinal_override
uint8 longitudinal_error_code

# LateralState (ID 18)
uint8 lateral_ctrl_state
uint8 lateral_override
uint8 lateral_error_code
