# OperationControl (ID 17)
uint8 operation_sw
uint8 autonomous_sw
uint8 emergency_sw

# LateralControl (ID 83)
uint8 steering_control_mode
float64 target_steering_angle

# GearControl (ID 82)
uint8 gear_control_mode
uint8 target_gear

# BrainState (ID 33)
uint8 brain_status

# AutonomousState (ID 16)
uint8 operation_mode
uint8 autonomous_mode
uint16 error_code
uint16 warning_code
