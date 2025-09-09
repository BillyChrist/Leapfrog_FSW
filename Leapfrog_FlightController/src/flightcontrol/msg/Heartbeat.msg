# IMU Data
float32 roll_deg
float32 pitch_deg
float32 yaw_deg
float32 acc_x_g
float32 acc_y_g
float32 acc_z_g
float32 angvel_x_degs
float32 angvel_y_degs
float32 angvel_z_degs

# TVC Data
float32 tvc_a_pos
float32 tvc_b_pos

# GPS Data
#float32 lat_lon

# Engine Telemetry
int32 engine_turbine_rpm
int32 engine_rpm_setpoint
int32 engine_egt_c
float32 engine_pump_voltage
uint8 engine_turbine_state
uint8 engine_off_condition
uint8 engine_throttle_percent
float32 engine_current_a

# Altitude
int32 altitude

# System Status
uint8 heartbeat_counter
bool guidance_internal
bool enable_acs
bool enable_tvc
#bool enable_gps
bool enable_engine
bool imu_calibration_status
#uint8 tvc_calibration
#uint8 gps_calibration

# Additional fields for STM32 compatibility
uint8 error_flags
float32 engine_thrust

# Additional fields for compatibility
float32 tvc_angle1
float32 tvc_angle2
float32 engine_hover_height
bool safe_engine
bool power_engine
