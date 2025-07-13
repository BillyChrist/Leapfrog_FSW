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
int16 altitude

# System Status
uint8 heartbeat_counter
uint8 guidance_internal
bool enable_acs
bool enable_tvc
#bool enable_gps
bool enable_engine

uint8 imu_calibration_status
#uint8 tvc_calibration
#uint8 gps_calibration
