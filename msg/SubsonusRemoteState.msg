std_msgs/Header header

uint16 device_address
uint32 system_status_valid_flags
uint32 filter_status_valid_flags
uint32 system_status
uint32 filter_status
uint32 data_valid_flags
uint32 unix_time_seconds
uint32 microseconds

# Position
float64 latitude
float64 longitude
float64 height

# Velocity
float32 velocity_north
float32 velocity_east
float32 velocity_down

# Acceleration
float32 body_acceleration_x
float32 body_acceleration_y
float32 body_acceleration_z

float32 g_force

# Orientation
float32 roll
float32 pitch
float32 heading

# Angular Velocity
float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z

# Standard Deviations
float32 latitude_stddev
float32 longitude_stddev
float32 height_stddev
float32 roll_stddev
float32 pitch_stddev
float32 heading_stddev
