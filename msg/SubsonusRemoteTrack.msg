std_msgs/Header header

uint16 device_address
uint8 tracking_status
uint32 system_status
uint32 filter_status
uint32 data_valid_flags
uint32 unix_time_seconds
uint32 microseconds

# Local coordinates
float64 local_latitude
float64 local_longitude
float64 local_height
float32 local_velocity_north
float32 local_velocity_east
float32 local_velocity_down
float32 local_roll
float32 local_pitch
float32 local_heading

# Local standard deviations
float32 local_latitude_stddev
float32 local_longitude_stddev
float32 local_height_stddev
float32 local_roll_stddev
float32 local_pitch_stddev
float32 local_heading_stddev

float32 local_depth

# Remote target
uint32 remote_age_microseconds
float32 remote_range
float32 remote_azimuth
float32 remote_elevation

# Remote position
float32 remote_position_raw_x
float32 remote_position_raw_y
float32 remote_position_raw_z
float32 remote_position_x
float32 remote_position_y
float32 remote_position_z

# Remote relative to local
float32 remote_north
float32 remote_east
float32 remote_down

# Remote global 
float64 remote_latitude
float64 remote_longitude
float64 remote_height

# Remote standard deviations
float32 remote_range_stddev
float32 remote_azimuth_stddev
float32 remote_elevation_stddev
float32 remote_latitude_stddev
float32 remote_longitude_stddev
float32 remote_height_stddev

float32 remote_depth

# Signal quality
int8 signal_level
int8 signal_to_noise_ratio
uint8 signal_correlation_ratio
uint8 signal_correlation_interference
uint32 reserved
