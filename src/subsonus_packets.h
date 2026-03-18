#ifndef SUBSONUS_PACKETS_H
#define SUBSONUS_PACKETS_H

#include <stdint.h>

#pragma pack(push, 1)

// Packet ID 20 (Subsonus System State Packet)
// Length 116
typedef struct {
    uint32_t system_status;
    uint32_t filter_status;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double latitude;
    double longitude;
    double height;
    float velocity_north;
    float velocity_east;
    float velocity_down;
    float body_acceleration_x;
    float body_acceleration_y;
    float body_acceleration_z;
    float g_force;
    float roll;
    float pitch;
    float heading;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float latitude_stddev;
    float longitude_stddev;
    float height_stddev;
    float roll_stddev;
    float pitch_stddev;
    float heading_stddev;
} subsonus_system_state_packet_t;

// Packet ID 24 (Remote Track Packet)
// Length 211
typedef struct {
    uint16_t device_address;
    uint8_t tracking_status;
    uint32_t system_status;
    uint32_t filter_status;
    uint32_t data_valid_flags;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double local_latitude;
    double local_longitude;
    double local_height;
    float local_velocity_north;
    float local_velocity_east;
    float local_velocity_down;
    float local_roll;
    float local_pitch;
    float local_heading;
    float local_latitude_stddev;
    float local_longitude_stddev;
    float local_height_stddev;
    float local_roll_stddev;
    float local_pitch_stddev;
    float local_heading_stddev;
    float local_depth;
    uint32_t remote_age_microseconds;
    float remote_range;
    float remote_azimuth;
    float remote_elevation;
    float remote_position_raw_x;
    float remote_position_raw_y;
    float remote_position_raw_z;
    float remote_position_x;
    float remote_position_y;
    float remote_position_z;
    float remote_north;
    float remote_east;
    float remote_down;
    double remote_latitude;
    double remote_longitude;
    double remote_height;
    float remote_range_stddev;
    float remote_azimuth_stddev;
    float remote_elevation_stddev;
    float remote_latitude_stddev;
    float remote_longitude_stddev;
    float remote_height_stddev;
    float remote_depth;
    int8_t signal_level;
    int8_t signal_to_noise_ratio;
    uint8_t signal_correlation_ratio;
    uint8_t signal_correlation_interference;
    uint32_t reserved;
} subsonus_remote_track_packet_t;

// Packet ID 25 (Remote State Packet)
// Length 130
typedef struct {
    uint16_t device_address;
    uint32_t system_status_valid_flags;
    uint32_t filter_status_valid_flags;
    uint32_t system_status;
    uint32_t filter_status;
    uint32_t data_valid_flags;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double latitude;
    double longitude;
    double height;
    float velocity_north;
    float velocity_east;
    float velocity_down;
    float body_acceleration_x;
    float body_acceleration_y;
    float body_acceleration_z;
    float g_force;
    float roll;
    float pitch;
    float heading;
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    float latitude_stddev;
    float longitude_stddev;
    float height_stddev;
    float roll_stddev;
    float pitch_stddev;
    float heading_stddev;
} subsonus_remote_state_packet_t;

// Packet ID 101 (Remote Orientation Packet)
// Length 23
typedef struct {
    uint16_t device_address;
    uint32_t observer_unix_time_seconds;
    uint32_t observer_microseconds;
    uint8_t data_valid_flags;
    float roll;
    float pitch;
    float heading;
} subsonus_remote_orientation_packet_t;

#pragma pack(pop)

#endif // SUBSONUS_PACKETS_H

#ifdef __cplusplus
extern "C"
{
#endif

int decode_subsonus_system_state_packet(subsonus_system_state_packet_t *subsonus_system_state_packet, an_packet_t *an_packet);
int decode_subsonus_remote_track_packet(subsonus_remote_track_packet_t *remote_track_packet, an_packet_t *an_packet);
int decode_subsonus_remote_state_packet(subsonus_remote_state_packet_t *remote_state_packet, an_packet_t *an_packet);
int decode_subsonus_remote_orientation_packet(subsonus_remote_orientation_packet_t *remote_orientation_packet, an_packet_t *an_packet);

#ifdef __cplusplus
}
#endif
