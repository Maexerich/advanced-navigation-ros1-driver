#include <stdint.h>
#include <string.h>
#include "an_packet_protocol.h"
#include "subsonus_packets.h"

int decode_subsonus_system_state_packet(subsonus_system_state_packet_t *subsonus_system_state_packet, an_packet_t *an_packet)
{
    if(an_packet->id == 20 && an_packet->length == 116)
    {
        memcpy(subsonus_system_state_packet, an_packet->data, 116);
        return 0;
    }
    return 1;
}

int decode_subsonus_remote_track_packet(subsonus_remote_track_packet_t *remote_track_packet, an_packet_t *an_packet)
{
    if(an_packet->id == 24 && an_packet->length == 211)
    {
        memcpy(remote_track_packet, an_packet->data, 211);
        return 0;
    }
    return 1;
}

int decode_subsonus_remote_state_packet(subsonus_remote_state_packet_t *remote_state_packet, an_packet_t *an_packet)
{
    if(an_packet->id == 25 && an_packet->length == 130)
    {
        memcpy(remote_state_packet, an_packet->data, 130);
        return 0;
    }
    return 1;
}

int decode_subsonus_remote_orientation_packet(subsonus_remote_orientation_packet_t *remote_orientation_packet, an_packet_t *an_packet)
{
    if(an_packet->id == 101 && an_packet->length == 23)
    {
        memcpy(remote_orientation_packet, an_packet->data, 23);
        return 0;
    }
    return 1;
}
