#ifndef UWB_PACKET_H_
#define UWB_PACKET_H_

#include "dw3000.h"

// Wire format after the ranging timestamps: magic[2], version, type, sequence,
// source_id[2] LE, flags, responder uptime_ms[4] LE, payload_length, payload.
#define UWB_APP_FRAME_PREFIX_LEN 18
#define UWB_APP_PACKET_HEADER_LEN 13
#define UWB_APP_PAYLOAD_OFFSET (UWB_APP_FRAME_PREFIX_LEN + UWB_APP_PACKET_HEADER_LEN)
#define UWB_APP_PAYLOAD_MAX_LEN \
    (FRAME_LEN_MAX - UWB_APP_FRAME_PREFIX_LEN - UWB_APP_PACKET_HEADER_LEN - FCS_LEN)

#define UWB_APP_MAGIC_0 0x55
#define UWB_APP_MAGIC_1 0x57
#define UWB_APP_VERSION 1
#define UWB_APP_TYPE_SENSOR_DATA 1

#define UWB_APP_MAGIC_0_OFFSET 0
#define UWB_APP_MAGIC_1_OFFSET 1
#define UWB_APP_VERSION_OFFSET 2
#define UWB_APP_TYPE_OFFSET 3
#define UWB_APP_SEQUENCE_OFFSET 4
#define UWB_APP_SOURCE_ID_OFFSET 5
#define UWB_APP_FLAGS_OFFSET 7
#define UWB_APP_TIMESTAMP_MS_OFFSET 8
#define UWB_APP_PAYLOAD_LEN_OFFSET 12

static inline void uwb_packet_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static inline uint16_t uwb_packet_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static inline void uwb_packet_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static inline uint32_t uwb_packet_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

#endif
