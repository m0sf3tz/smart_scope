#pragma once

#define MAGIC_BYTE_0_1 0x0102
#define MAGIC_BYTE_2_3 0x0304
#define MAGIC_BYTE_4_5 0x0506
#define MAGIC_BYTE_6_7 0x0708

#define MAX_TLV_SIZE (124)

typedef enum {
  TLV_TYPE_IMU,
  TLV_TYPE_UI,
  TLV_TYPE_MAX
} tlv_message_type_e;

// Only two TLV structures are supported, IMU and Encoder
// wire format is as follows:
//
// (tlv_header_structure)
// (tlv_info)
// (data)
//
// For simplicities sake, we will only send a single TLV at a time

// Made to match the encoding of the IWR6843 header 
// structure. This way we can reuse the same parsing 
// code for both the nordic(what this code is running on)
// as well as the radar.
typedef struct {
    uint16_t    magicWord[4];
    uint32_t    not_used0;
    uint32_t    totalPacketLen;
    uint32_t    not_used1; 
    uint32_t    tlvNumber;
    uint32_t    timeCpuCycles;
    uint32_t    not_used2;
    uint32_t    not_used3;
    uint32_t    not_used4;
} tlv_header_structure;

typedef struct {
  uint32_t type;
  uint32_t data_length;
} tlv_message;

size_t encode_tlv(uint8_t * const, const size_t, const void*, tlv_message_type_e const, size_t const);
