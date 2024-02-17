#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <assert.h>

#include "ui.h"
#include "imu.h"
#include "usb.h"
#include "tlv.h"

void static encode_tlv_header(uint8_t*, size_t);
void static encode_tlv_message(uint8_t*, tlv_message_type_e const, const size_t);
void static encode_tlv_data(uint8_t*, const void*, const size_t);

size_t encode_tlv(uint8_t* const buff,
  const size_t buff_len, const void* data, tlv_message_type_e const type, size_t const data_length)
{

  size_t tlv_len = sizeof(tlv_header_structure) + sizeof(tlv_message) + data_length;
  assert(tlv_len <= buff_len); 
  
  encode_tlv_header(buff, tlv_len);
  encode_tlv_message(buff, type, data_length);
  encode_tlv_data(buff, data, data_length);

  return tlv_len; 
}

void static encode_tlv_header(uint8_t* tlv, size_t len)
{
  assert(tlv);
  static uint32_t tlvNumber;  
  tlv_header_structure* tlv_header_ptr = (tlv_header_structure*)tlv;

  tlv_header_ptr->magicWord[0] = MAGIC_BYTE_0_1;
  tlv_header_ptr->magicWord[1] = MAGIC_BYTE_2_3;
  tlv_header_ptr->magicWord[2] = MAGIC_BYTE_4_5;
  tlv_header_ptr->magicWord[3] = MAGIC_BYTE_6_7;

  tlv_header_ptr->totalPacketLen = len;
  tlv_header_ptr->tlvNumber      = tlvNumber++;
  tlv_header_ptr->timeCpuCycles  = k_cycle_get_32();
}

void static encode_tlv_message(uint8_t* tlv, tlv_message_type_e const type, const size_t data_length)
{
  assert(tlv);
  tlv += sizeof(tlv_header_structure);

  tlv_message* tlv_message_ptr   = (tlv_message*)tlv;
  tlv_message_ptr->type          = type;
  tlv_message_ptr->data_length   = data_length;
}

void static encode_tlv_data(uint8_t* tlv, const void* data, const size_t data_length)
{
  assert(tlv);
  tlv += sizeof(tlv_header_structure);
  tlv += sizeof(tlv_message);
  memcpy(tlv, data, data_length);
}
