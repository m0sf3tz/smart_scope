#pragma once

#include <stdint.h>
typedef uint32_t ui_event;

// Note - these are shared w/ the sensor board
// don't update anything here without updating the 
// code running on the zephyr 

typedef enum {
  TLV_TYPE_IMU,
  TLV_TYPE_UI,
  TLV_TYPE_MAX
} tlv_message_type_e;

typedef enum { 
  ROTARY_BUTTON = 0,
  ROTARY_LEFT,
  ROTARY_RIGHT,
  BUTTON_0,
  BUTTON_1,
  TOTAL_EVENT_COUNT
} ui_event_e;

// See page 9/198 (rev 4) of the IMU data sheet for meaning 
typedef struct {
  // Accelerometer (m/s^2)
  float a_x;
  float a_y;
  float a_z;
  
  // Gyroscope, (rads/s)
  float r_p;
  float r_r;
  float r_y;

  // this value increases by 32768 each second
  uint32_t cpu_cycles_since_boot;
} imu_t;
