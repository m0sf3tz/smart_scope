#pragma once 

// See page 9/198, rev 4 of the IMU data sheet for meaning 
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

void init_imu(void);
