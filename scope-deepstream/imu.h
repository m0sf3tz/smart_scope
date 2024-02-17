#pragma once

#include "mq.h"

#define DEGREES_IN_RAD (57.2958)
#define MESSAGE_QUEUE_NAME_IMU_DISPLAY "/mq_imu_display"
#define TOTAL_SAMPLES_FOR_VARIANCE (550)

typedef struct{
  float pitch;
  float roll;
  float rotation;
} pitch_roll_rot_t;

typedef struct{
  float mean_rotation;
  float variance_rotation;
} rotation_analysis_t;

float calibrate_imu(void);
void init_imu_thread(void);
pitch_roll_rot_t imu_get_orientation(void);
rotation_analysis_t calculate_mean_rotation_and_variance(void);
