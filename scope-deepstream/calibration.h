#pragma once

#include <stdint.h>
#include <stdint.h>
#include "menu.h"

#define MAX_CALIBRATION_POINTS (20)
#define DONT_CARE (0)

// 10m, 15m, 20m, 25m, 30m etc
#define TOTAL_CALIBRATION_DISTANCE_POINTS (5)
#define CALIBRATION_DISTANCE_JUMP         (5)
#define CALIBRATION_DISTANCE_START        (10)
#define MAX_CALIBRATION_DISTANCE          (CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*(TOTAL_CALIBRATION_DISTANCE_POINTS-1))

// 1, 2 ... etc
#define TOTAL_CALIBRATION_VELOCITY_POINTS (4)  
#define CALIBRATION_VELOCITY_START        (0)
#define CALIBRATION_VELOCITY_JUMP         (1)
#define MAX_CALIBRATION_VELOCITY          (CALIBRATION_VELOCITY_JUMP*(TOTAL_CALIBRATION_VELOCITY_POINTS-1))

#define CALIBRATIONS_MAX_DISTANCE_PIXELS  (300)

// Distance, Offset
typedef struct{
  int16_t drop_calibration_points[MAX_CALIBRATION_POINTS];
  int16_t lead_calibration_points[MAX_CALIBRATION_POINTS][MAX_CALIBRATION_POINTS];
} __attribute__((packed)) calibration_data_t;

typedef struct{
  uint32_t crc32;
  calibration_data_t data;
} __attribute__((packed)) calibration_data_with_crc;

void save_calibration_data_with_crc(void);
int load_calibration_data_and_verify_crc(void);
void save_calibration_value(calibration_enum_e, uint16_t, uint16_t, int16_t);
int16_t fetch_calibration_value(calibration_enum_e, uint16_t, uint16_t);
