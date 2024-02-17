#pragma once 

#include <stdbool.h>
#include "radar_tlv.h"

#define RADAR_HISTORY_CIRC_BUFFER_LEN (60)
#define TRACKING_IMPLEMENTATION
#define RADAR_CALIBRATED_MQ_PATH ("/mq_radar_calibrated")
#define RADAR_CHECK_NUMBER_OF_FRAMES_PERIOD_MS 3500
#define RADAR_MINIMUM_NUMBER_OF_RECENT_FRAMES 5

// see here for how to derive:
// https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/911459/iwr6843isk-ods-calculating-x-y-en-z-coordinates
typedef struct{
  float x;
  float y;
  float z;
  int16_t snr; 
  int16_t noise; 
} point_cartesian_t;

typedef struct {
  PointCloudMetaData meta_data;
  point_cartesian_t points[MAX_CLOUD_POINTS];
} cartesian_point_cloud_and_meta_t; 

typedef struct{
  int total_frames;
  int total_points;
} radar_history_t;

void            init_radar_thread(int);
int             get_radar_frame(char*, size_t);
bool            radar_received_sufficient_frames_recently(void);  
radar_history_t fetch_radar_history(void);


