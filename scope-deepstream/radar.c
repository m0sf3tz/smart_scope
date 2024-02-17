#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <stdbool.h>

#include "sensor_board_tlv.h"
#include "radar_tlv.h"
#include "radar.h"
#include "mq.h"
#include "algo.h"

//#define DEBUG_PRINT

static mqd_t radar_mq;
static mqd_t radar_calibrated_mq;
static pthread_t radar_th;
static pthread_mutex_t history_mutex = PTHREAD_MUTEX_INITIALIZER;

static int radar_statitics_register_event(int);
static void* radar_thread(void*);
static int process_radar_frame(char*, FILE*);

static int radar_frames_received; 
static int radar_points_received; 
static int radar_history[RADAR_HISTORY_CIRC_BUFFER_LEN];

static double get_ms_since_start(){
  static struct timeval start_time;
  struct timeval time_now;
  
  if(start_time.tv_sec == 0){
    gettimeofday(&start_time, NULL);
  }
  
  gettimeofday(&time_now, NULL);
  long seconds        = time_now.tv_sec  - start_time.tv_sec;
  long micro_seconds  = time_now.tv_usec - start_time.tv_usec;
  
  return (seconds * 1000.0 + micro_seconds / 1000.0);
}

static void open_radar_mq(){
  radar_mq            = open_mq(RADAR_MQ_PATH, O_RDONLY | O_CREAT); // IN
  radar_calibrated_mq = open_mq(RADAR_CALIBRATED_MQ_PATH, O_RDWR | O_CREAT | O_NONBLOCK); // OUT (flips the image and does other calibration)
}

int get_radar_frame(char* frame_calibrated, size_t buff_len){
  assert(buff_len <= MESSAGE_QUEUE_SIZE);
  return mq_receive(radar_calibrated_mq, frame_calibrated, buff_len, NULL);
}

static int process_radar_frame(char* frame, FILE* dump_file){
  assert(frame);
  cartesian_point_cloud_and_meta_t cart_cloud;
  char buff[MESSAGE_QUEUE_SIZE];
  static int frame_num;

  PointCloudSpherical* point_cloud_ptr = (PointCloudSpherical*)(frame);
  //printf("New sample with %d frames\n", point_cloud_ptr->meta_data.points);

  //puts("Processing new frame.\n\n");
  double ms_since_start = get_ms_since_start();

  for(int i = 0; i < point_cloud_ptr->meta_data.points; i++){
#ifdef DEBUG_PRINT
    printf("range:%f\n",        point_cloud_ptr->points[i].sphere.range);
    printf("azimuthAngle:%f\n", point_cloud_ptr->points[i].sphere.azimuthAngle);
    printf("elevAngle:%f\n",    point_cloud_ptr->points[i].sphere.elevAngle);
#endif

    float R     = point_cloud_ptr->points[i].sphere.range;        // R
    float phi   = point_cloud_ptr->points[i].sphere.elevAngle;    // Θ
    float theta = point_cloud_ptr->points[i].sphere.azimuthAngle; // Φ

    // The sensor is installed upside down... hence the -1 factor.
    float X = R * cos(phi) * sin(theta) * -1;
    float Z = R * sin(phi) * -1; 
    float Y = R * cos(phi) * cos(theta);
    
    cart_cloud.points[i] = (point_cartesian_t){X,Y,Z, point_cloud_ptr->points[i].side.snr, point_cloud_ptr->points[i].side.noise};

    snprintf(buff, MESSAGE_QUEUE_SIZE, "%f, %d, %f, %f, %f\n", ms_since_start, frame_num, X, Z, Y);
    fwrite(buff, 1, strlen(buff), dump_file);
  }
  
  frame_num++;
  cart_cloud.meta_data = point_cloud_ptr->meta_data;
  size_t calibrated_size = sizeof(PointCloudMetaData) + (point_cloud_ptr->meta_data.points)*sizeof(point_cartesian_t);
  assert(calibrated_size < MESSAGE_QUEUE_SIZE);

  radar_statitics_register_event(point_cloud_ptr->meta_data.points);
  mq_send(radar_calibrated_mq, (char*)(&cart_cloud), calibrated_size, 0);
}

static int radar_statitics_register_event(int count){
  static size_t sample_index;
  int time_now = (int)get_ms_since_start();
  
  pthread_mutex_lock(&history_mutex);
  radar_history[sample_index] = time_now;
  radar_points_received += count;
  radar_frames_received += 1;
  pthread_mutex_unlock(&history_mutex);

  sample_index = (sample_index + 1) % RADAR_HISTORY_CIRC_BUFFER_LEN;
}

bool radar_received_sufficient_frames_recently(){
  int time_now = (int)get_ms_since_start();
  int time_cutoff = time_now - RADAR_CHECK_NUMBER_OF_FRAMES_PERIOD_MS;
  int recent_frames = 0;
 
  if(time_now < RADAR_CHECK_NUMBER_OF_FRAMES_PERIOD_MS){
    return false;
  }

  pthread_mutex_lock(&history_mutex);
  for(int i = 0; i < RADAR_HISTORY_CIRC_BUFFER_LEN; i++){ 
    if(radar_history[i] >= time_cutoff){
      recent_frames++;
    }
  }
  pthread_mutex_unlock(&history_mutex);

  if(recent_frames > RADAR_MINIMUM_NUMBER_OF_RECENT_FRAMES){
    return 1;
  } else {
    return 0;
  }
}

radar_history_t fetch_radar_history(){
  radar_history_t history;

  pthread_mutex_lock(&history_mutex);
  history.total_frames = radar_frames_received;
  history.total_points = radar_points_received;
  pthread_mutex_unlock(&history_mutex);
  
  return history;
}

static int get_radar_point_cloud(char* frame){
  assert(frame);

  int rc = mq_receive(radar_mq, frame, MESSAGE_QUEUE_SIZE, NULL);
  if(-1 == rc){
    printf("Failed to recieve from message queue, error: %s\n",strerror(errno));
    return -1;
  }
}

static int time_arg;
void init_radar_thread(int time){
  open_radar_mq();
  time_arg = time;
  
  int rc = pthread_create(&radar_th, NULL, radar_thread, &time_arg);
  if(rc != 0){
    printf("Failed to start radar_thread with error %s\n", strerror(rc));
    assert(0);
  } 
}

static void* radar_thread(void* arg){
  printf("Radar thread staring, dump log file_name = radar_%d.dump\n", *(int*)(arg));
  char buff[MESSAGE_QUEUE_SIZE];
  snprintf(buff, MESSAGE_QUEUE_SIZE, "radar_%d.dump", *(int*)(arg));
  FILE * radar_fp = fopen(buff, "w+");

  // Write the header
  snprintf(buff, MESSAGE_QUEUE_SIZE, "TIME(mS), frame, X,Z,Y\n");
  fwrite(buff, 1, strlen(buff), radar_fp);
  
  while(1){
    get_radar_point_cloud(buff);
    process_radar_frame(buff, radar_fp);
  }
}
