#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>

#include "sensor_board_tlv.h"
#include "imu.h"

static mqd_t imu_mq;
static pthread_t imu_th;

static float roll_degrees;
static float pitch_degrees;
static float angular_rotation_degrees;
static float gyro_rotation_samples[TOTAL_SAMPLES_FOR_VARIANCE];
static float calibrated_rotation_offset;

static imu_t rolling_average_imu_sample;
static pthread_mutex_t imu_sample_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t calculate_imu_variance = PTHREAD_MUTEX_INITIALIZER;

static void* imu_thread(void*);

static void open_imu_mq(){
  imu_mq = open_mq(MESSAGE_QUEUE_NAME_IMU_DISPLAY, O_RDONLY | O_CREAT);
}

float calibrate_imu(){
#define COUNT_CALIBRATION (5)
  float gyro_history[COUNT_CALIBRATION];
 
  for(int i = 0; i < COUNT_CALIBRATION; i++){
    sleep(2);
    rotation_analysis_t rot = calculate_mean_rotation_and_variance();
    gyro_history[i] = rot.mean_rotation;
  }
  
  float rot = 0;
  for(int i = 0; i < COUNT_CALIBRATION; i++){
    rot += gyro_history[i];
  }

  calibrated_rotation_offset = rot/COUNT_CALIBRATION;
  printf("IMU calibrated with angular offset = %f\n", calibrated_rotation_offset);
}

rotation_analysis_t calculate_mean_rotation_and_variance(){
  float mean = 0;
  rotation_analysis_t rot_var;

  pthread_mutex_unlock(&calculate_imu_variance);

  // First, we need to find the mean  
  for(int i = 0; i < TOTAL_SAMPLES_FOR_VARIANCE; i++){
    mean += gyro_rotation_samples[i];
  }
  mean = mean/TOTAL_SAMPLES_FOR_VARIANCE;

  float sigma_square_mean_minus_val = 0;
  for(int i = 0; i < TOTAL_SAMPLES_FOR_VARIANCE; i++){
    sigma_square_mean_minus_val += (gyro_rotation_samples[i] - mean)*(gyro_rotation_samples[i] - mean);
  }

  pthread_mutex_unlock(&calculate_imu_variance);

  rot_var.mean_rotation     = mean - calibrated_rotation_offset;
  rot_var.variance_rotation = sigma_square_mean_minus_val/TOTAL_SAMPLES_FOR_VARIANCE;
  return rot_var;
}

static void imu_store_sample_for_variance_calculation(imu_t* sample){
  static size_t variance_index;

  // Variance calculations can take a bit of time, since we lock
  // calculate_imu_variance during that time, we will use
  // a trylock here and exit if we don't get the lock instead
  // of freezing this entire thread
  if(-1 == pthread_mutex_trylock(&calculate_imu_variance)){
    return;
  }
  gyro_rotation_samples[variance_index] = sample->r_y * DEGREES_IN_RAD;
  pthread_mutex_unlock(&calculate_imu_variance);
  
  if(variance_index == TOTAL_SAMPLES_FOR_VARIANCE - 1){
    variance_index = 0;
  } else {
    variance_index++;
  }
}

static int imu_calculate_rolling_average(imu_t* sample){
#define ROLLING_AVERAGE_SAMPLES (5)
  static imu_t samples[ROLLING_AVERAGE_SAMPLES];
  static size_t last_index = 0;
 
  samples[(last_index + 1) % ROLLING_AVERAGE_SAMPLES] = *sample;

  imu_t new_average = { 0 };
  for(int i = 0; i < ROLLING_AVERAGE_SAMPLES; i++) {
    new_average.a_x += samples[i].a_x;
    new_average.a_y += samples[i].a_y;
    new_average.a_z += samples[i].a_z;
    new_average.r_y += samples[i].r_y;
  }

  new_average.a_x = new_average.a_x / ROLLING_AVERAGE_SAMPLES;
  new_average.a_y = new_average.a_y / ROLLING_AVERAGE_SAMPLES;
  new_average.a_z = new_average.a_z / ROLLING_AVERAGE_SAMPLES;
  new_average.r_y = new_average.r_y / ROLLING_AVERAGE_SAMPLES;
  
  last_index = (last_index + 1) % ROLLING_AVERAGE_SAMPLES;
  
  pthread_mutex_lock(&imu_sample_mutex);
  rolling_average_imu_sample = new_average;

  roll_degrees             = DEGREES_IN_RAD*atan(new_average.a_x/new_average.a_z);
  pitch_degrees            = DEGREES_IN_RAD*atan(new_average.a_y/new_average.a_z);
  angular_rotation_degrees = DEGREES_IN_RAD*new_average.r_y;
  pthread_mutex_unlock(&imu_sample_mutex);
}

pitch_roll_rot_t imu_get_orientation(){
  pitch_roll_rot_t orientation;

  pthread_mutex_lock(&imu_sample_mutex);
  orientation.pitch    = pitch_degrees;
  orientation.roll     = roll_degrees;
  orientation.rotation = angular_rotation_degrees;
  pthread_mutex_unlock(&imu_sample_mutex);

  return orientation;
}

static int get_imu_sample(char* sample){
#define MAX_ACCELERATION 30 // Gs experienced in a car crash - reasonable limit
  assert(sample);
  imu_t* imu_ptr;

  int rc = mq_receive(imu_mq, sample, MESSAGE_QUEUE_SIZE, NULL);
  if(-1 == rc){
    printf("Failed to recieve from message queue, error: %s\n",strerror(errno));
    return -1;
  }
  
  imu_ptr = (imu_t*)(sample);
  if(imu_ptr->a_x > MAX_ACCELERATION || imu_ptr->a_y > MAX_ACCELERATION || imu_ptr->a_z > MAX_ACCELERATION){
    printf("Unexpectedly high acceleration...");
    return -1;
  }

  // Make clockwise spin positive
  imu_ptr->r_y *= -1;

  // Gets fed to display, always ongoing
  imu_calculate_rolling_average(imu_ptr);

  imu_store_sample_for_variance_calculation(imu_ptr);
  
  return 0;
}

void init_imu_thread(){
  open_imu_mq();
  
  int rc = pthread_create(&imu_th, NULL, imu_thread, NULL);
  if(rc != 0){
    printf("Failed to start imu_thread with error %s\n", strerror(rc));
    assert(0);
  } 
}

static void* imu_thread(void* arg){
#define IMU_PERIOD_DISPLAY (1250000) // 800Hz
  printf("IMU thread staring.\n");

  struct timespec sleep_frame_duration;
  sleep_frame_duration.tv_sec  = 0;
  sleep_frame_duration.tv_nsec = IMU_PERIOD_DISPLAY;
 
  char mq_buff[MESSAGE_QUEUE_SIZE]; 
  while(1){
    nanosleep(&sleep_frame_duration, NULL);
    get_imu_sample(mq_buff);
  }
}
