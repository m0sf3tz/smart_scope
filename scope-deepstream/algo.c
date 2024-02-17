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

#include "gstnvdsmeta.h"
#include "sensor_board_tlv.h"
#include "radar_tlv.h"
#include "radar.h"
#include "mq.h"
#include "algo.h"
#include "deepstream.h"
#include "time.h"
#include "imu.h"
#include "interpolate.h"

static mqd_t radar_calibrated_mq;
static mqd_t inference_output_mq; 
static mqd_t crosshair_input_mq; 

static pthread_t aiming_th, distance_th;
static pthread_mutex_t distance_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t algo_mutex = PTHREAD_MUTEX_INITIALIZER;

static void *distance_thread(void*);
static void *aiming_thread(void*);
static void find_centeroid(char*);
static void filter_outliers(char*, char*, point_cartesian_t*, int*);
static aim_sm_curr_state_e get_state(void);
static void draw_crosshair(context_t*);

static sm_t aim_sm_transition(context_t*);
static sm_t aim_sm_lock(context_t*);

static int check_if_inference_is_centered(context_t*);
static void set_ctx_entery_track_state(context_t*);

static float calculated_distance;
static aim_sm_curr_state_e curr_state;
static overlay_info_t overlay_info;

float get_distance(){
  float distance;

  pthread_mutex_lock(&distance_mutex);
  distance = calculated_distance;
  pthread_mutex_unlock(&distance_mutex);

  return distance;
}

float get_angular_trained_angular_velocity(){
  float ret;  
  pthread_mutex_lock(&algo_mutex);
  ret = overlay_info.angular_velocity;
  pthread_mutex_unlock(&algo_mutex);  
  return ret;
}

float get_angular_trained_distance(){
  float ret;  
  pthread_mutex_lock(&algo_mutex);
  ret = overlay_info.calculated_distance;
  pthread_mutex_unlock(&algo_mutex);
  
  return ret;
}

bool state_request_progress_bar(){
  aim_sm_curr_state_e state = get_state(); 
  if(state == STATE_LOCK){
    return true;
  } else {
    return false;
  }
}

bool state_request_corrected_crosshair(){
  aim_sm_curr_state_e state = get_state(); 
  if(state == STATE_FIRE){
    return true;
  } else {
    return false;
  }
}

progress_bar_t get_lock_progress_bar(){
  progress_bar_t prog; 
  pthread_mutex_lock(&algo_mutex);
  prog = overlay_info.prog;
  pthread_mutex_unlock(&algo_mutex);
  return prog;
}

bool state_request_bounding_hashes(){
#ifdef ALWAYS_DRAW_TARGET
  return true;
#endif

  aim_sm_curr_state_e state = get_state(); 
  if(state == STATE_LOCK || state == STATE_TRACK){
    return true;
  } else {
    return false;
  }
}

bool state_request_show_angular_velocity(){
  aim_sm_curr_state_e state = get_state(); 
  if(state == STATE_FIRE){
    return true;
  } else {
    return false;
  }
}

bool state_request_simple_crosshair(){
  return state_request_corrected_crosshair();
}

void get_overlay_text(char *const dst, int *x_offset){
  pthread_mutex_lock(&algo_mutex);
  strncpy(dst, overlay_info.overlay_str, MAX_DISPLAY_LEN);
  *x_offset = overlay_info.overlay_x_offset;
  pthread_mutex_unlock(&algo_mutex);
}

static void open_radar_mq(){
  radar_calibrated_mq = open_mq(RADAR_CALIBRATED_MQ_PATH, O_RDONLY | O_CREAT | O_NONBLOCK);
  inference_output_mq = open_mq(MESSAGE_QUEUE_OUTPUT_INF, O_RDONLY | O_CREAT | O_NONBLOCK);
  crosshair_input_mq  = open_mq(MESSAGE_QUEUE_CROSS,      O_WRONLY | O_CREAT | O_NONBLOCK);  
}

static aim_sm_curr_state_e get_state(){
  aim_sm_curr_state_e state;  
  pthread_mutex_lock(&algo_mutex);
  state = curr_state;
  pthread_mutex_unlock(&algo_mutex);
  return state;
}

static void set_state(aim_sm_curr_state_e state){
  pthread_mutex_lock(&algo_mutex);
  curr_state = state;
  pthread_mutex_unlock(&algo_mutex);
}

static void set_angular_velocity_plus_distance(const float angular_velocity, const float distance){
  pthread_mutex_lock(&algo_mutex);
  overlay_info.angular_velocity  = angular_velocity;
  overlay_info.calculated_distance = distance;
  pthread_mutex_unlock(&algo_mutex);
}

static void set_overlay_info(const char *str, int x_offset, progress_bar_t prog){
  pthread_mutex_lock(&algo_mutex);
  strncpy(overlay_info.overlay_str, str, MAX_DISPLAY_LEN);
  overlay_info.overlay_x_offset = x_offset;
  overlay_info.prog = prog;
  pthread_mutex_unlock(&algo_mutex); 
}

void init_algo_thread(){
  open_radar_mq();
  
  int rc = pthread_create(&distance_th, NULL, distance_thread, NULL);
  if(rc != 0){
    printf("Failed to start distance thread with error %s\n", strerror(rc));
    assert(0);
  }

  rc = pthread_create(&aiming_th, NULL, aiming_thread, NULL);
  if(rc != 0){
    printf("Failed to start aiming thread with error %s\n", strerror(rc));
    assert(0);
  }
}

/* filters a point cloud in two ways,
    A) If no mean is provided, filters any point on the x/z axis that is 
       too far off the center boreline
    B) If a mean is provided, every point in the point cloud is compared against this mean,
       if it's too far away from it, it is discarded. We can then do another pass to calculate a new
       mean after we have discarded outliers.
*/ 
static void filter_outliers(char* input_points, char* output_points, point_cartesian_t *mean, int* new_points){
#define REJECTION_POINT_OFFSET_ROUGH  (100)
#define REJECTION_POINT_OFFSET_FINE   (5)
#define MINIMUM_VIABLE_DISTANCE_RADAR (6.0)
  cartesian_point_cloud_and_meta_t* cart_cloud_ptr_input  = (cartesian_point_cloud_and_meta_t*)(input_points);
  cartesian_point_cloud_and_meta_t* cart_cloud_ptr_output = (cartesian_point_cloud_and_meta_t*)(output_points);
  int new_point_count = 0;

  if(mean == NULL) {
    for(int i = 0; i < cart_cloud_ptr_input->meta_data.points; i++){
      if(cart_cloud_ptr_input->points[i].x < REJECTION_POINT_OFFSET_ROUGH &&
         cart_cloud_ptr_input->points[i].z < REJECTION_POINT_OFFSET_ROUGH
          ) {

        // The inside of the case is made of metal which causes reflections.
        // Ignore anything less than MINIMUM_VIABLE_DISTANCE_RADAR meters away.
        if(cart_cloud_ptr_input->points[i].y > MINIMUM_VIABLE_DISTANCE_RADAR){
          cart_cloud_ptr_output->points[new_point_count] = cart_cloud_ptr_input->points[i];
          new_point_count++;
        }
      }
    }
  } else {
    float dx, dy, dz, distance;
    for(int i = 0; i < cart_cloud_ptr_input->meta_data.points; i++){
      dx = cart_cloud_ptr_input->points[i].x - mean->x;
      dy = cart_cloud_ptr_input->points[i].y - mean->y;
      dz = cart_cloud_ptr_input->points[i].z - mean->z;
      
      distance = sqrt(dx*dx + dy*dy + dz*dz);
      if(distance < REJECTION_POINT_OFFSET_FINE) {
         cart_cloud_ptr_output->points[new_point_count] = cart_cloud_ptr_input->points[i];
         new_point_count++;
      }  
    }
  }

  cart_cloud_ptr_output->meta_data = cart_cloud_ptr_input->meta_data;
  cart_cloud_ptr_output->meta_data.points = new_point_count;
  *new_points = new_point_count;
  //printf("Old count = %d, new count %d\n", cart_cloud_ptr_input->meta_data.points, new_point_count);
}

static point_cartesian_t find_euclidean_mean(char *input_points){
  cartesian_point_cloud_and_meta_t* cart_cloud_ptr_input  = (cartesian_point_cloud_and_meta_t*)(input_points);
  double x_mean = 0;
  double y_mean = 0;
  double z_mean = 0;
  size_t count = cart_cloud_ptr_input->meta_data.points;

  for(int i = 0; i < count; i++){
    x_mean += cart_cloud_ptr_input->points[i].x;
    y_mean += cart_cloud_ptr_input->points[i].y;
    z_mean += cart_cloud_ptr_input->points[i].z;
  }
 
  point_cartesian_t mean;
  mean.x = x_mean/count;
  mean.y = y_mean/count;
  mean.z = z_mean/count;
  
  //printf("x_mean %f, y_mean %f, z_mean %f\n", mean.x, mean.y, mean.z);
  return mean; 
}

static void find_centeroid(char *input_points){
#define ROLLING_AVERAGE_SAMPLES (5)
  static float samples[ROLLING_AVERAGE_SAMPLES];
  static size_t last_index = 0;
  int new_points; 
 
  assert(input_points);
  char filtered_points[MESSAGE_QUEUE_SIZE];
  point_cartesian_t mean;

  // step A) rough filtering, only accepts points that are close to the x/z axis, does not care about groupings
  filter_outliers(input_points, filtered_points, NULL, &new_points);
  if(0 == new_points) { return; }

  // step B) find the center based on the above filtering
  mean = find_euclidean_mean(filtered_points);

  samples[(last_index + 1) % ROLLING_AVERAGE_SAMPLES] = mean.y;
  float new_average = 0;
  for(int i = 0; i < ROLLING_AVERAGE_SAMPLES; i++) {
    new_average += samples[i];
  }
  new_average = new_average / ROLLING_AVERAGE_SAMPLES;
  last_index = (last_index + 1) % ROLLING_AVERAGE_SAMPLES;
  
  pthread_mutex_lock(&distance_mutex);
  calculated_distance = new_average;
  pthread_mutex_unlock(&distance_mutex);
}

static void *distance_thread(void* arg){
#define FRAME_PERIOD_RADAR (33333333)
  printf("Distance thread starting\n");
  
  char buff[MESSAGE_QUEUE_SIZE];
  struct timespec sleep_frame_duration;
  sleep_frame_duration.tv_sec = 0;
  sleep_frame_duration.tv_nsec = FRAME_PERIOD_RADAR;

  while(1){
    // run every 33ms or so - this is how often we get a new frame from
    // the radar.
    nanosleep(&sleep_frame_duration, NULL);

    if (0 < get_radar_frame(buff, MESSAGE_QUEUE_SIZE)) {
      find_centeroid(buff);
    }
  }
}

cartesian_point_t calculate_bounding_box_center(inference_detected_t box){
  cartesian_point_t center;

  center.x = box.left + box.width/2;
  center.y = box.top  + box.height/2;
  
  return center;
}

// Will aim for the "chest" area
static cartesian_point_t calculate_optimial_aim_location(inference_detected_t box){
  cartesian_point_t center;

  center.x = box.left + box.width/2;
  center.y = box.top  + box.height*(1/4);
  
  return center;
}

// returns 1 if bounding box is centered, 0 otherwise
int calculate_if_bounding_box_centered(inference_detected_t box){
#define DISTANCE_TO_CENTER_MAX (150)
  cartesian_point_t center = calculate_bounding_box_center(box);

  int dx = center.x - SCREEN_WIDTH_PIXELS/2;
  int dy = center.y - SCREEN_HEIGHT_PIXELS/2;

  int distance_to_center = sqrt(dx*dx + dy*dy);
 
  if(distance_to_center < DISTANCE_TO_CENTER_MAX){
    return 1;
  } else {
    return 0;
  }
}

static sm_t aim_sm_fire(context_t *ctx){
  int time_now = (int)get_ms_since_start();
  ctx->state = STATE_FIRE; 
  
  draw_crosshair(ctx); 
 
  if(time_now > ctx->aim_fire_exit_time){
    return (sm_t) {aim_sm_lock};
  }
  
  return (sm_t) {aim_sm_fire};
}

static sm_t aim_sm_failed(context_t *ctx){
  int time_now = (int)get_ms_since_start();
  ctx->state = STATE_FAIL; 
  
  if(time_now > ctx->aim_fail_exit_time){
    return (sm_t) {aim_sm_lock};
  }
  
  return (sm_t) {aim_sm_failed};
}

static void set_ctx_entery_fail_state(context_t *ctx){
  ctx->aim_fail_exit_time = (int)get_ms_since_start() + COOLDOWN_DURATION_FAIL_MS;
}

static void set_ctx_entery_fire_state(context_t *ctx){
  double distance = get_distance();

  ctx->aim_fire_exit_time = (int)get_ms_since_start() + COOLDOWN_DURATION_FIRE_MS;
  ctx->angular_velocity   = ctx->gyro_result.mean_rotation/DEGREES_IN_RAD * distance; // mean_rotation is in degrees
  ctx->target_distance    = distance;
#ifdef DEBUG_ALWAYS_GO_TO_NEXT_STATE 
  // We might not actually have an inference, still need a sane aimpoint
  ctx->last_aim_point     = (cartesian_point_t){SCREEN_WIDTH_PIXELS/2, SCREEN_HEIGHT_PIXELS/2};
#else 
  ctx->last_aim_point     = calculate_optimial_aim_location(ctx->last_inference);
#endif 

  set_angular_velocity_plus_distance(ctx->angular_velocity, distance);
}

static sm_t aim_sm_track(context_t *ctx){
  int time_now = (int)get_ms_since_start();
  ctx->state = STATE_TRACK; 
  
  if(check_if_inference_is_centered(ctx)){
    ctx->aim_track_centered_frames++;
  }

  if(time_now > ctx->aim_track_exit_time){
    ctx->aim_fail_reason = 0;
    // Did deepstream capture enough frames?
    if(ctx->aim_track_centered_frames < SAMPLES_DURING_TRACK_MIN) {
      ctx->aim_fail_reason |= AIM_CV_FAIL_REASON;
    }
    
    // Was the gyro variance reasonable during the tracking phase?
    rotation_analysis_t gyro_result = calculate_mean_rotation_and_variance();
    ctx->gyro_result = gyro_result;
    if(gyro_result.variance_rotation > MAX_VARIANCE_ROTATION){
      ctx->aim_fail_reason |= AIM_GYRO_VAR_FAIL_REASON;
    }

    // Check for number of recent radar frames
    if(!radar_received_sufficient_frames_recently()){
      ctx->aim_fail_reason |= AIM_RADAR_FAIL_REASON;
    }

#ifdef DEBUG_ALWAYS_GO_TO_NEXT_STATE
    ctx->aim_fail_reason = AIM_NO_FAIL; 
#endif 

    if(ctx->aim_fail_reason){
      set_ctx_entery_fail_state(ctx);
      return (sm_t) {aim_sm_failed};
    } else {
      set_ctx_entery_fire_state(ctx);
      return (sm_t) {aim_sm_fire};
    }
  }

  return (sm_t) {aim_sm_track};
}

static sm_t aim_sm_lock(context_t *ctx){
#define HISTORY_BUFF_SIZE (SAMPLES_TO_LOCK*2)
  static size_t sample_index;
  static int lock_history[HISTORY_BUFF_SIZE];

  int time_now = (int)get_ms_since_start();
  int time_cutoff = time_now - SAMPLING_PERIOD_FOR_LOCK_IN_MS;
  ctx->state = STATE_LOCK; 

  if(check_if_inference_is_centered(NULL)){
    lock_history[sample_index] = time_now;
    sample_index = (sample_index + 1) % HISTORY_BUFF_SIZE; 
  }

  // If within the past 1.5s we had at least 30 frames 
  // centered, we assume we have lock and go to the next
  // state.
  int recent_frames_in_lock = 0;
  for(int i = 0; i < HISTORY_BUFF_SIZE; i++){
    if(lock_history[i] >= time_cutoff){
      recent_frames_in_lock++;   
    }
  }

  ctx->aim_lock_recent_centered_frames = recent_frames_in_lock;
 
#ifdef DEBUG_ALWAYS_GO_TO_NEXT_STATE
  if(1) {
#else
  if(recent_frames_in_lock > SAMPLES_TO_LOCK) {
#endif
    set_ctx_entery_track_state(ctx);
    return (sm_t) {aim_sm_track};
  }
  
  return (sm_t) {aim_sm_lock};
}

static cartesian_point_t calculate_bullet_drop_and_lead(context_t *ctx){
  cartesian_point_t target;
 
  target.x = ctx->last_aim_point.x + (int)(get_interpolation_lead(ctx->target_distance, ctx->angular_velocity));
  target.y = ctx->last_aim_point.y - (int)(get_interpolation_distance(ctx->target_distance));

  return target;
}

static void draw_crosshair(context_t *ctx){
  char mq_buff[MESSAGE_QUEUE_SIZE];
  inference_detected_t *inference;
  aim_overlay_t aim_overlay;

#ifdef DEBUG_ALWAYS_GO_TO_NEXT_STATE
  // Move the corrected aimpoint around on the screen
  static int skip;
  if(skip == 10){
    skip = 0;
    ctx->last_aim_point.x += 1;
    ctx->last_aim_point.y += 1;
  } else {
    skip++;
  }
  if(ctx->last_aim_point.x >= SCREEN_WIDTH_PIXELS)  ctx->last_aim_point.x = 0;
  if(ctx->last_aim_point.y >= SCREEN_HEIGHT_PIXELS) ctx->last_aim_point.y = 0;
  goto EXIT;
#endif
 
  int rc = mq_receive(inference_output_mq, mq_buff, MESSAGE_QUEUE_SIZE, NULL);
  if(-1 == rc){
    goto EXIT;
  }

  // Update target - new inference available 
  inference = (inference_detected_t*)(mq_buff);
  ctx->last_aim_point = calculate_optimial_aim_location(*inference);

EXIT:
  aim_overlay.aim_target                                    = ctx->last_aim_point;
  aim_overlay.aim_target_corrected_for_bullet_lead_and_drop = calculate_bullet_drop_and_lead(ctx);
  mq_send(crosshair_input_mq, (char*)&aim_overlay, sizeof(aim_overlay), 0);
}

// returns 1 if the crosshair is on top of a target.
static int check_if_inference_is_centered(context_t* ctx){
  char mq_buff[MESSAGE_QUEUE_SIZE];
  inference_detected_t *inference;
  
  int rc = mq_receive(inference_output_mq, mq_buff, MESSAGE_QUEUE_SIZE, NULL);
  if(-1 == rc){
    return 0;
  }

  inference = (inference_detected_t*)(mq_buff);
  if(calculate_if_bounding_box_centered(*inference)){
    if(ctx){
      ctx->last_center    = calculate_bounding_box_center(*inference);
      ctx->last_inference = *inference;
    }
    return 1;
  }
  return 0;
}

static void set_ctx_entery_track_state(context_t *ctx){
  ctx->aim_track_exit_time = (int)get_ms_since_start() + TRACK_DURATION_MS;
  ctx->aim_track_centered_frames = 0;
}

static void update_crosshair_overlay_based_on_state(context_t *ctx){
#define PROGRESS_BAR_WIDTH (100)
#define PROGRESS_BAR_HEIGHT (20)
  char overlay_str[MAX_DISPLAY_LEN];
  int time_now = (int)get_ms_since_start();
  int x_offset;
  progress_bar_t prog;

  if(ctx->state == STATE_LOCK){
    int percentage_complete = (ctx->aim_lock_recent_centered_frames*1.0/SAMPLES_TO_LOCK) * 100;
    if(percentage_complete > 100){
      percentage_complete = 100;
    }
   
    prog.width  = PROGRESS_BAR_WIDTH; 
    prog.height = PROGRESS_BAR_HEIGHT;
    prog.percentage_complete = percentage_complete;
    prog.top_left_corner_of_progress_bar.x = 420; // right
    prog.top_left_corner_of_progress_bar.y = 271; // up
    snprintf(overlay_str, MAX_DISPLAY_LEN, "SEEKING");
    x_offset = 265;
  } else if(ctx->state == STATE_TRACK) {
    int time_left_in_state = ctx->aim_track_exit_time - time_now;
    if(time_left_in_state < 0){
      time_left_in_state = 0;
    }
    int percentage_complete = (time_left_in_state*1.0/TRACK_DURATION_MS) * 100;
    snprintf(overlay_str, MAX_DISPLAY_LEN, "TRACKING");
    x_offset = 330;
  } else if(ctx->state == STATE_FAIL) {
    snprintf(overlay_str, MAX_DISPLAY_LEN, "FAILED [");
    x_offset = 330; 
    if(ctx->aim_fail_reason & AIM_CV_FAIL_REASON) {
      strcat(overlay_str, "CV");
      x_offset -= 20; 
    }  
    if(ctx->aim_fail_reason & AIM_GYRO_VAR_FAIL_REASON) {
      strcat(overlay_str, " GY");
      x_offset -= 20; 
    }
    if(ctx->aim_fail_reason & AIM_RADAR_FAIL_REASON) {
      strcat(overlay_str, " RD");
      x_offset -= 20; 
    }
    strcat(overlay_str, "]");
  } else {
    snprintf(overlay_str, MAX_DISPLAY_LEN, " ");
    x_offset = 341;
  }
 
  set_overlay_info(overlay_str, x_offset, prog);
}

static void *aiming_thread(void *arg){
#define FRAME_PERIOD_CAMERA (3333333) // 300fps 
  /* We run the state machine about 300 times a second */

  printf("Aiming thread starting\n");
  struct timespec sleep_frame_duration;
  sleep_frame_duration.tv_sec = 0;
  sleep_frame_duration.tv_nsec = FRAME_PERIOD_CAMERA;
  context_t ctx;

  state_fun state_fun_ptr = aim_sm_lock;
  while(1){
    nanosleep(&sleep_frame_duration, NULL);
    state_fun_ptr = state_fun_ptr(&ctx).next_state;

    set_state(ctx.state);
    update_crosshair_overlay_based_on_state(&ctx);
  }
}
