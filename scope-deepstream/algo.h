#pragma once 

#include "imu.h"
#include "deepstream.h"

// Un-comment the following to always go to the next state regardless of
// radar/imu/cv input
//#define DEBUG_ALWAYS_GO_TO_NEXT_STATE
#define ALWAYS_DRAW_TARGET

typedef enum {STATE_LOCK, STATE_TRACK, STATE_FIRE, STATE_FAIL} aim_sm_curr_state_e;
#define AIM_NO_FAIL              (0 << 0)
#define AIM_CV_FAIL_REASON       (1 << 0)
#define AIM_GYRO_VAR_FAIL_REASON (1 << 1)
#define AIM_RADAR_FAIL_REASON    (1 << 2)

// LOCK STATE
#define SAMPLES_TO_LOCK (20)
#define SAMPLING_PERIOD_FOR_LOCK_IN_MS (1500)
// TRACK STATE
#define SAMPLES_DURING_TRACK_MIN (20)
#define TRACK_DURATION_MS (1500)
#define MAX_VARIANCE_ROTATION (15.0)
// COOLDOWN STATE
#define COOLDOWN_DURATION_FAIL_MS (1000)
#define COOLDOWN_DURATION_FIRE_MS (5000)

typedef struct{
  int x;
  int y;
} cartesian_point_t; 

typedef struct{
  cartesian_point_t top_left_corner_of_progress_bar;
  int width;
  int height;
  int percentage_complete;
} progress_bar_t;

typedef struct{
  char overlay_str[MAX_DISPLAY_LEN];
  int overlay_x_offset;
  progress_bar_t prog; 
  float angular_velocity;
  float calculated_distance;
} overlay_info_t;

typedef struct{
  int                  aim_lock_recent_centered_frames;
  int                  aim_track_exit_time;
  size_t               aim_track_centered_frames;
  uint32_t             aim_fail_reason;
  int                  aim_fail_exit_time;
  int                  aim_fire_exit_time;
  float                aim_rotation;
  aim_sm_curr_state_e  state;
  rotation_analysis_t  gyro_result;
  cartesian_point_t    last_center;
  cartesian_point_t    last_aim_point;
  inference_detected_t last_inference;
  float                angular_velocity;
  double               target_distance;
} context_t;

typedef struct{
  cartesian_point_t aim_target;
  cartesian_point_t aim_target_corrected_for_bullet_lead_and_drop;
} aim_overlay_t;

typedef struct sm_t{
  struct sm_t (*next_state)(context_t *ctx);
} sm_t;

typedef sm_t (*state_fun)(context_t *ctx);

float          get_distance(void);
void           init_algo_thread(void);
void           get_overlay_text(char *const, int *);
bool           state_request_bounding_hashes(void);
bool           state_request_show_angular_velocity(void);
bool           state_request_progress_bar(void);
int            calculate_if_bounding_box_centered(inference_detected_t);
progress_bar_t get_lock_progress_bar(void);
float          get_angular_trained_angular_velocity(void);
float          get_angular_trained_distance(void);
bool           state_request_corrected_crosshair(void);
bool           state_request_simple_crosshair(void);
