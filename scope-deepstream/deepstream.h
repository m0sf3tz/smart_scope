#pragma once
#include <stdint.h>
#include "gstnvdsmeta.h"

// Uncomment to print debug information on screen
#define DEBUG_PRINT_ON_SCREEN

#define PGIE_CLASS_ID_VEHICLE (0)
#define PGIE_CLASS_ID_PERSON  (2)

#define MAX_DISPLAY_LEN      (64)
#define SCREEN_WIDTH_PIXELS  (800)
#define SCREEN_HEIGHT_PIXELS (600)

/* Overlay stuff */
#define ORIENTATION_BAR_HEIGHT       (200)
#define ORIENTATION_BAR_WIDTH        (20)
#define ROLL_BAR_LEFT_OFFSET         (10)
#define PITCH_BAR_LEFT_OFFSET        (40)
#define CROSSHAIR_X_LEN_SEGMENT      (60)
#define CROSSHAIR_Y_LEN_SEGMENT      (90)
#define CROSSHAIR_SEGMENT_SEPERATION (30)
#define CROSSHAIR_WIDTH              (2)
#define CROSSHAIR_CENTER_RADIUS      (7)

// Colors
#define RED_COLOR  (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0}
#define WHITE_COLOR (NvOSD_ColorParams){1.0, 1.0, 1.0, 1.0}

typedef struct{
  bool has_screen;
  bool has_zoom;
  bool force_bb;
  bool calibrate_imu_on_boot;
} prog_config_t;

typedef struct{
  int text_num; 
  int circle_num;
  int line_num;
  int box_num;
} nv_ods_meta_shapes_counter_t;

// confusing, nvidia switches between "x" and "left" and "y" and "top"
typedef struct{
  int left;    // "x" 
  int top;     // "y"
  int width; 
  int height;
  int valid; 
} inference_detected_t;

typedef struct {
  int font_size;
  int x;
  int y;
  char str[MAX_DISPLAY_LEN];
} text_overlay_t;


// Used when drawing the hashes around a target
//   _          _
//  |            | (NE_CORNER)
//
//
//
//
//
//  |_          _| (SE_CORNER)
//  
typedef enum {NW_CORNER, NE_CORNER, SW_CORNER, SE_CORNER} corner_type_e;

int deepstream_init(uint32_t, prog_config_t);
void generic_func_display(NvDsFrameMeta *, NvDsDisplayMeta *, char*, nv_ods_meta_shapes_counter_t*);

