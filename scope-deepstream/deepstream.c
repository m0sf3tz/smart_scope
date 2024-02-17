/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <cuda_runtime_api.h>
#include <assert.h>
#include <mqueue.h>
#include "gstnvdsmeta.h"

#include "time.h"
#include "deepstream.h"
#include "imu.h"
#include "radar.h"
#include "menu.h"
#include "algo.h"

static void draw_text_overlay(NvDsFrameMeta*, NvDsDisplayMeta*, nv_ods_meta_shapes_counter_t*);
static void draw_text_api(NvDsFrameMeta*, NvDsDisplayMeta*, nv_ods_meta_shapes_counter_t*, text_overlay_t*, int);
static void draw_tilt(NvDsFrameMeta*, NvDsDisplayMeta*, nv_ods_meta_shapes_counter_t*);
static void draw_crosshair(NvDsFrameMeta*, NvDsDisplayMeta*, nv_ods_meta_shapes_counter_t*);
static void draw_target(NvDsFrameMeta *, NvDsDisplayMeta*, nv_ods_meta_shapes_counter_t*);
static void draw_bounding_hashes(NvDsFrameMeta *, NvDsDisplayMeta *, nv_ods_meta_shapes_counter_t*, inference_detected_t);
static void draw_progress_bar(NvDsFrameMeta *, NvDsDisplayMeta *, nv_ods_meta_shapes_counter_t*, progress_bar_t);

static void validate_nvosd_counter(nv_ods_meta_shapes_counter_t*);
static bool validate_point(cartesian_point_t);

static int  draw_line(NvDsFrameMeta *, NvDsDisplayMeta *, nv_ods_meta_shapes_counter_t*, int, NvOSD_ColorParams, cartesian_point_t, cartesian_point_t);
static void draw_bounding_box_corner(NvDsFrameMeta *, NvDsDisplayMeta *, nv_ods_meta_shapes_counter_t*, corner_type_e, cartesian_point_t);
static void extract_inference(NvDsMetaList*, NvDsFrameMeta*, inference_detected_t*);

static mqd_t inference_output_mq; 
static mqd_t crosshair_input_mq;

static int bounding_box_enable_overide;
 
/* Extracts meta-data from the inference, also adds text/overlays over the image */
static GstPadProbeReturn
osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
    gpointer u_data) 
{
    GstBuffer *buf = (GstBuffer *) info->data;
    NvDsMetaList * l_frame = NULL;
    NvDsDisplayMeta *display_meta = NULL;
    nv_ods_meta_shapes_counter_t counter = {0}; 
    NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);
    display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
    inference_detected_t bounding_box = {0};

    /* Iterate through the frames in this batch */
    for (l_frame = batch_meta->frame_meta_list; l_frame != NULL; l_frame = l_frame->next) {
        NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
        
        /* This is filled out by the inference model, we extract persons location*/
        extract_inference(l_frame, frame_meta, &bounding_box);

        if(draw_menu(frame_meta, display_meta)) {
          /* Won't draw anything else, only the menu */
          goto RETURN;
        }
        /* Draw on the screen */
        draw_tilt(frame_meta, display_meta, &counter);
        draw_crosshair(frame_meta, display_meta, &counter);
        draw_text_overlay(frame_meta, display_meta, &counter);

        if(state_request_corrected_crosshair()){
          draw_target(frame_meta, display_meta, &counter);
        }
        
        if(state_request_progress_bar()){
          progress_bar_t prog = get_lock_progress_bar();
          draw_progress_bar(frame_meta, display_meta, &counter, prog);
        } 
       
        // Only draw bounding box if we are in certain states AND the target is near the crosshair 
        if(state_request_bounding_hashes() && calculate_if_bounding_box_centered(bounding_box)) {
          draw_bounding_hashes(frame_meta, display_meta, &counter, bounding_box);
        }

        validate_nvosd_counter(&counter);
   } 

RETURN:
    return GST_PAD_PROBE_OK;
}

/* Extracts where deepstream thinks people are */
static void extract_inference(NvDsMetaList * l_frame, NvDsFrameMeta *frame_meta, inference_detected_t *bounding_box_ptr)
{
  NvDsMetaList* l_obj = NULL;
  NvDsObjectMeta* obj_meta = NULL;
  char mq_buff[MESSAGE_QUEUE_SIZE];
  bounding_box_ptr->valid = false;

  for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
      NvDsObjectMeta* obj_meta = (NvDsObjectMeta *) (l_obj->data);
      if (obj_meta->class_id == PGIE_CLASS_ID_PERSON) {
        inference_detected_t bounding_box;
        NvOSD_RectParams params = obj_meta->rect_params;

        bounding_box.left   = (int)params.left;
        bounding_box.top    = (int)params.top;
        bounding_box.width  = (int)params.width;
        bounding_box.height = (int)params.height;
        
        // Copy the bounding box info, this is later used to draw bounding "hashes" 
        // around a target
        bounding_box_ptr->valid = true;
        memcpy(bounding_box_ptr, &bounding_box, sizeof(bounding_box));       

        printf("Sending sample @ time %f\n", get_ms_since_start()); 
        int rc = mq_send(inference_output_mq, (char*)&bounding_box, sizeof(inference_detected_t), 0);
        if(rc) {
          puts("Failed to send out of inference pipeline");
        }
      }
  }

  // A bug in this version of deepstream does not seem to support 
  // disabling the bounding boxes... as a workaround we will push it 
  // outside the screen...
  if(false == is_bounding_box_enabled() && !bounding_box_enable_overide){
    for (l_obj = frame_meta->obj_meta_list; l_obj != NULL; l_obj = l_obj->next) {
        NvDsObjectMeta* obj_meta = (NvDsObjectMeta *) (l_obj->data);
        if (obj_meta->class_id == PGIE_CLASS_ID_PERSON) {
          NvOSD_RectParams params = obj_meta->rect_params;
          obj_meta->rect_params.left = SCREEN_WIDTH_PIXELS + 1;
          obj_meta->rect_params.top  = SCREEN_HEIGHT_PIXELS + 1;
        }
        obj_meta->text_params.x_offset = SCREEN_WIDTH_PIXELS;
        obj_meta->text_params.y_offset = SCREEN_HEIGHT_PIXELS;
    }
  }
}

static void validate_nvosd_counter(nv_ods_meta_shapes_counter_t* counter){
  assert(counter->text_num   < MAX_ELEMENTS_IN_DISPLAY_META);
  assert(counter->circle_num < MAX_ELEMENTS_IN_DISPLAY_META);
  assert(counter->line_num   < MAX_ELEMENTS_IN_DISPLAY_META);
  assert(counter->box_num    < MAX_ELEMENTS_IN_DISPLAY_META);
}

static void draw_text_api(NvDsFrameMeta *frame_meta,
  NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter, text_overlay_t * text, int count)
{
  NvOSD_TextParams *txt_params = &display_meta->text_params[0];

  for(int i = 0; i < count; i++) {
    if (validate_point((cartesian_point_t){text->x, text->y})){
      continue;
    }

    txt_params[counter->text_num].display_text = g_malloc0 (MAX_DISPLAY_LEN);
    snprintf(txt_params[counter->text_num].display_text, MAX_DISPLAY_LEN, "%s", text->str);
    
    txt_params[counter->text_num].x_offset              = text->x;
    txt_params[counter->text_num].y_offset              = text->y;
    txt_params[counter->text_num].font_params.font_size = text->font_size;
    
    txt_params[counter->text_num].font_params.font_name        = "Consolas";
    txt_params[counter->text_num].font_params.font_color.red   = 1.0;
    txt_params[counter->text_num].font_params.font_color.green = 0.0;
    txt_params[counter->text_num].font_params.font_color.blue  = 0.0;
    txt_params[counter->text_num].font_params.font_color.alpha = 1.0;
    counter->text_num++;
    text++;
  }

  display_meta->num_labels = counter->text_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
}

static void draw_text_overlay(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter)
{
  char debug_text[DISPLAY_BUFF_LEN] = {0};
  char string[DISPLAY_BUFF_LEN];

  // Optional debug info, enabled from menu
  if(is_debug_info_enabled()){
    radar_history_t history = fetch_radar_history();

    snprintf(string, MAX_DISPLAY_LEN, "Radar:\n  Distance: %.1f\n", get_distance());
    strcat(debug_text, string); 

    snprintf(string, MAX_DISPLAY_LEN, "  Frames: %d\n", history.total_frames);
    strcat(debug_text, string); 

    snprintf(string, MAX_DISPLAY_LEN, "  Points: %d\n", history.total_points);
    strcat(debug_text, string); 

    pitch_roll_rot_t imu_sample = imu_get_orientation();
    snprintf(string, MAX_DISPLAY_LEN, "º/s: %.2f\n", imu_sample.rotation);
    strcat(debug_text, string); 

    snprintf(string, MAX_DISPLAY_LEN, "T: %.4f", get_ms_since_start());
    strcat(debug_text, string); 
    
    if(state_request_show_angular_velocity()){
      snprintf(string, MAX_DISPLAY_LEN, "\n[TRAINED: ω: %.2f, d: %.2f]", get_angular_trained_angular_velocity(), get_angular_trained_distance());
      strcat(debug_text, string); 
    }
    generic_func_display(frame_meta, display_meta, debug_text, counter);
  }

  // This is the text over the crosshair (information about locking, etc) 
  text_overlay_t txt;
  txt.font_size = 20;
  txt.y = 260; 
  get_overlay_text(txt.str, &txt.x);
  draw_text_api(frame_meta, display_meta, counter, &txt, 1);
}

void generic_func_display(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, char * txt, nv_ods_meta_shapes_counter_t* counter){
  assert(txt);

  NvOSD_TextParams *txt_params = &display_meta->text_params[0];
  int text_num = 0;
  txt_params[text_num].display_text = g_malloc0 (DISPLAY_BUFF_LEN);
  snprintf(txt_params[text_num].display_text, DISPLAY_BUFF_LEN, "%s", txt);

  txt_params[text_num].font_params.font_name        = "Courier";
  txt_params[text_num].font_params.font_size        = 15;
  txt_params[text_num].font_params.font_color.red   = 1.0;
  txt_params[text_num].font_params.font_color.green = 0.0;
  txt_params[text_num].font_params.font_color.blue  = 0.0;
  txt_params[text_num].font_params.font_color.alpha = 1.0;
  
  txt_params[text_num].set_bg_clr = 1;
  txt_params[text_num].text_bg_clr.red = 0.0;
  txt_params[text_num].text_bg_clr.green = 0.0;
  txt_params[text_num].text_bg_clr.blue = 0.0;
  txt_params[text_num].text_bg_clr.alpha = 1.0;

  txt_params[text_num].x_offset = (10);
  txt_params[text_num].y_offset = (10);
  text_num++;

  // This is ugly - two threads will call this function but
  // it was shoehorned last minute. the deepstream thread might 
  // add multiple text overlays, so it needs to keep count, so it 
  // will pass valid counter pointer. The menu thread will only
  // ever add a single text item so it does not really care about 
  // the counter so it will pass NULL
  if(counter){
    counter->text_num++;
  }
  display_meta->num_labels = text_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
}

static void draw_progress_bar(NvDsFrameMeta * frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter, progress_bar_t prog){
  NvOSD_RectParams *rect_params = &display_meta->rect_params[0];

  assert(prog.percentage_complete >= 0);
  assert(prog.percentage_complete <= 100);

  int completed_progress_bar_width = (prog.width * 1.0) * (prog.percentage_complete / 100.0);

  // progress infill
  rect_params[counter->box_num].left         = prog.top_left_corner_of_progress_bar.x + 1;
  rect_params[counter->box_num].top          = prog.top_left_corner_of_progress_bar.y;
  rect_params[counter->box_num].width        = completed_progress_bar_width;
  rect_params[counter->box_num].height       = prog.height;
  rect_params[counter->box_num].border_width = 4;
  rect_params[counter->box_num].border_color = RED_COLOR;
  rect_params[counter->box_num].bg_color     = RED_COLOR;
  rect_params[counter->box_num].has_bg_color = 1;
  counter->box_num++;

  // progress outer box
  rect_params[counter->box_num].left         = prog.top_left_corner_of_progress_bar.x;
  rect_params[counter->box_num].top          = prog.top_left_corner_of_progress_bar.y;
  rect_params[counter->box_num].width        = prog.width;
  rect_params[counter->box_num].height       = prog.height;
  rect_params[counter->box_num].border_width = 4;
  rect_params[counter->box_num].border_color = RED_COLOR;
  counter->box_num++;

  display_meta->num_rects = counter->box_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
}

// This replaces standard NVIDIA bounding box (red square) with tacti-cool "hashes" at corners
//   _          _
//  |            |
//
//
//
//
//
//  |_          _|
//
static void draw_bounding_hashes(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter, inference_detected_t bounding_box){
  if(0 == bounding_box.valid){
    return;
  }
  draw_bounding_box_corner(frame_meta, display_meta, counter, NW_CORNER, (cartesian_point_t){bounding_box.left                     , bounding_box.top                       });
  draw_bounding_box_corner(frame_meta, display_meta, counter, NE_CORNER, (cartesian_point_t){bounding_box.left + bounding_box.width, bounding_box.top                       });
  draw_bounding_box_corner(frame_meta, display_meta, counter, SW_CORNER, (cartesian_point_t){bounding_box.left                     , bounding_box.top + bounding_box.height });
  draw_bounding_box_corner(frame_meta, display_meta, counter, SE_CORNER, (cartesian_point_t){bounding_box.left + bounding_box.width, bounding_box.top + bounding_box.height });
}

static void draw_bounding_box_corner(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter, corner_type_e corner, cartesian_point_t center){
#define HASH_TARGET_LEN   (15)
#define HASH_TARGET_WIDTH (2)
  NvOSD_ColorParams color = {1.0, 1.0, 1.0, 1.0};

  if(NW_CORNER == corner) {
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x + HASH_TARGET_LEN, center.y}); 
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x                  , center.y + HASH_TARGET_LEN}); 
  } else if(NE_CORNER == corner) {
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x - HASH_TARGET_LEN, center.y}); 
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x                  , center.y + HASH_TARGET_LEN}); 
  } else if(SW_CORNER == corner) {
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x                  , center.y - HASH_TARGET_LEN}); 
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x + HASH_TARGET_LEN, center.y }); 
  } else if(SE_CORNER == corner) {
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x                  , center.y - HASH_TARGET_LEN}); 
    draw_line(frame_meta, display_meta, counter, HASH_TARGET_WIDTH, color, center, (cartesian_point_t){center.x - HASH_TARGET_LEN, center.y }); 
  }
}

static int draw_line(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter, int width, NvOSD_ColorParams color, cartesian_point_t start, cartesian_point_t end){
#define GENERIC_LINE_WIDTH (5)
  NvOSD_LineParams *line_params = display_meta->line_params;

  if(validate_point(start) | validate_point(end)){
    return 1;
  }

  line_params[counter->line_num].x1 = start.x;
  line_params[counter->line_num].y1 = start.y;
  line_params[counter->line_num].x2 = end.x;
  line_params[counter->line_num].y2 = end.y;
  line_params[counter->line_num].line_width = GENERIC_LINE_WIDTH;
  line_params[counter->line_num].line_color = color;
  counter->line_num++;

  display_meta->num_lines = counter->line_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
  return 0;
}


// checks if a cartesian point is on the screen 
static bool validate_point(cartesian_point_t point){
  if(point.x < 0 | point.y < 0){
    return 1;
  }

  if (point.x >= SCREEN_WIDTH_PIXELS | point.y >= SCREEN_HEIGHT_PIXELS){
    return 1;
  }
  return 0;
}

// At least half of the circle on the screen
static bool validate_circle(cartesian_point_t center, int radius){
  if(validate_point(center)){
    return 1;
  }
  
  if ( (center.x - radius) < 0 | (center.x + radius) > SCREEN_WIDTH_PIXELS) {
    return 1;
  }

  if ( (center.y - radius) < 0 | (center.y + radius) > SCREEN_WIDTH_PIXELS) {
    return 1;
  }

  return 0;
}

static bool validate_box(cartesian_point_t top_left, int width, int height){
  if(validate_point(top_left)){
    return 1;
  }

// Seems like deepstream does not mind drawing a box as long as it's top-left point is on the screen 
#if 0 
  if ( (top_left.x + width) > SCREEN_WIDTH_PIXELS) {
    return 1;
  }

  if ( (top_left.y + height) > SCREEN_HEIGHT_PIXELS) {
    return 1;
  }
#endif

  return 0;
}

// Draws the target and target information
//
// Draws something like this
// 
//        _________________
//        |________________| (infobox)
//       /
//      / (line A)
//     /
//  ( )  (corrected aimpoint)
//
//

static void draw_target(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter) 
{
#define UNCORRECTED_TARGET_POINT_RADIUS   (5)
#define CORRECTED_TARGET_POINT_RADIUS     (17)
#define SLOPED_LINE_WIDTH                 (40)
#define LINE_FLAIR_WIDTH                  (5)
#define INFO_BOX_WIDTH                    (212)
#define INFO_BOX_HEIGHT                   (35)
#define INFO_BOX_BORDER_WIDTH             (5)
#define INFO_BOX_TEXT_SIZE                (20)
#define INFO_BOX_TEXT_Y_OFFSET_TO_LINE_A  (39)
#define INFO_BOX_FUDGE_FACTOR             (1)

#define SINE_45_DEGREES                 (.7071)
#define COS_45_DEGREES                  (SINE_45_DEGREES)


  char mq_buff[MESSAGE_QUEUE_SIZE];
  int rc = mq_receive(crosshair_input_mq, mq_buff, MESSAGE_QUEUE_SIZE, NULL);
  if(rc == -1) {
    return;
  }
  aim_overlay_t *aim_overlay = (aim_overlay_t*)mq_buff;
  
  NvOSD_CircleParams *circle_params = &display_meta->circle_params[0];
  NvOSD_RectParams   *rect_params = &display_meta->rect_params[0];

  // Since Deepstream is placing these items on the screen, lets make sure they are being put somewhere reasonable.
  // deepstream can crash (and take down the kernel....) if an errant item is drawn

  // aim_overlay contains both the corrected and uncorrected aim points.
  // corrected accounts for bullet drop and lead
  // uncorrected is basically where the inference currently thinks the target is
  
  if(validate_circle(aim_overlay->aim_target, UNCORRECTED_TARGET_POINT_RADIUS) == 0){
    if(is_uncorrected_aim_point_enabled()){
      circle_params[counter->circle_num].xc           = aim_overlay->aim_target.x;
      circle_params[counter->circle_num].yc           = aim_overlay->aim_target.y;
      circle_params[counter->circle_num].radius       = UNCORRECTED_TARGET_POINT_RADIUS;
      circle_params[counter->circle_num].circle_color = WHITE_COLOR;
      counter->circle_num++;
    }
  }

  // The "corrected" aimpoint
  if(validate_circle(aim_overlay->aim_target_corrected_for_bullet_lead_and_drop, CORRECTED_TARGET_POINT_RADIUS) == 0){
    circle_params[counter->circle_num].xc           = aim_overlay->aim_target_corrected_for_bullet_lead_and_drop.x;
    circle_params[counter->circle_num].yc           = aim_overlay->aim_target_corrected_for_bullet_lead_and_drop.y; 
    circle_params[counter->circle_num].radius       = CORRECTED_TARGET_POINT_RADIUS;
    circle_params[counter->circle_num].circle_color = RED_COLOR;
    counter->circle_num++;
  } else {
    printf("WARN: Deepstream is trying to draw inference outside of the screen area!\n");
    goto EXIT;
  }

  // This is line A in the above diagram
  cartesian_point_t line_a_start;
  line_a_start.x = aim_overlay->aim_target_corrected_for_bullet_lead_and_drop.x + SINE_45_DEGREES*CORRECTED_TARGET_POINT_RADIUS; 
  line_a_start.y = aim_overlay->aim_target_corrected_for_bullet_lead_and_drop.y - SINE_45_DEGREES*CORRECTED_TARGET_POINT_RADIUS;
  cartesian_point_t line_a_end;
  line_a_end.x = line_a_start.x + SINE_45_DEGREES*CORRECTED_TARGET_POINT_RADIUS*2;
  line_a_end.y = line_a_start.y - SINE_45_DEGREES*CORRECTED_TARGET_POINT_RADIUS*2;
 
  // This is the info box, it will contain information about the lock
  rect_params[counter->box_num].left         = line_a_end.x - INFO_BOX_FUDGE_FACTOR;                   // fudge factor is to make things "look" nicer on the screen
  rect_params[counter->box_num].top          = line_a_end.y - INFO_BOX_HEIGHT - INFO_BOX_FUDGE_FACTOR; // fudge factor is to make things "look" nicer on the screen
  rect_params[counter->box_num].width        = INFO_BOX_WIDTH;
  rect_params[counter->box_num].height       = INFO_BOX_HEIGHT;
  rect_params[counter->box_num].border_width = INFO_BOX_BORDER_WIDTH;
  rect_params[counter->box_num].border_color = RED_COLOR;
  if(validate_box((cartesian_point_t){rect_params[counter->box_num].left, rect_params[counter->box_num].top}, INFO_BOX_WIDTH, INFO_BOX_HEIGHT)){
    goto EXIT;
  }

  // Either draw both "line A" and the infobox or neither
  counter->box_num++;
  draw_line(frame_meta, display_meta, counter, LINE_FLAIR_WIDTH, RED_COLOR, line_a_start, line_a_end);

  // Add the lock details, these must fit on the screen if the box is on the screen
  text_overlay_t txt;
  txt.font_size = INFO_BOX_TEXT_SIZE;
  txt.x = line_a_end.x;
  txt.y = line_a_end.y - INFO_BOX_TEXT_Y_OFFSET_TO_LINE_A;
  snprintf(txt.str, MAX_DISPLAY_LEN, "%.1fm/s %.1fm", get_angular_trained_angular_velocity(), get_angular_trained_distance());
  draw_text_api(frame_meta, display_meta, counter, &txt, 1);

EXIT:
  display_meta->num_rects   = counter->box_num;
  display_meta->num_circles = counter->circle_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
  return;
}

// TODO: replace with generic line API
static void draw_crosshair(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter) 
{
// Will draw something like this on the screen:
//
// --- O ---
//     |
//     |
//
  NvOSD_LineParams *line_params = display_meta->line_params;
  NvOSD_CircleParams *circle_params = display_meta->circle_params;

  if(!state_request_corrected_crosshair()){
    /* Two horizontal sections */
    line_params[counter->line_num].x1 = SCREEN_WIDTH_PIXELS/2 - CROSSHAIR_X_LEN_SEGMENT - CROSSHAIR_SEGMENT_SEPERATION/2;
    line_params[counter->line_num].y1 = SCREEN_HEIGHT_PIXELS/2;
    line_params[counter->line_num].x2 = SCREEN_WIDTH_PIXELS/2 - CROSSHAIR_SEGMENT_SEPERATION/2;
    line_params[counter->line_num].y2 = SCREEN_HEIGHT_PIXELS/2;
    line_params[counter->line_num].line_width = CROSSHAIR_WIDTH;
    line_params[counter->line_num].line_color = (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0};
    counter->line_num++;

    line_params[counter->line_num].x1 = SCREEN_WIDTH_PIXELS/2 + CROSSHAIR_SEGMENT_SEPERATION/2;
    line_params[counter->line_num].y1 = SCREEN_HEIGHT_PIXELS/2;
    line_params[counter->line_num].x2 = SCREEN_WIDTH_PIXELS/2 + CROSSHAIR_X_LEN_SEGMENT + CROSSHAIR_SEGMENT_SEPERATION/2;
    line_params[counter->line_num].y2 = SCREEN_HEIGHT_PIXELS/2;
    line_params[counter->line_num].line_width = CROSSHAIR_WIDTH;
    line_params[counter->line_num].line_color = (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0};
    counter->line_num++;

    /* Single vertical section */
    line_params[counter->line_num].x1 = SCREEN_WIDTH_PIXELS/2;
    line_params[counter->line_num].y1 = SCREEN_HEIGHT_PIXELS/2 + CROSSHAIR_SEGMENT_SEPERATION/2;
    line_params[counter->line_num].x2 = SCREEN_WIDTH_PIXELS/2;
    line_params[counter->line_num].y2 = SCREEN_HEIGHT_PIXELS/2 + CROSSHAIR_SEGMENT_SEPERATION/2 + CROSSHAIR_Y_LEN_SEGMENT;
    line_params[counter->line_num].line_width = CROSSHAIR_WIDTH;
    line_params[counter->line_num].line_color = (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0};
    counter->line_num++;
  }

  /* Center "O" */
  circle_params[counter->circle_num].xc           = SCREEN_WIDTH_PIXELS/2;
  circle_params[counter->circle_num].yc           = SCREEN_HEIGHT_PIXELS/2;
  circle_params[counter->circle_num].radius       = CROSSHAIR_CENTER_RADIUS;
  circle_params[counter->circle_num].circle_color = (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0};
  counter->circle_num++;

  display_meta->num_circles = counter->circle_num;
  display_meta->num_lines = counter->line_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
}

static void draw_tilt(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta, nv_ods_meta_shapes_counter_t* counter) 
{
  pitch_roll_rot_t orientation = imu_get_orientation();
  int roll_offset_indicator  = SCREEN_HEIGHT_PIXELS/2 + orientation.roll*(ORIENTATION_BAR_HEIGHT/2)/90;  
  int pitch_offset_indicator = SCREEN_HEIGHT_PIXELS/2 + orientation.pitch*(ORIENTATION_BAR_HEIGHT/2)/90;  
  NvOSD_RectParams *rect_params = &display_meta->rect_params[0];

  // Roll
  rect_params[counter->box_num].left         = ROLL_BAR_LEFT_OFFSET;
  rect_params[counter->box_num].top          = (SCREEN_HEIGHT_PIXELS/2) - (ORIENTATION_BAR_HEIGHT/2);
  rect_params[counter->box_num].width        = ORIENTATION_BAR_WIDTH;
  rect_params[counter->box_num].height       = ORIENTATION_BAR_HEIGHT;
  rect_params[counter->box_num].border_width = 4;
  rect_params[counter->box_num].border_color = (NvOSD_ColorParams){0.0, 1.0, 0.0, 1.0};
  counter->box_num++;

  rect_params[counter->box_num].left         = ROLL_BAR_LEFT_OFFSET;
  rect_params[counter->box_num].top          = (SCREEN_HEIGHT_PIXELS/2);
  rect_params[counter->box_num].width        = ORIENTATION_BAR_WIDTH;
  rect_params[counter->box_num].height       = 1;
  rect_params[counter->box_num].border_width = 2;
  rect_params[counter->box_num].border_color = (NvOSD_ColorParams){0.0, 1.0, 0.0, 1.0};
  counter->box_num++;

  rect_params[counter->box_num].left         = ROLL_BAR_LEFT_OFFSET;
  rect_params[counter->box_num].top          = roll_offset_indicator;
  rect_params[counter->box_num].width        = ORIENTATION_BAR_WIDTH;
  rect_params[counter->box_num].height       = 10;
  rect_params[counter->box_num].border_width = 4;
  rect_params[counter->box_num].border_color = (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0};
  counter->box_num++;

#if DRAW_PITCH
  // Pitch
  rect_params[counter->box_num].left         = PITCH_BAR_LEFT_OFFSET;
  rect_params[counter->box_num].top          = (SCREEN_HEIGHT_PIXELS/2) - (ORIENTATION_BAR_HEIGHT/2);
  rect_params[counter->box_num].width        = ORIENTATION_BAR_WIDTH;
  rect_params[counter->box_num].height       = ORIENTATION_BAR_HEIGHT;
  rect_params[counter->box_num].border_width = 4;
  rect_params[counter->box_num].border_color = (NvOSD_ColorParams){0.0, 1.0, 0.0, 1.0};
  counter->box_num++;

  rect_params[counter->box_num].left         = PITCH_BAR_LEFT_OFFSET;
  rect_params[counter->box_num].top          = (SCREEN_HEIGHT_PIXELS/2);
  rect_params[counter->box_num].width        = ORIENTATION_BAR_WIDTH;
  rect_params[counter->box_num].height       = 1;
  rect_params[counter->box_num].border_width = 2;
  rect_params[counter->box_num].border_color = (NvOSD_ColorParams){0.0, 1.0, 0.0, 1.0};
  counter->box_num++;

  rect_params[counter->box_num].left         = PITCH_BAR_LEFT_OFFSET;
  rect_params[counter->box_num].top          = pitch_offset_indicator;
  rect_params[counter->box_num].width        = ORIENTATION_BAR_WIDTH;
  rect_params[counter->box_num].height       = 10;
  rect_params[counter->box_num].border_width = 4;
  rect_params[counter->box_num].border_color = (NvOSD_ColorParams){1.0, 0.0, 0.0, 1.0};
  counter->box_num++;
#endif

  display_meta->num_rects = counter->box_num;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
}

static gboolean
bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:{
      gchar *debug;
      GError *error;
      gst_message_parse_error (msg, &error, &debug);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      if (debug)
        g_printerr ("Error details: %s\n", debug);
      g_free (debug);
      g_error_free (error);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

int deepstream_init(uint32_t unix_time_in_seconds, prog_config_t config)
{
  bounding_box_enable_overide = config.force_bb;
  int argc = 1;
  char* argv[] = {"smartscope", NULL};
  char** argv_p = argv;
  
  GMainLoop *loop = NULL;

  GstElement *pipeline       = NULL;
  GstElement *source         = NULL;
  GstElement *caps_nvargus   = NULL;
  GstElement *h264parser     = NULL;
  GstElement *decoder        = NULL;
  GstElement *streammux      = NULL;
  GstElement *sink           = NULL;
  GstElement *fakesink       = NULL;
  GstElement *pgie           = NULL;
  GstElement *nvvidconv      = NULL;
  GstElement *nvosd          = NULL;
  GstElement *queue_display  = NULL;
  GstElement *queue_record   = NULL;
  GstElement *tee            = NULL;
  GstElement *encoder        = NULL;
  GstElement *parser         = NULL;
  GstElement *mux            = NULL;
  GstElement *filesink       = NULL;
  GstElement *nvvidconv2     = NULL;
 
  GstElement *transform = NULL;
  GstBus *bus = NULL;
  guint bus_watch_id;
  GstPad *osd_sink_pad = NULL;
  GstCaps *caps_nvargus_src = NULL;
  GstCaps *caps_scale_src = NULL;
  char output_video_name[300];

  /* Create the message queues used */
  inference_output_mq = open_mq(MESSAGE_QUEUE_OUTPUT_INF, O_WRONLY | O_CREAT | O_NONBLOCK);  
  crosshair_input_mq  = open_mq(MESSAGE_QUEUE_CROSS,      O_RDONLY | O_CREAT | O_NONBLOCK);  

  /* Standard GStreamer initialization */
  gst_init (&argc, &argv_p);
  loop = g_main_loop_new (NULL, FALSE);

  /* Pipe-line, will contain everything */
  pipeline = gst_pipeline_new ("dstest1-pipeline");

  source = gst_element_factory_make ( "nvarguscamerasrc", "nv-camera" );
  g_object_set (G_OBJECT (source), "bufapi-version", 1, NULL);

  /* Configure Camera */
  caps_nvargus = gst_element_factory_make ("capsfilter", "caps-sink" );
  if(config.has_zoom) {
    caps_nvargus_src = gst_caps_from_string ("video/x-raw(memory:NVMM), format=NV12, width=1920, height=1080, framerate=30/1");
  } else {
    caps_nvargus_src = gst_caps_from_string ("video/x-raw(memory:NVMM), format=NV12, width=1280, height=720, framerate=30/1");
  }
  g_object_set (G_OBJECT (caps_nvargus), "caps", caps_nvargus_src, NULL);
  gst_caps_unref (caps_nvargus_src);

  if (!pipeline) {
    g_printerr ("Failed to create gst-pipeline. Exiting. (0)\n");
    assert(0);
  }

  /* Inference + display pipeline */
  streammux      = gst_element_factory_make ("nvstreammux"   , "stream-muxer");
  pgie           = gst_element_factory_make ("nvinfer"       , "primary-nvinference-engine");
  nvvidconv      = gst_element_factory_make ("nvvideoconvert", "nvvideo-converter");
  nvosd          = gst_element_factory_make ("nvdsosd"       , "nv-onscreendisplay");
  queue_display  = gst_element_factory_make ("queue"         , "queue-display");
  tee            = gst_element_factory_make ("tee"           , "tee");
  transform      = gst_element_factory_make ("nvegltransform", "nvegl-transform");
  sink           = gst_element_factory_make ("nveglglessink" , "nvvideo-renderer");
  fakesink       = gst_element_factory_make ("fakesink"      , "fake-sink");
 
  /* Record pipeline */
  queue_record  = gst_element_factory_make ("queue"          , "queue-record");
  encoder       = gst_element_factory_make ("nvv4l2h265enc"  , "encoder");
  parser        = gst_element_factory_make ("h265parse"      , "parser");
  mux           = gst_element_factory_make ("matroskamux"    , "mux");
  filesink      = gst_element_factory_make ("filesink"       , "file-sink");
  nvvidconv2    = gst_element_factory_make ("nvvideoconvert" , "nvvideo-converter2");

  /* Set file name */
  sprintf(output_video_name, "video_%d.mp4", unix_time_in_seconds);
  g_object_set(filesink, "location", output_video_name, NULL);

  if ( !source       || !caps_nvargus || !pgie          ||
       !nvvidconv    || !nvosd        || !queue_display ||
       !tee          || !encoder      || !parser        ||
       !mux          || !filesink     || !sink          || 
       !queue_record || !nvvidconv2   || !fakesink) {
    g_printerr ("One element could not be created. Exiting. (1)\n");
    assert(0);
  }
 
  g_object_set (G_OBJECT (streammux), "width", SCREEN_WIDTH_PIXELS, NULL);
  g_object_set (G_OBJECT (streammux), "height", SCREEN_HEIGHT_PIXELS, NULL);
  g_object_set (G_OBJECT (streammux), "batch-size", 1, NULL);
  g_object_set (G_OBJECT (sink), "sync", 0, NULL);

  /* Set the configuration for the inference */
  g_object_set (G_OBJECT (pgie), "config-file-path", "dstest1_pgie_config.txt", NULL);

  /* Add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* Ad all the items to the pipeline (not link, just add) */
  gst_bin_add_many (GST_BIN (pipeline),
      source,
      caps_nvargus,
      streammux,
      pgie,
      nvvidconv,
      nvosd,
      tee,
      queue_display,
      transform,
      queue_record,
      nvvidconv2,
      encoder,
      parser,
      mux,
      filesink,
      NULL);
  
  if(config.has_screen){
    gst_bin_add_many (GST_BIN (pipeline), sink, NULL);
  } else {
    gst_bin_add_many (GST_BIN (pipeline), fakesink, NULL);
  }

  GstPad *sinkpad, *srcpad;
  gchar pad_name_sink[16] = "sink_0";
  gchar pad_name_src[16] = "src";

  sinkpad = gst_element_get_request_pad (streammux, pad_name_sink);
  if (!sinkpad) {
    g_printerr ("Streammux request sink pad failed. Exiting.\n");
    assert(0);
  }

  srcpad = gst_element_get_static_pad (caps_nvargus, pad_name_src);
  if (!srcpad) {
    g_printerr ("Decoder request src pad failed. Exiting.\n");
    assert(0);
  }

  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK) {
    g_printerr ("Failed to link decoder to stream muxer. Exiting.\n");
    assert(0);
  }

  gst_object_unref (sinkpad);
  gst_object_unref (srcpad);

  /* Link the display + inference pipeline */
  if (!gst_element_link_many (source, caps_nvargus, NULL)) {
    g_printerr ("Elements could not be linked: (1). Exiting.\n");
    return -1;
  }
 
  if(!config.has_screen) {
    if (!gst_element_link_many (streammux, pgie, nvvidconv, nvosd, tee, queue_display, fakesink, NULL)){
      g_printerr ("Elements could not be linked: (2). Exiting.\n");
      return -1;
    }
  } else {
    if (!gst_element_link_many (streammux, pgie, nvvidconv, nvosd, tee, queue_display, transform, sink, NULL)) {
      g_printerr ("Elements could not be linked: (2). Exiting.\n");
      return -1;
    }
  }

  /* Link the recording pipeline */
  if (!gst_element_link_many (tee, queue_record, nvvidconv2, encoder, parser, mux, filesink, NULL)) {
    g_printerr ("Elements could not be linked: (5). Exiting.\n");
    return -1;
  }

  /* Lets add probe to get informed of the meta data generated, we add probe to
   * the sink pad of the osd element, since by that time, the buffer would have
   * had got all the metadata. */
  osd_sink_pad = gst_element_get_static_pad (nvosd, "sink");
  if (!osd_sink_pad) {
    g_print ("Unable to get sink pad\n");
    assert(0);
  } else {
    gst_pad_add_probe (osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, osd_sink_pad_buffer_probe, NULL, NULL);
  }
  gst_object_unref (osd_sink_pad);

  /* Set the pipeline to "playing" state */
  g_print ("Now playing: %s\n", argv[1]);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
