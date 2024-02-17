#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <stdbool.h>
#include <cuda_runtime_api.h>
#include "gstnvdsmeta.h"

#include "menu.h"
#include "ui.h"
#include "sensor_board_tlv.h"
#include "deepstream.h"
#include "calibration.h"
#include "time.h"
#include "interpolate.h"

// Extra one is to hold the sentinel value
menu_item_t menu_stack[MAX_MENU_DEPTH + 1];
menu_item_t main_menu[MAX_MENU_DEPTH + 1]; 
menu_item_t calibration_lead_distance_menu[TOTAL_CALIBRATION_DISTANCE_POINTS + 1]; 
menu_item_t calibration_lead_velocity_menu[TOTAL_CALIBRATION_VELOCITY_POINTS + 1];
menu_item_t calibration_drop_distance_menu[TOTAL_CALIBRATION_DISTANCE_POINTS + 1]; 

static void calibration_ui_handle_input(ui_event_e event);
static void generate_bullet_drop_calibration_menus(void);
static void generate_bullet_lead_calibration_menus(void);
static void fill_main_menu(void);
static void update_menu_ui_event(ui_event_e);
static void reset_menu(void);
static bool menu_active(void);
static bool is_menu_valid(menu_item_t*);
static unwind_menu_stack_t unwind_stack_calibration(void);
static void null_ui_function(ui_event_e);
static void reset_func_display(NvDsFrameMeta*, NvDsDisplayMeta*);
static void pre_entry_reset(void);
static void pre_entry_bbox(void);
static void bbox_func_display(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta);
static void dump_calibration_drop(NvDsFrameMeta*, NvDsDisplayMeta*);
static void dump_calibration_lead(NvDsFrameMeta*, NvDsDisplayMeta*);
static void save_calibration_table(NvDsFrameMeta *, NvDsDisplayMeta *);
static void toggle_single_distance_calibration_point(NvDsFrameMeta *, NvDsDisplayMeta *);
static void pre_entry_toggle_single_distance_calibration(void);
static void uncorrected_aimpoint_func_display(NvDsFrameMeta *, NvDsDisplayMeta *);
static void debug_info_func_display(NvDsFrameMeta*, NvDsDisplayMeta*);
static void pre_entry_draw_uncorrected_aim_point(void);
static void pre_entry_draw_debug_info(void);

static bool force_single_calibration_distance;
static int16_t current_calibrated_value;
static uint32_t last_event_time;
static uint32_t menu_timeout_time;
static size_t curr_menu_index;
static size_t stack_index;
static menu_item_t* curr_menu = main_menu; 
static void (*leaf_func_display)(NvDsFrameMeta*, NvDsDisplayMeta*);
static void (*leaf_func_handle_ui)(ui_event_e);
static void display_current_menu(NvDsFrameMeta*, NvDsDisplayMeta*);
static bool in_leaf_func;
static uint32_t reset_time;
static bool enable_bounding_box;
static bool enable_uncorrected_aim_point = true; // this will draw on the screen where we are aiming for without lead/drop correction
static bool display_debug_info = true;

bool is_debug_info_enabled(){
  return display_debug_info;
}

bool is_bounding_box_enabled(){
  return enable_bounding_box;
}

bool is_uncorrected_aim_point_enabled(){
  return enable_uncorrected_aim_point;
}

bool draw_menu(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  ui_event_e new_event;

  if(fetch_new_event(&new_event)){
    if(!menu_active() && (ROTARY_BUTTON == new_event)){
      return false;
    }
  
    last_event_time = get_time_monotonic();
    menu_timeout_time = last_event_time + MENU_TIMEOUT_TIME;

    if(in_leaf_func) {
      leaf_func_handle_ui(new_event);
    } else {
      update_menu_ui_event(new_event);
    }
  }

  if(menu_active()){
    // If a menu is active, we are either in a leaf menu function - ie,
    // calibration menu etc, or we are still displaying the menu
    if(in_leaf_func) {
      leaf_func_display(frame_meta, display_meta);
    } else {
      display_current_menu(frame_meta, display_meta);
    }

    return true;
  } else {
    reset_menu();
    return false;
  }
}

static bool menu_active(){
  return (0 != last_event_time) && (get_time_monotonic() < menu_timeout_time);
}

static bool kill_active_menu(){
  menu_timeout_time = 0;
}

static void display_current_menu(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  // Sanity check our index and menu
  assert(is_menu_valid(&curr_menu[curr_menu_index]));
  generic_func_display(frame_meta, display_meta, curr_menu[curr_menu_index].text, NULL);
}

static void reset_menu(){
  stack_index = 0;
  curr_menu_index = 0;
  in_leaf_func = false;

  curr_menu = main_menu; 
  leaf_func_display   = NULL;
  leaf_func_handle_ui = NULL;
}

static void update_menu_ui_event(ui_event_e new_event){
  if(ROTARY_BUTTON == new_event){
    menu_stack[stack_index] = curr_menu[curr_menu_index];
    stack_index++;
  
    // The way the menu is encoded, we are have a "next" menu,
    // or we are at a leaf in a menu and we select our final UI
    // functions.
    if(curr_menu[curr_menu_index].next){
      curr_menu = curr_menu[curr_menu_index].next;
    } else {
      puts("Entering Leaf UI function...");
      if(curr_menu[curr_menu_index].pre_entry) {
        curr_menu[curr_menu_index].pre_entry();
      }

      leaf_func_display   = curr_menu[curr_menu_index].display;
      leaf_func_handle_ui = curr_menu[curr_menu_index].handle_ui;
      in_leaf_func = true;
    }

    curr_menu_index = 0; 
  } else if (ROTARY_RIGHT == new_event) {
    if(!is_menu_valid(&curr_menu[curr_menu_index + 1])){
      curr_menu_index = 0;
    } else {
      curr_menu_index++;
    }
  } else if (ROTARY_LEFT == new_event) {
    if(curr_menu_index == 0){
      // This is annoying, we are going from the top of the list to the bottom, but since we don't
      // have a "tail pointer" we need to iterate.
      while(is_menu_valid(&curr_menu[curr_menu_index + 1])){
        curr_menu_index++;  
      }
    } else {
      curr_menu_index--;
    }
  } else {
    puts("Unknown UI event");
    assert(0);
  }
} 

static bool is_menu_valid(menu_item_t *item){
  assert(item);
  
  if(item->text[0] == '\0') {
    return false;
  } else {
    return true;
  }
}

void register_all_menus(){
  fill_main_menu();
  
  generate_bullet_lead_calibration_menus();
  generate_bullet_drop_calibration_menus();
}

#define REGISTER_MAIN_MENU_ITEM(STR, NEXT, MISC, HANDLE_UI, HANDLE_DISPLAY, PRE, TYPE) do { \
   snprintf(main_menu[item].text, MAX_MENU_TEXT, STR); \
   main_menu[item].next = NEXT; \
   main_menu[item].misc = MISC; \
   main_menu[item].handle_ui = HANDLE_UI; \
   main_menu[item].display = HANDLE_DISPLAY; \
   main_menu[item].pre_entry = PRE; \
   main_menu[item].type = TYPE; \
   item++; \
  } while(0);

static void fill_main_menu(){
  int item = 0;

  REGISTER_MAIN_MENU_ITEM("Calibrate bullet lead", calibration_lead_distance_menu, MISC_NULL_VAL, NULL, NULL, NULL, LEAD_CALIBRATION_MENU);
  REGISTER_MAIN_MENU_ITEM("Calibrate bullet drop", calibration_drop_distance_menu, MISC_NULL_VAL, NULL, NULL, NULL, DROP_CALIBRATION_MENU);
  REGISTER_MAIN_MENU_ITEM("Dump calibration (drop)", NULL, MISC_NULL_VAL, null_ui_function, dump_calibration_drop, NULL, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Dump calibration (lead)", NULL, MISC_NULL_VAL, null_ui_function, dump_calibration_lead, NULL, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Save calibration tables (both)", NULL, MISC_NULL_VAL, null_ui_function, save_calibration_table, NULL, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Toggle force single distance calibration point", NULL, MISC_NULL_VAL, null_ui_function, toggle_single_distance_calibration_point, pre_entry_toggle_single_distance_calibration, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Restart program", NULL, MISC_NULL_VAL, null_ui_function, reset_func_display, pre_entry_reset, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Toggle bounding box", NULL, MISC_NULL_VAL, null_ui_function, bbox_func_display, pre_entry_bbox, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Toggle uncorrected aimpoint", NULL, MISC_NULL_VAL, null_ui_function, uncorrected_aimpoint_func_display, pre_entry_draw_uncorrected_aim_point, MISC_NULL_VAL);
  REGISTER_MAIN_MENU_ITEM("Toggle debug info", NULL, MISC_NULL_VAL, null_ui_function, debug_info_func_display, pre_entry_draw_debug_info, MISC_NULL_VAL);

  assert(MAX_MENU_DEPTH > item);
}

static void pre_entry_calibration(){
  unwind_menu_stack_t stack = unwind_stack_calibration();
  current_calibrated_value = fetch_calibration_value(stack.type, stack.distance, stack.velocity);
}

static void pre_entry_reset(){
  reset_time = get_time_monotonic() + MENU_TIMEOUT_TIME/2;  
}

static void null_ui_function(ui_event_e event){
}

static void pre_entry_bbox(){
  enable_bounding_box = !enable_bounding_box;
}

static void pre_entry_draw_debug_info(){
  display_debug_info = !display_debug_info;
}

static void pre_entry_draw_uncorrected_aim_point(){
  enable_uncorrected_aim_point = !enable_uncorrected_aim_point;
}

static void bbox_func_display(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  char * txt;

  if(is_bounding_box_enabled()){
    txt = "Bounding box is enabled.";
  } else {
    txt = "Bounding box is disabled.";
  }
  generic_func_display(frame_meta, display_meta, txt, NULL);
}

static void debug_info_func_display(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  char str[DISPLAY_BUFF_LEN];
  snprintf(str, DISPLAY_BUFF_LEN, "Drawing debug info: %s", (display_debug_info ? "ON" : "OFF"));
  generic_func_display(frame_meta, display_meta, str, NULL);
}

static void uncorrected_aimpoint_func_display(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  char str[DISPLAY_BUFF_LEN];
  snprintf(str, DISPLAY_BUFF_LEN, "Drawing uncorrected aim point: %s", (is_uncorrected_aim_point_enabled ? "ON" : "OFF"));
  generic_func_display(frame_meta, display_meta, str, NULL);
}

static void reset_func_display(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  if (get_time_monotonic() > reset_time){
    // Note... this is ugly - it does not properly clean up gstream. It's ok since we will
    // rarely use this feature but something to keep in mind.
    exit(0);
  }

  generic_func_display(frame_meta, display_meta, "Getting ready for shutdown.", NULL);
}

static void pre_entry_toggle_single_distance_calibration(){
  char str[DISPLAY_BUFF_LEN];
  force_single_calibration_distance ? (force_single_calibration_distance = 0) : (force_single_calibration_distance = 1);
}

static void toggle_single_distance_calibration_point(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  char str[DISPLAY_BUFF_LEN];
  snprintf(str, DISPLAY_BUFF_LEN, "Forcing single calibration distance: %s", (force_single_calibration_distance ? "ON" : "OFF"));
  generic_func_display(frame_meta, display_meta, str, NULL);
}

static void save_calibration_table(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  save_calibration_data_with_crc();
  generic_func_display(frame_meta, display_meta, "Saving calibration data.", NULL);
}

static void dump_calibration_lead(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
#define PRINTF_GRID_WIDTH (8)
  char lead_table[DISPLAY_BUFF_LEN];
  char string[DISPLAY_BUFF_LEN];
  snprintf(lead_table, DISPLAY_BUFF_LEN, "Calibration table [bullet lead]\n\n");

  strcat(lead_table, "             ");
  for(int v = 0; v < TOTAL_CALIBRATION_VELOCITY_POINTS; v++){
    int velocity = CALIBRATION_VELOCITY_START + CALIBRATION_VELOCITY_JUMP*v;
    snprintf(string, MAX_MENU_TEXT, "%dm/s    ", velocity);
    strcat(lead_table, string);
  }
  strcat(lead_table, "\n");

  for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
    int distance = CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*d;
    snprintf(string, MAX_MENU_TEXT, "Distance %d: ", distance);
    strcat(lead_table, string);

    // This code prints out the calibration values plus a pad such that strlen(pad + calibration_value) == PRINTF_GRID_WIDTH
    for(int v = 0; v < TOTAL_CALIBRATION_VELOCITY_POINTS; v++){
      snprintf(string, MAX_MENU_TEXT, "%d", fetch_calibration_value(LEAD_CALIBRATION_MENU, d, v));
      int value_str_len = strlen(string);
      assert( (PRINTF_GRID_WIDTH-value_str_len) > 0);
      strcat(lead_table, string);
      snprintf(string, MAX_MENU_TEXT, "%*s", PRINTF_GRID_WIDTH-value_str_len, "");
      strcat(lead_table, string);
    }
    strcat(lead_table, "\n");
  }
  generic_func_display(frame_meta, display_meta, lead_table, NULL);
}

static void dump_calibration_drop(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  char drop_table[DISPLAY_BUFF_LEN];
  char string[DISPLAY_BUFF_LEN];
  snprintf(drop_table, DISPLAY_BUFF_LEN, "Calibration table [bullet drop]\n");
   
  for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
    int distance = CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*d;
    snprintf(string, DISPLAY_BUFF_LEN, "%d meters : %d pixels\n", distance, fetch_calibration_value(DROP_CALIBRATION_MENU, d, DONT_CARE));
    strcat(drop_table, string);
  }
  
  generic_func_display(frame_meta, display_meta, drop_table, NULL);
}

static void calibration_ui_handle_input(ui_event_e event){
#define WHEEL_HISTORY_CIRC_BUFF_LEN (15)
  static int wheel_history[WHEEL_HISTORY_CIRC_BUFF_LEN];
  static int wheel_index;

  // implements rotary "acceleration"
  int calibrated_jump_value = 1;
  if(event == ROTARY_LEFT || event == ROTARY_RIGHT) {
    int time_now = (int)get_ms_since_start();
    int time_cutoff = time_now - MENU_ACCELERATE_CUTOFF_MS;
    int recent_ui = 0;

    wheel_history[wheel_index] = time_now;
    for(int i = 0; i < WHEEL_HISTORY_CIRC_BUFF_LEN; i++){ 
      if(wheel_history[i] >= time_cutoff){
        recent_ui++;
      }
    }
    wheel_index = (wheel_index + 1) % WHEEL_HISTORY_CIRC_BUFF_LEN ;
    
    if(recent_ui > 10) {
      calibrated_jump_value = 5;
    }
    if(recent_ui > 15) {
      calibrated_jump_value = 15;
    }
  }

  if(event == ROTARY_RIGHT){
    current_calibrated_value += calibrated_jump_value;
  } 
  if(event == ROTARY_LEFT){
    current_calibrated_value -= calibrated_jump_value;
  }

  if(event == ROTARY_BUTTON){
    unwind_menu_stack_t stack = unwind_stack_calibration(); 
    printf("%d %d %d %d\n", stack.type, stack.distance, stack.velocity, current_calibrated_value);

    if(force_single_calibration_distance){
      // Calibrates the gun to work for only at one point. Will make debugging easier since 
      // it reduces calibration points greatly 
      for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
        save_calibration_value(stack.type, d, stack.velocity, current_calibrated_value);
      }
    } else {
      save_calibration_value(stack.type, stack.distance, stack.velocity, current_calibrated_value);
    }

    // Update interpolation menues 
    interpolate_create_lead();
    init_interpolation_distance();
    kill_active_menu();  
  }  
}

static void bullet_calibration_display_func(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta){
  // Sanity check our index and menu
  assert(is_menu_valid(&curr_menu[curr_menu_index]));
  char txt[DISPLAY_BUFF_LEN];

  if(menu_stack[ITEM_UNDER_CALIBRATION_INDEX].type == DROP_CALIBRATION_MENU) {
    snprintf(txt, DISPLAY_BUFF_LEN, 
      "%s at distance %s \nValue is (%d)\nClick wheel to accept new value.",
      menu_stack[ITEM_UNDER_CALIBRATION_INDEX].text, 
      menu_stack[DISTANCE_UNDER_CALIBRATION_INDEX].text, 
      current_calibrated_value
    );
  } else if(menu_stack[ITEM_UNDER_CALIBRATION_INDEX].type == LEAD_CALIBRATION_MENU) {
    snprintf(txt, DISPLAY_BUFF_LEN, 
      "%s at distance %s and velocity %s\nValue is (%d)\nClick wheel to accept new value.",
      menu_stack[ITEM_UNDER_CALIBRATION_INDEX].text, 
      menu_stack[DISTANCE_UNDER_CALIBRATION_INDEX].text, 
      menu_stack[VELOCITY_UNDER_CALIBRATION_INDEX].text,
      current_calibrated_value
    );
  }

  generic_func_display(frame_meta, display_meta, txt, NULL);
}

static unwind_menu_stack_t unwind_stack_calibration(){ 
  unwind_menu_stack_t stack;

  stack.type     = menu_stack[ITEM_UNDER_CALIBRATION_INDEX].type;
  stack.distance = menu_stack[DISTANCE_UNDER_CALIBRATION_INDEX].misc;
  stack.velocity = menu_stack[VELOCITY_UNDER_CALIBRATION_INDEX].misc;

  return stack;
}

static void generate_bullet_lead_calibration_menus() {
  for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
    int distance = CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*d;

    snprintf(calibration_lead_distance_menu[d].text, MAX_MENU_TEXT, "%dm", distance);
    calibration_lead_distance_menu[d].next      = calibration_lead_velocity_menu;
    calibration_lead_distance_menu[d].misc      = d;
    calibration_lead_distance_menu[d].pre_entry = pre_entry_calibration;
  }
  
  for(int v = 0; v < TOTAL_CALIBRATION_VELOCITY_POINTS; v++){
    int velocity = CALIBRATION_VELOCITY_START + CALIBRATION_VELOCITY_JUMP*v;

    snprintf(calibration_lead_velocity_menu[v].text, MAX_MENU_TEXT, "%dm/s", velocity);
    calibration_lead_velocity_menu[v].handle_ui = calibration_ui_handle_input;
    calibration_lead_velocity_menu[v].display   = bullet_calibration_display_func;
    calibration_lead_velocity_menu[v].misc      = v;
    calibration_lead_velocity_menu[v].pre_entry = pre_entry_calibration;
  }
}

static void generate_bullet_drop_calibration_menus(){
  for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
    int distance = CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*d;

    snprintf(calibration_drop_distance_menu[d].text, MAX_MENU_TEXT, "%dm", distance);
    calibration_drop_distance_menu[d].display   = bullet_calibration_display_func;
    calibration_drop_distance_menu[d].handle_ui = calibration_ui_handle_input;
    calibration_drop_distance_menu[d].misc      = d;
    calibration_drop_distance_menu[d].pre_entry = pre_entry_calibration;
  }
}
