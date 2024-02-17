#pragma once

#include "gstnvdsmeta.h"
#include "sensor_board_tlv.h"

#define MAX_MENU_DEPTH (25)
#define MAX_MENU_TEXT  (75)

#define ITEM_UNDER_CALIBRATION_INDEX      (0)
#define DISTANCE_UNDER_CALIBRATION_INDEX  (1)
#define VELOCITY_UNDER_CALIBRATION_INDEX  (2)
#define DISPLAY_BUFF_LEN                  (400)

#define MENU_TIMEOUT_TIME (6)
#define MISC_NULL_VAL (0)

#define MENU_LEAD_TYPE (0)
#define MENU_DROP_TYPE (1)

#define MENU_ACCELERATE_CUTOFF_MS (1250)

typedef enum {LEAD_CALIBRATION_MENU, DROP_CALIBRATION_MENU} calibration_enum_e;
typedef uint32_t menu_type;

typedef struct menu_item{
  char text[MAX_MENU_TEXT]; // Text to display on the screen
  struct menu_item * next;   // Next menu if this menu is selected
  void (*pre_entry)();
  void (*display)(NvDsFrameMeta*, NvDsDisplayMeta*);
  void (*handle_ui)(ui_event_e);
  uint32_t misc; // for example, if this menu is the calibration menu for 10m, this value will be 10
                 // this way we can avoid ugly atoi etc.
  menu_type type;
  
} menu_item_t;

typedef struct{
  calibration_enum_e type;
  uint32_t distance; 
  uint32_t velocity;
} unwind_menu_stack_t;

bool draw_menu(NvDsFrameMeta *frame_meta, NvDsDisplayMeta *display_meta);
void register_all_menus(void);
bool is_bounding_box_enabled(void);
bool is_uncorrected_aim_point_enabled(void);
bool is_debug_info_enabled(void);
