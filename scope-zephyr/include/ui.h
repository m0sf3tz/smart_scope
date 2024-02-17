#pragma once 

#define LEFT   0
#define RIGHT  1
#define BUTTON 2

#define DEBOUNCE_TIME_MS (8)

typedef enum {
  ROTARY_A = 0,
  ROTARY_B
} rotary_encoder_e;

typedef uint32_t ui_event_t;
typedef enum { 
  ROTARY_BUTTON = 0,
  ROTARY_LEFT,
  ROTARY_RIGHT,
  BUTTON_0,
  BUTTON_1,
  TOTAL_EVENT_COUNT
} ui_event_e;

typedef struct {
  struct gpio_callback * callback;
  gpio_callback_handler_t handler;
  gpio_port_pins_t pin_mask;
  const struct gpio_dt_spec* gpio_dt_spec;
} gpio_init_t;

void init_encoder(void);
void rotary_encoder_entry(void*, void*, void*);
void button_entry(void*, void*, void*);
