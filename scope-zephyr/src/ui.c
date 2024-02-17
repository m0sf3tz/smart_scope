#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <assert.h>
#include "ui.h"
#include "usb.h"
#include "tlv.h"

bool static debounce_handler(const ui_event_t);

#define QUEUE_ALIGNMENT     (4)

#define ROTARY_MESSAGE_SIZE (sizeof(ui_event_t))
#define ROTARY_QUEUE_LEN    (4)
K_MSGQ_DEFINE(rotary_msgq,
        ROTARY_MESSAGE_SIZE,
        ROTARY_QUEUE_LEN,
        QUEUE_ALIGNMENT);

#define BUTTON_MESSAGE_SIZE (sizeof(ui_event_t))
#define BUTTON_QUEUE_LEN    (4)
K_MSGQ_DEFINE(button_msgq,
        BUTTON_MESSAGE_SIZE,
        BUTTON_QUEUE_LEN,
        QUEUE_ALIGNMENT);

#define HANDLE_CALLBACK(gpio, queue) do { \
  if (debounce_handler(gpio)) { return; } \
  k_msgq_put(&queue, (int[]){gpio}, K_NO_WAIT); \
  } while(0)

// Rotary Encoder
static const struct gpio_dt_spec gpio_dt_spec_encoder_button = GPIO_DT_SPEC_GET(DT_NODELABEL(rotary_click_0), gpios);
static const struct gpio_dt_spec gpio_dt_spec_encoder_a = GPIO_DT_SPEC_GET(DT_NODELABEL(rotary_a), gpios);
static const struct gpio_dt_spec gpio_dt_spec_encoder_b = GPIO_DT_SPEC_GET(DT_NODELABEL(rotary_b), gpios);
static struct gpio_callback gpio_cb_data_encoder_button;
static struct gpio_callback gpio_cb_data_encoder_a;
static struct gpio_callback gpio_cb_data_encoder_b;

// Front Button
static const struct gpio_dt_spec gpio_dt_spec_button_0 = GPIO_DT_SPEC_GET(DT_NODELABEL(button_0), gpios);
static const struct gpio_dt_spec gpio_dt_spec_button_1 = GPIO_DT_SPEC_GET(DT_NODELABEL(button_1), gpios);
static struct gpio_callback gpio_cb_data_button_0;
static struct gpio_callback gpio_cb_data_button_1;

static void rotary_encoder_cb_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  HANDLE_CALLBACK(ROTARY_A, rotary_msgq);
}

static void rotary_encoder_cb_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  HANDLE_CALLBACK(ROTARY_B, rotary_msgq);
}

static void rotary_encoder_cb_button(const struct device *dev, struct gpio_callback *cb, uint32_t pins) 
{
  HANDLE_CALLBACK(ROTARY_BUTTON, ui_ouput_msgq);
}

static void button_cb_0(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  HANDLE_CALLBACK(BUTTON_0, ui_ouput_msgq);
}

static void button_cb_1(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  HANDLE_CALLBACK(BUTTON_1, ui_ouput_msgq);
}

static bool debounce_handler(const ui_event_t event)
{
  static uint64_t last_callback[TOTAL_EVENT_COUNT];

  if((k_uptime_get() - last_callback[event]) < DEBOUNCE_TIME_MS) { return true; }
  last_callback[event] = k_uptime_get();
  return false;
}

static void setup_pin(const struct gpio_dt_spec* gpio, struct gpio_callback* cb)
{
  int rc = -1;

  if (!gpio_is_ready_dt(gpio))
  {
		printk("Error: gpio port %s is not ready\n", gpio->port->name);
	  assert(0);
	}

	rc = gpio_pin_configure_dt(gpio, GPIO_INPUT);
	if (rc != 0)
  {
		printk("Error %d: failed to configure %s pin %d\n", rc, gpio->port->name, gpio->pin);
	  assert(0);
	}

	rc = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc != 0)
  {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", rc, gpio->port->name, gpio->pin);
	  assert(0);
	}

	gpio_add_callback(gpio->port, cb);
	printk("Set up button at %s pin %d\n", gpio->port->name, gpio->pin);
}

void init_encoder(void)
{
  gpio_init_t gpios[] =
  {
    { 
      &gpio_cb_data_encoder_button,
      rotary_encoder_cb_button,
      BIT(gpio_dt_spec_encoder_button.pin), 
      &gpio_dt_spec_encoder_button 
    }, 
    { 
      &gpio_cb_data_encoder_a,
      rotary_encoder_cb_a,
      BIT(gpio_dt_spec_encoder_a.pin),
      &gpio_dt_spec_encoder_a
    }, 
    { 
      &gpio_cb_data_encoder_b,
      rotary_encoder_cb_b,
      BIT(gpio_dt_spec_encoder_b.pin),
      &gpio_dt_spec_encoder_b
    },
    { 
      &gpio_cb_data_button_0,
      button_cb_0,
      BIT(gpio_dt_spec_button_0.pin),
      &gpio_dt_spec_button_0
    }, 
    { 
      &gpio_cb_data_button_1,
      button_cb_1,
      BIT(gpio_dt_spec_button_1.pin),
      &gpio_dt_spec_button_1
    },
  };

  for(int i = 0; i < ARRAY_SIZE(gpios); i++)
  {
	  gpio_init_callback(gpios[i].callback, gpios[i].handler, gpios[i].pin_mask); 
    setup_pin(gpios[i].gpio_dt_spec, gpios[i].callback);
  }
}

void rotary_encoder_entry(void* arg0, void* arg1, void* arg2)
{
#define WAIT 0
#define PEND 1

// This is the maximum time allowed between a->b or b->a transitions.
// Experimentally this ended up working well.
#define MAX_DELAY_ENCODER 20
  rotary_encoder_e expected_event;
  rotary_encoder_e event;
  ui_event_t dir;
  int rc;
  int state;

  while(true)
  {
    state = WAIT;
    switch(state)
    {
    case WAIT:
      k_msgq_get(&rotary_msgq, &event, K_FOREVER);
      if(ROTARY_A == event)
      {
        expected_event = ROTARY_B;
        dir            = ROTARY_RIGHT;
      }
      else
      { 
        expected_event = ROTARY_A;
        dir            = ROTARY_LEFT;
      }
      state = PEND;
    case PEND:
      for(int ms_waited = 0; ms_waited < MAX_DELAY_ENCODER; ms_waited++)
      {
        k_sleep(K_MSEC(1));
        rc = k_msgq_get(&rotary_msgq, &event, K_NO_WAIT);
        if(0 == rc)
        {
          if(expected_event == event) 
          {
            if(dir == ROTARY_LEFT) 
            {
              printk("ENCODER: LEFT\n");
            }
            else
            {
              printk("ENCODER: RIGHT\n");
            }
            k_msgq_put(&ui_ouput_msgq, &dir, K_NO_WAIT);
            break;
          }
          else 
          {
            printk("unexpected input\n");
            break;
          }
        }
      }
    default:
      k_sleep(K_MSEC(5));
    }
  }
}
