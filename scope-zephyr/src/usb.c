#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <assert.h>

#include "tlv.h"
#include "usb.h"
#include "imu.h"
#include "ui.h"

static void create_and_send_tlv(tlv_message_type_e const, uint8_t* const, size_t const);

#define QUEUE_ALIGNMENT  (4)

#define IMU_MESSAGE_SIZE (sizeof(imu_t))
#define IMU_QUEUE_LEN    (2)
K_MSGQ_DEFINE(imu_ouput_msgq,
        IMU_MESSAGE_SIZE,
        IMU_QUEUE_LEN,        
        QUEUE_ALIGNMENT);

#define ENCODER_MESSAGE_SIZE (sizeof(imu_t))
#define ENCODER_QUEUE_LEN    (2)
K_MSGQ_DEFINE(ui_ouput_msgq,
        ENCODER_MESSAGE_SIZE,
        ENCODER_QUEUE_LEN,       
        QUEUE_ALIGNMENT);

void init_usb()
{
  if (usb_enable(NULL))
  {
		assert(0);
	}

#ifdef WAIT_FOR_DTR
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	const struct device *const spew    = DEVICE_DT_GET(DT_NODELABEL(uart_spew));
  
  open_and_wait_for_drt(console);
  open_and_wait_for_drt(spew);
#endif
}

#ifdef WAIT_FOR_DTR
void open_and_wait_for_drt(const struct device *const dev)
{
	/* Wait until DTR is released, this happens when the user opens the /dev/ttyACM0 device */
	while (!dtr) 
  {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(10));
	}
}
#endif 

void usb_entry(void* arg0, void* arg1, void* arg2)
{
  imu_t imu_sample;
  ui_event_t ui_event;

  struct k_poll_event events[2];
  k_poll_event_init(&events[0],
                    K_POLL_TYPE_FIFO_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY,
                    &imu_ouput_msgq);
  k_poll_event_init(&events[1],
                    K_POLL_TYPE_FIFO_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY,
                    &ui_ouput_msgq);
  
  while(1)
  {
    if(0 == k_poll(events, 2, K_FOREVER)) 
    {
      if(0 == k_msgq_get(&imu_ouput_msgq, &imu_sample, K_NO_WAIT))
      {
        printk("got an imu sample\n");
        create_and_send_tlv(TLV_TYPE_IMU, (uint8_t*)&imu_sample, sizeof(imu_sample));
      }
      if(0 == k_msgq_get(&ui_ouput_msgq, &ui_event, K_NO_WAIT))
      {
        printk("got an UI event (%d)\n", ui_event);
        create_and_send_tlv(TLV_TYPE_UI, (uint8_t*)&ui_event, sizeof(ui_event_t));
      }
    }
    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;
  }
}

static void tlv_send(const uint8_t * data, size_t len) 
{
  assert(data);

	const struct device *const spew = DEVICE_DT_GET(DT_NODELABEL(uart_spew));
  while(len--) {
    uart_poll_out(spew, *data++);
  }
}

static void create_and_send_tlv(tlv_message_type_e const type, uint8_t* const data, size_t const size)
{
  static uint8_t buff[MAX_TLV_SIZE];

  size_t len = encode_tlv(buff, MAX_TLV_SIZE, data, type, size); 
  tlv_send(buff, len);
}
