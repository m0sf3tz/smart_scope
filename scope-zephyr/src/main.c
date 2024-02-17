#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <assert.h>

#include "ui.h"
#include "imu.h"
#include "usb.h"
#include "tlv.h"

#define ENCODER_THREAD_SIZE 1024
K_THREAD_STACK_DEFINE(encoder_thread_stack, ENCODER_THREAD_SIZE);
struct k_thread encoder_thread;

#define IMU_THREAD_SIZE 1024
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_SIZE);
struct k_thread imu_thread;

#define USB_THREAD_SIZE 1024
K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_SIZE);
struct k_thread usb_thread;

int main()
{
  init_usb();
  init_imu();
  init_encoder();

  k_thread_create(&encoder_thread, encoder_thread_stack,
     K_THREAD_STACK_SIZEOF(encoder_thread_stack),
     rotary_encoder_entry,
     NULL, NULL, NULL,
     0, 0, K_NO_WAIT);


  k_thread_create(&usb_thread, usb_thread_stack,
     K_THREAD_STACK_SIZEOF(usb_thread_stack),
     usb_entry,
     NULL, NULL, NULL,
     1, 0, K_NO_WAIT);

/*
  k_thread_create(&imu_thread, imu_thread_stack,
     K_THREAD_STACK_SIZEOF(imu_thread_stack),
     send_tlv_entry,
     NULL, NULL, NULL,
     3, 0, K_NO_WAIT);
*/
}
