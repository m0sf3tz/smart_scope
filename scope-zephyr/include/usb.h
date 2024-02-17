#pragma once 

extern struct k_msgq imu_ouput_msgq;
extern struct k_msgq ui_ouput_msgq;

void init_usb(void);
void usb_entry(void*, void*, void*);
