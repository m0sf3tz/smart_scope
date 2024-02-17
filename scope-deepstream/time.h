#pragma once 
#include <stdint.h>

int get_seconds_from_epoch(void);
double get_ms_since_start(void); // format = ms.us 
uint32_t get_time_monotonic(void);
