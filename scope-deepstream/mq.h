#pragma once
#include <mqueue.h>
#include "message_queue.h"

#define MESSAGE_QUEUE_NAME       "/mq_imu_display" 

// goes to python
#define MESSAGE_QUEUE_OUTPUT_INF "/mq_inference" 
// comes from python
#define MESSAGE_QUEUE_CROSS      "/mq_crosshair" 

mqd_t open_mq(const char *, int);
