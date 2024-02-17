#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>

#include "sensor_board_tlv.h"
#include "mq.h"

#define MESSAGE_QUEUE_NAME_UI "/mq_ui"

bool fetch_new_event(ui_event_e* event_arg);
void init_ui_thread(void);
