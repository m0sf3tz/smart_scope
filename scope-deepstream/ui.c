#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>

#include "sensor_board_tlv.h"
#include "ui.h"

static mqd_t     ui_mq;
static pthread_t ui_th;

static pthread_mutex_t ui_mutex = PTHREAD_MUTEX_INITIALIZER;
static tlv_message_type_e event = TOTAL_EVENT_COUNT;

static void* ui_thread(void*);

bool fetch_new_event(ui_event_e* event_ret){
  bool ret = false;

  pthread_mutex_lock(&ui_mutex);
  if(event != TOTAL_EVENT_COUNT){
    *event_ret = event;
    event = TOTAL_EVENT_COUNT;
    ret = true;
  }
  pthread_mutex_unlock(&ui_mutex);

  return ret;
}

static void set_new_event(ui_event_e new_event){
  pthread_mutex_lock(&ui_mutex);
  event = new_event;
  pthread_mutex_unlock(&ui_mutex);
}

static void open_ui_mq(){
  ui_mq = open_mq(MESSAGE_QUEUE_NAME_UI, O_RDONLY | O_CREAT);
}

void init_ui_thread(){
  open_ui_mq();
  
  int rc = pthread_create(&ui_th, NULL, ui_thread, NULL);
  if(rc != 0){
    printf("Failed to start ui_thread with error %s\n", strerror(rc));
    assert(0);
  }
}

static void get_ui_event(){
  char mq_buff[MESSAGE_QUEUE_SIZE]; 
  
  int rc = mq_receive(ui_mq, mq_buff, MESSAGE_QUEUE_SIZE, NULL);
  if(-1 == rc){
    printf("Failed to recieve from UI message queue, error: %s\n", strerror(errno));
    return;
  }

  // UI event is a single byte
  ui_event event = mq_buff[0];
  bool send_event = false;
  switch(event){
    case(ROTARY_BUTTON):
      send_event = true;
      puts("ROTARY_BUTTON event");
      break;
    case(ROTARY_LEFT):
      puts("ROTARY_LEFT event");
      send_event = true;
      break;
    case(ROTARY_RIGHT):
      puts("ROTARY_RIGHT event");
      send_event = true;
      break;
  }

  if(send_event) {
    set_new_event(event);
  }
}

static void* ui_thread(void* arg){
  printf("UI thread staring.\n");

  while(1){
    get_ui_event();
  }
}
