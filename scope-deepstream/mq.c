#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "mq.h"

mqd_t open_mq(const char* mq_path, int flags) 
{
  assert(mq_path);
  
  struct mq_attr attr = {0};
  attr.mq_maxmsg = MESSAGE_QUEUE_LEN;
  attr.mq_msgsize = MESSAGE_QUEUE_SIZE;

  mqd_t ret = mq_open(mq_path, flags, 0600, &attr);
  if(ret == -1){
    printf("Failed to open message queue for (%s)\n", mq_path);
    assert(0);
  }
  return ret;
}
