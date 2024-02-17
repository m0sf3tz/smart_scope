#pragma once

#define MESSAGE_QUEUE_LEN  (4)
#define MESSAGE_QUEUE_SIZE (1024*8)

// A little ugly here - but this way
// we can share between C and C++
#ifdef __cplusplus
#include <mqueue.h>
#include <errno.h>
#include <assert.h>
#include <string.h>

#include <iostream>
#include <cstdint>
#include <string>
#include <map>

class message_queue {
  private:
    static inline std::map<std::string, mqd_t> mq_store;

  public:
    message_queue() = default;
    message_queue (const message_queue&) = delete;
    message_queue& operator= (const message_queue&) = delete;

    int enqueue_message(std::string mq_path, char* buff, size_t len) {
      assert(MESSAGE_QUEUE_SIZE > len);
      mqd_t mq;

      if(0 == message_queue::mq_store.count(mq_path)) {
        struct mq_attr attr = {0};
        attr.mq_maxmsg = MESSAGE_QUEUE_LEN;
        attr.mq_msgsize = MESSAGE_QUEUE_SIZE;

        mq = mq_open (mq_path.c_str(), O_CREAT | O_WRONLY | O_NONBLOCK, 0600, &attr);
        if(mq == -1) {
          std::cout << "mq_open failed with error: " <<  strerror(errno) << std::endl;
          assert(0);
        }
        mq_store.insert({mq_path, mq});
      } else {
        mq = mq_store[mq_path];
      }
       
      int rc = mq_send(mq, buff, len, 0);
      if(rc) {
        std::cout << "mq_send len: " << len << std::endl;
        std::cout << "mq_send failed: Errno" << strerror(errno)  << std::endl;
      }
      return rc;
    }
};

int mq_enqueue(std::string, uint8_t*, size_t);
#endif
