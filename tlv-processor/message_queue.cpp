#include "message_queue.h"
#include <string>

using std::string;

int mq_enqueue(string mq_path, uint8_t * buff, size_t size) {
  message_queue mq; 
  return mq.enqueue_message(mq_path, reinterpret_cast<char*>(buff), size);
}
