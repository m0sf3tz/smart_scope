#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <time.h>

#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <fstream>
#include <regex>

#include "tty.h"
#include "sensor_board_tlv.h"
#include "radar_tlv.h"
#include "message_queue.h"

using std::make_tuple;
using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::fstream;
using std::regex;
using std::regex_search;
using std::tuple;

/* One for tracking logic, another for displaying orientation on the screen*/
static inline string mq_path_imu_tracking {"/mq_imu_tracking"};
static inline string mq_path_imu_display  {"/mq_imu_display"};

static inline string mq_path_ui  {"/mq_ui"};

void process_sensor_tlv(processed_tlv, tlv_message_type_e);

tty_handler setup_sensor_board() {
  vector<tuple<string, string, speed_t, string, int>> sensor_ports;

  // sensor board data plane
  sensor_ports.push_back(make_tuple("/dev/ttyACM1",
    "not_canonical",
    B115200, 
    "",
    IS_DATA_PORT)
  );

  tty_handler sensor_board(sensor_ports);

  return sensor_board;
}

// Returns size of package going to python OR -1 in case of error 
void process_sensor_board_tlv(processed_tlv tlv) {
  MmwDemo_output_message_tlv* sensor_tlv = reinterpret_cast<MmwDemo_output_message_tlv*> (tlv.buff + sizeof(MmwDemo_output_message_header_t));

  tlv_message_type_e type;

  if(TLV_TYPE_IMU == sensor_tlv->type) {
    type = TLV_TYPE_IMU;
  } else if (TLV_TYPE_UI == sensor_tlv->type) { 
    type = TLV_TYPE_UI;
  } else {
    cout << "Unhandled TVL for sensor board!" << endl;
    return;
  }
  
  process_sensor_tlv(tlv, type);
}

void process_sensor_tlv(processed_tlv tlv_imu, tlv_message_type_e type){
  uint8_t* sensor_sample = reinterpret_cast<uint8_t*>(tlv_imu.buff                            +
                                                   sizeof(MmwDemo_output_message_header_t) +
                                                   sizeof(MmwDemo_output_message_tlv_t)
                                                  );

  if(TLV_TYPE_IMU == type){
    mq_enqueue(mq_path_imu_tracking, sensor_sample, sizeof(imu_t));
    mq_enqueue(mq_path_imu_display, sensor_sample, sizeof(imu_t));
  } else {
    mq_enqueue(mq_path_ui, sensor_sample, sizeof(imu_t));
  }
}

int main() {
  tty_handler sensor_board = setup_sensor_board();

  while(true){
    sensor_board.tty_read_frame(); // blocking read
    auto last_sensor_tlv = sensor_board.get_last_processed_tlv();
    process_sensor_board_tlv(last_sensor_tlv);
  }
}
