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
#include <tuple>
#include <regex>
#include <fstream>

#define MAX_TLV_READ_SIZE  (2048)
#define MAGIC_START_BYTES  (8)
#define MAX_TLV_SIZE       (1024*200)

#define IS_DATA_PORT (0)
#define IS_CFG_PORT  (1)

#define PORT_IN_TUPLE       (0)
#define MODE_IN_TUPLE       (1)
#define SPEED_IN_TUPLE      (2)
#define CFG_PATH_IN_TUPLE   (3)
#define DESCIPLINE_IN_TUPLE (4)

#define REQUEST_RESET      (-0xFFFF)
#define MAX_ZERO_LEN_READS (0xFF)

// From TI, header information 
typedef struct MmwDemo_output_message_header_t {
    uint16_t    magicWord[4];
    uint32_t    version;
    uint32_t    totalPacketLen;
    uint32_t    platform; 
    uint32_t    frameNumber;
    uint32_t    timeCpuCycles;
    uint32_t    numDetectedObj;
    uint32_t    numTLVs;
    uint32_t    subFrameNumber;
} MmwDemo_output_message_header;

// From TI, TLV header
typedef struct MmwDemo_output_message_tlv_t {
    uint32_t    type;
    uint32_t    length;
} MmwDemo_output_message_tlv;

typedef struct {
  uint8_t* buff;
  size_t len;
} processed_tlv;

typedef enum { STATE_FIND_MAGIC, STATE_READ_REST } tlv_read_state_machine_e;

class tty_handler {
 private:
  size_t _number_of_ports;
  int data_port_fd;
  int cfg_port_fd;

  std::vector<std::tuple<std::string, std::string, speed_t, std::string, int>> _port_configs;
  
  // Both the Radar and Accelerometer will start the TLV by sending
  // 4 uint16_t in the header"
  //
  // uint16_t[0] = 0x0201
  // uint16_t[1] = 0x0403
  // uint16_t[2] = 0x0605
  // uint16_t[2] = 0x0807
  //
  // Since both the host computer (running this code) and the sensor CPUs
  // Are little endian, the byte ordering needs to be swapped to come up 
  // with key_d
  static const inline uint8_t magic_key[] = {2,1,4,3,6,5,8,7};
  uint8_t read_buff[MAX_TLV_READ_SIZE + 1];
  uint8_t tlv_buff[MAX_TLV_SIZE + 1]; 
  size_t read_index{0};
  size_t remainder{0};
  size_t last_tlv_size{0};
  int    bytes_read{0};
  int    zero_len_reads_counter{0};

  int read_stream() {
    read_index = 0; 
    int rc = read(data_port_fd, read_buff, MAX_TLV_READ_SIZE);  
    if(0 == rc){
      zero_len_reads_counter++;
    } else {
      zero_len_reads_counter = 0;  
    }
    
    if(zero_len_reads_counter > MAX_ZERO_LEN_READS){
      return REQUEST_RESET;
    } 
  
    return rc;
  }

  void init_ports();
  void apply_cfg();
  void tty_send_cfg(const std::string& cfg);
  bool tty_check_done();
  std::vector<std::string> read_cfg(std::string&);
 
 public:
  int read_tlv(uint8_t*);
  // expects a tuple <port_path, mode , speed, cfg_file_path, type>
  // Port is the /dev/ path
  // Mode is if the port is canonical mode or not
  // speed is the buad rate
  // type is one of "IS_DATA_PORT" or "IS_CFG_PORT"
  tty_handler(std::vector<std::tuple<std::string, std::string, speed_t, std::string, int>>&);

  ~tty_handler(){
    printf("\n\n***Closing USB ports**\n\n");
    if(data_port_fd){
      close(data_port_fd);
    }
    if(cfg_port_fd){
      close(cfg_port_fd);
    }
  }
 
  // Reads a single TLV from the stream
  int tty_read_frame();

  processed_tlv get_last_processed_tlv() { 
    processed_tlv ret;
    ret.buff = tlv_buff;
    ret.len  = last_tlv_size;
    return ret;
  }
};

// Helper functions
void usb_radar_reset();
