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

#include "tty.h"
#include "radar_tlv.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::tuple;
using std::make_tuple;
using std::get;
using std::regex;
using std::fstream;

tty_handler::tty_handler(vector<tuple<string, string, speed_t, string, int>>& port_configs) {
  if(0 == port_configs.size()) {
    printf("Incorrect number of ports specified\n");
    assert(0);
  }

  _number_of_ports = port_configs.size();
  _port_configs = port_configs;

  init_ports();
  apply_cfg();
}

// Sets up line disciple for TTY
void tty_handler::init_ports() {
  auto iter = _port_configs.begin();

  while(iter != _port_configs.end()) {
    string port = get<PORT_IN_TUPLE>(*iter);
    string mode = get<MODE_IN_TUPLE>(*iter);
    int speed   = get<SPEED_IN_TUPLE>(*iter);

    int fd = open(port.c_str(), O_NOCTTY | O_SYNC | O_RDWR);
    if(-1 == fd) {
        printf("Failed to open port located at %s returned with errno: %s\n", port.c_str(), strerror(errno));
        assert(0);
    }
    (IS_DATA_PORT == get<DESCIPLINE_IN_TUPLE>(*iter)) ? (data_port_fd = fd) : (cfg_port_fd = fd);
    
    struct termios tty;
    if(tcgetattr(fd, &tty) != 0) {
			printf("Failed to get tty attributes, returned with errno: %s\n", strerror(errno));
      assert(0);
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit
    tty.c_cflag &= ~CSTOPB; // Single stop bit
    tty.c_cflag &= ~CSIZE;  // 8its
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_oflag &= ~ONLCR;
    tty.c_lflag &= ~(ECHO | ECHOE);
    tty.c_lflag &= ~ISIG;  // Disable interpreting INTR, QUIT and SUSP signals
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
   
    // Configure the configuration port as a Canonical TTY - this way
    // the TTY driver will read one line at a time.
    // Configure the data plane to be non-canonical mode
    // set time==0, byte==1 such that the terminal will 
    // always pause for a single byte to come through
    if (mode == "canonical") {  
	    tty.c_lflag |= ICANON; // Enable Canonical mode
    } else { 
	    tty.c_lflag &= ~ICANON; // Disable Canonical mode
      tty.c_cc[VMIN]  = 1;
      tty.c_cc[VTIME] = 0;
    }
 
    if(B115200 == speed) {
      cfsetispeed(&tty, B115200);
      cfsetospeed(&tty, B115200);
    } else {
      cfsetispeed(&tty, B921600);
      cfsetospeed(&tty, B921600);   
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
       printf("Failed to set tty attributes, returned with errno: %s\n", strerror(errno));
       assert(0);
    }
    iter++; 
  }
}

void tty_handler::apply_cfg() {
  printf("Starting to send Configs to radar\n");
  
  auto iter = _port_configs.begin();
  while(iter != _port_configs.end()) {
    string cfg_file = get<CFG_PATH_IN_TUPLE>(*iter);
    if(cfg_file == "") {
      iter++; 
      continue;
    }

    auto list_of_configs = read_cfg(cfg_file);

    for(const auto& cfg_cmd : list_of_configs) {
      tty_send_cfg(cfg_cmd);

      if(false == tty_check_done()) { 
        printf("Did not receive 'Done' when sending: %s\n", cfg_cmd.c_str());
        assert(0);
      }
    }
    iter++; 
  } 
}

void tty_handler::tty_send_cfg(const string& cfg) {
  printf("Applying CLI CMD: %s\n", cfg.c_str());
  int rc = write(cfg_port_fd, cfg.c_str(), cfg.size());
  if(rc != cfg.size()) {
	  printf("Failed to write to cfg port with errno = %s\n", strerror(errno));
    assert(0);
  }
}

void process_ui_tlv(uint8_t* data, size_t size) {
  uint32_t* message = (uint32_t*)(data + sizeof(MmwDemo_output_message_header_t) + sizeof(MmwDemo_output_message_tlv));
  cout << "UI event : " << *message << endl;
}

// Read the terminal, looks for "Done" which 
// Is what the 3.6 version of the mmWave demo 
// outputs once it has been configured.
bool tty_handler::tty_check_done() {
#define CHECK_FOR_DONE_RETRY_COUNT (5)
#define READ_BUFF_SIZE  (250)

  char read_buff[READ_BUFF_SIZE + 1] = { 0 };
  size_t index = 0;
 
  for(int i = 0; i < CHECK_FOR_DONE_RETRY_COUNT; i++) {
    int rc = read(cfg_port_fd, &read_buff[index], READ_BUFF_SIZE - index);

    if (rc < 0) {
      printf("Failed to read cfg with read error: %s\n", strerror(errno));
      assert(0);
    } else if (rc > 0) {
      string read_so_far { read_buff };
      regex rx { "Done" };

      if (regex_search(read_so_far, rx)) {
        return true;
      }
      index += rc;
    }
  }
  return false;
}

vector<string> tty_handler::read_cfg(string& cfg_file_path) {
  string line;
  fstream cfg_stream(cfg_file_path);
  vector<string> list_of_configs;

  if (cfg_stream.is_open()) {
    puts("Reading in config file");
    while(getline(cfg_stream, line)) {
      list_of_configs.push_back(line + '\r');
    }
    puts("**** Done sending config  ****");
  } else {
    puts("Could not open stream!");
    assert(0);
  }
  return list_of_configs;
}

// reduce code duplication

// Note, sometimes the radar gets stuck and repetivly returns 0,
// A condition has been added to check for this, if we hit this clause
// We will reset the USB FD for the radar
#define READ_CHUNK() do {                                                  \
      remainder = bytes_read - read_index;                                 \
      max_read = (remainder >= to_read) ? to_read : remainder;             \
      memcpy(tlv_buff + tlv_processed, &read_buff[read_index], max_read);  \
      to_read       -= max_read;                                           \
      read_index    += max_read;                                           \
      tlv_processed += max_read;                                           \
      if(to_read) {                                                        \
        bytes_read = read_stream();                                        \
        if(bytes_read == REQUEST_RESET){                                   \
          return REQUEST_RESET;                                            \
        }                                                                  \
      }                                                                    \
    } while(to_read);                                                      \

#define RESET_LOGIC() do {      \
      magic_matched = 0;        \
      tlv_processed = 0;        \
      state = STATE_FIND_MAGIC; \
    } while(0);                 \

int tty_handler::tty_read_frame() {
  int magic_matched    = 0;  // how many of the magic start bytes we matched
  size_t tlv_processed = 0;  // how much of the TLV we have currently read in
  int state            = STATE_FIND_MAGIC; 
  MmwDemo_output_message_header_t *ptr = NULL;
  
  size_t remainder;
  size_t to_read;
  size_t max_read;

  while(true) {
    switch(state) {
      case STATE_FIND_MAGIC:
        remainder = bytes_read - read_index;
        if (remainder == 0) {
          bytes_read = read_stream();
          printf("read %d\n", bytes_read);
          if(bytes_read == 0) { 
            break;
          }
        }

        while(read_index != bytes_read) {
          if(magic_key[magic_matched] == read_buff[read_index]) {
            magic_matched++;
            if(MAGIC_START_BYTES == magic_matched) {
              state = STATE_READ_REST;
              read_index++;
              break;
            }
          } else {
            magic_matched = 0;
          }
          read_index++;
        }
        if(state != STATE_READ_REST) {
          puts("Missed header");
        } else {
          printf("Found magic header\n");
          tlv_processed = MAGIC_START_BYTES;
        }
        break;
      case STATE_READ_REST:
        to_read = sizeof(MmwDemo_output_message_header) - MAGIC_START_BYTES;
        READ_CHUNK()
        
        ptr = reinterpret_cast<MmwDemo_output_message_header_t*>(tlv_buff); 
        to_read = ptr->totalPacketLen - sizeof(MmwDemo_output_message_header);
        last_tlv_size = ptr->totalPacketLen;
        if(to_read > MAX_TLV_SIZE - sizeof(MmwDemo_output_message_header)) {
          printf("Error: exceeded TLV size!\n");
          RESET_LOGIC();
          break;
        }       

        printf("Will read %lu bytes on frame %d with [%d] TLVs", to_read, ptr->frameNumber, ptr->numTLVs);
        READ_CHUNK();
        return 0;
    }
  }
}

// This function does a hard reset of a USB port
void usb_radar_reset(){
  // TODO... don't hardcode paths
  system("/opt/nvidia/deepstream/deepstream-6.0/sources/apps/scope-jetson/usb-reset/usbreset");
}
