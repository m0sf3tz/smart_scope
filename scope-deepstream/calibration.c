#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <stdint.h>

#include "calibration.h"
#include "crc.h"
#include "menu.h"

#define CALIBRATION_FILE_NAME "bullet_calibration.dat"
#define CALIBRATION_FILE_PATH "/etc/radar"
#define CALIBRATION_FULL_PATH CALIBRATION_FILE_PATH "/" CALIBRATION_FILE_NAME

static calibration_data_with_crc cal_data;
static int calibration_fd;

static int validate_crc_calibration_data(void);
static int read_calibration_data(void);
static void save_calibration_data(void);
static int open_calibration_file(void);

int16_t fetch_calibration_value(calibration_enum_e type, uint16_t distance, uint16_t velocity){
  assert(distance <= MAX_CALIBRATION_POINTS);
  assert(velocity <= MAX_CALIBRATION_POINTS);

  if(DROP_CALIBRATION_MENU == type){
    return cal_data.data.drop_calibration_points[distance];
  } else if (LEAD_CALIBRATION_MENU == type) {
    return cal_data.data.lead_calibration_points[distance][velocity];
  }
}

void save_calibration_value(calibration_enum_e type, uint16_t distance, uint16_t velocity, int16_t value){
  assert(distance <= MAX_CALIBRATION_POINTS);
  assert(velocity <= MAX_CALIBRATION_POINTS);

  if(DROP_CALIBRATION_MENU == type){
    cal_data.data.drop_calibration_points[distance] = value;
  } else if (LEAD_CALIBRATION_MENU == type) {
    cal_data.data.lead_calibration_points[distance][velocity] = value;
  }
}

int load_calibration_data_and_verify_crc(){
  open_calibration_file();

  if(read_calibration_data()){
    return -1;
  }
  if(validate_crc_calibration_data()){
    return -1;
  }
  puts("Successfully loaded calibration data");
  return 0;
}

void save_calibration_data_with_crc(){
  uint32_t calculated_crc32 = crc32((uint8_t*)&cal_data.data, sizeof(calibration_data_t));
  cal_data.crc32 = calculated_crc32;

  save_calibration_data();
}

static int read_calibration_data(){
  int rc = read(calibration_fd, &cal_data, sizeof(cal_data));
  if(rc != sizeof(cal_data)){
    return -1;
  }
  return 0;
}

static int validate_crc_calibration_data(){
  uint32_t calculated_crc32 = crc32((uint8_t*)&cal_data.data, sizeof(calibration_data_t));
  if(calculated_crc32 != cal_data.crc32){
    return -1;
  }
  return 0;
}

static void save_calibration_data(){
  lseek(calibration_fd,0,SEEK_SET);

  int rc = write(calibration_fd, &cal_data, sizeof(cal_data));
  if(rc != sizeof(cal_data)){
    assert(0);
  } 
}

static int open_calibration_file(){
  printf("Calibration data path: %s\n", CALIBRATION_FULL_PATH);
  int rc = open(CALIBRATION_FULL_PATH , O_RDWR | O_CREAT, 0666);
  if (rc == -1){
    printf("Failed to open calibration file, error %s\n", strerror(errno));
    assert(0);
  }
  calibration_fd = rc; 
}
