#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>

#include "deepstream.h"
#include "imu.h"
#include "ui.h"
#include "menu.h"
#include "radar.h"
#include "algo.h"
#include "time.h"
#include "calibration.h"
#include "interpolate.h"

static void smart_scope(prog_config_t config){
  int seconds_from_epoch = get_seconds_from_epoch();

  load_calibration_data_and_verify_crc();
  init_interpolation_distance();
  interpolate_create_lead();
  init_imu_thread();
  init_algo_thread();
  if(config.calibrate_imu_on_boot){
    calibrate_imu();
  }
  init_ui_thread();
  init_radar_thread(seconds_from_epoch);
  register_all_menus();

  deepstream_init(seconds_from_epoch, config);
}

int main(int argc, char **argv){
  prog_config_t config = {0};
  int opt;

  while ((opt = getopt (argc, argv, "szfc")) != -1){
    switch (opt)
    {
    case 's':
      config.has_screen = 1;
      break;
    case 'z':
      config.has_zoom = 1;
      break;
    case 'f':
      config.force_bb = 1;
      break;
    case 'c':
      config.calibrate_imu_on_boot = 1;
      break;
    defualt:
      assert(0);
    }
  }

  // Does not return
  smart_scope(config);
}
