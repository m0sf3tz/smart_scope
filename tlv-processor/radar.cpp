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

static PointCloudSpherical radar_point_cloud;

tty_handler setup_radar() {
  vector<tuple<string, string, speed_t, string, int>> radar_ports;

  // Control Plane
  radar_ports.push_back(make_tuple("/dev/ttyUSB0",
    "canonical", 
    B115200,
    "./configs/68xx_traffic_monitoring_70m_MIMO_2D.cfg",
    IS_CFG_PORT)
  );

  // Radar data plane
  radar_ports.push_back(make_tuple("/dev/ttyUSB1",
    "not_canonical",
    B921600, 
    "",
    IS_DATA_PORT)
  );

  // Either opens radar successfully or asserts
  tty_handler radar(radar_ports);

  return radar;
}

// TLV structure 
// <header> <MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS> [ point cloud ] <MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO> [ point cloud side info] <MISC TLVs> [MISC DATA] 
// Returns size of package going to python OR -1 in case of error 
int process_radar_tlv(processed_tlv tlv) {
  // Read the First TLV
  MmwDemo_output_message_tlv* tlv_ptr_spherical = reinterpret_cast<MmwDemo_output_message_tlv*> (tlv.buff + sizeof(MmwDemo_output_message_header_t));
  // don't boher if we don't have any points
  if(MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS != tlv_ptr_spherical->type) { 
    // The code running on the IRW68XX will first send TLVs containing 
    // point cloud information (which we are after).
    // However, if nothing is detected, it will send out other meta data we don't care
    // about, in that case we won't bother processing it.
    // If points are detected, the first header will be of type MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS
    return -1;
  }

  size_t cloud_payload_len = tlv_ptr_spherical->length; 
  int points_in_cloud = tlv_ptr_spherical->length / sizeof(DPIF_PointCloudSpherical);
  if(points_in_cloud > MAX_CLOUD_POINTS){
    points_in_cloud = MAX_CLOUD_POINTS;
    puts("WARNING: exceeed number of points in a frame, will clip!");
  }
  printf("Detected %u in the point cloud\n", points_in_cloud);

  // This is badly named (by TI, so we won't updat it) 
  // What we do here is extract individual points in the point 
  // cloud and store them. Cld will contain an individual point
  // in the point cloud, not the entire cloud
  DPIF_PointCloudSpherical* cld;
  for(int i = 0; i < points_in_cloud; i++) {
    cld = reinterpret_cast<DPIF_PointCloudSpherical*>(tlv.buff                                + 
                                                      sizeof(MmwDemo_output_message_header_t) +
                                                      sizeof(MmwDemo_output_message_tlv_t)    +
                                                      i*sizeof(DPIF_PointCloudSpherical)
                                                     );
    radar_point_cloud.points[i].sphere = *cld;
  }

  // Get the TLV for the SNR 
  MmwDemo_output_message_tlv* tlv_ptr_snr = reinterpret_cast<MmwDemo_output_message_tlv*>(tlv.buff                                +
                                                                                          sizeof(MmwDemo_output_message_header_t) + 
                                                                                          sizeof(MmwDemo_output_message_tlv_t)    +
                                                                                          cloud_payload_len);
  if(MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO != tlv_ptr_snr->type) {
    printf("Error, unexpected TLV in place of SNR TLV: TLV for SNR == %d\n", tlv_ptr_snr->type);
    return -1;
  }

  size_t side_info_payload_len = tlv_ptr_snr->length; 
  int points_in_side_info = side_info_payload_len / sizeof(DPIF_PointCloudSideInfo);
  assert(points_in_cloud == points_in_side_info);

  for(int i = 0; i < points_in_side_info; i++) {
    DPIF_PointCloudSideInfo* side_info = reinterpret_cast<DPIF_PointCloudSideInfo*>(tlv.buff                                +
                                                                                    sizeof(MmwDemo_output_message_header_t) +
                                                                                    sizeof(MmwDemo_output_message_tlv_t)    +
                                                                                    cloud_payload_len                       +
                                                                                    sizeof(MmwDemo_output_message_tlv_t)    +
                                                                                    i*sizeof(DPIF_PointCloudSideInfo)
                                                                                   );
    radar_point_cloud.points[i].side = *side_info;
  }  

  radar_point_cloud.meta_data.points = points_in_cloud;
  return sizeof(PointCloudMetaData) + points_in_cloud*sizeof(SphericalPointAndSnr);
}

void enque_to_python_radar(int buff_size){
  struct timespec tp;
  clock_gettime(CLOCK_MONOTONIC, &tp);
  radar_point_cloud.meta_data.seconds     = tp.tv_sec;
  radar_point_cloud.meta_data.nanoseconds = tp.tv_nsec;
  mq_enqueue(RADAR_MQ_PATH, reinterpret_cast<uint8_t*>(&radar_point_cloud), buff_size);
}

void program_loop() {
  tty_handler radar = setup_radar();
  while(true){
    // Blocking read 
    if(REQUEST_RESET == radar.tty_read_frame()){
      break;
    }

    auto last_radar_tlv = radar.get_last_processed_tlv();
    auto buff_size = process_radar_tlv(last_radar_tlv);
    if(buff_size > 0) {
      enque_to_python_radar(buff_size);
    }
  }

  // tty_hanlders destructor will clean up the port
}

int main(){
  while(true){
    program_loop();
    
    // if we return from program_loop - it means we have a big 
    // radar bug - do a hard reset of the radar system
    usb_radar_reset();
    
    sleep(3); // give some time for the USB to re-enumerate
  }
}
