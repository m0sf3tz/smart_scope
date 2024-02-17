#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>
#include <glib.h>

int get_seconds_from_epoch() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec;
}

double get_ms_since_start(){
  static struct timeval start_time;
  struct timeval time_now;
  
  if(start_time.tv_sec == 0){
    gettimeofday(&start_time, NULL);
  }
  
  gettimeofday(&time_now, NULL);
  long seconds        = time_now.tv_sec  - start_time.tv_sec;
  long micro_seconds  = time_now.tv_usec - start_time.tv_usec;
  
  return (seconds * 1000.0 + micro_seconds / 1000.0);
}

uint32_t get_time_monotonic(){
  struct timespec monotime;
  clock_gettime(CLOCK_MONOTONIC, &monotime);
  return monotime.tv_sec;
}
