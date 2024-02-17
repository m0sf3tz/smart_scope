#include <stdio.h>
#include <stdlib.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_interp2d.h>
#include <gsl/gsl_spline2d.h>

#include "calibration.h"
#include "menu.h"

static gsl_interp *interp;
static gsl_spline2d *spline;
static gsl_interp_accel *xacc;
static gsl_interp_accel *yacc;

static double drop_distance[TOTAL_CALIBRATION_DISTANCE_POINTS];
static double drop_value[TOTAL_CALIBRATION_DISTANCE_POINTS];

// TODO: leaks if called more than twice 
void interpolate_create_lead() {
  const gsl_interp2d_type *T = gsl_interp2d_bilinear;

  double *xa = malloc(TOTAL_CALIBRATION_DISTANCE_POINTS * sizeof(double));
  double *ya = malloc(TOTAL_CALIBRATION_VELOCITY_POINTS * sizeof(double));
  double *za = malloc(TOTAL_CALIBRATION_DISTANCE_POINTS * TOTAL_CALIBRATION_VELOCITY_POINTS * sizeof(double));

  spline = gsl_spline2d_alloc(T, TOTAL_CALIBRATION_DISTANCE_POINTS, TOTAL_CALIBRATION_VELOCITY_POINTS);
  xacc = gsl_interp_accel_alloc();
  yacc = gsl_interp_accel_alloc();

  for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
    xa[d] = (double)(CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*d);
  }
  for(int v = 0; v < TOTAL_CALIBRATION_VELOCITY_POINTS; v++){
    ya[v] = (double)(CALIBRATION_VELOCITY_JUMP*v);
  }

  for(int d = 0; d < TOTAL_CALIBRATION_DISTANCE_POINTS; d++){
    for(int v = 0; v < TOTAL_CALIBRATION_VELOCITY_POINTS; v++){
      gsl_spline2d_set(spline,
        za,
        d,
        v,
        fetch_calibration_value(LEAD_CALIBRATION_MENU, d, v)
      );
    }
  }

  gsl_spline2d_init(spline, xa, ya, za, TOTAL_CALIBRATION_DISTANCE_POINTS, TOTAL_CALIBRATION_VELOCITY_POINTS);
}

double get_interpolation_lead(double distance_to_interpolate, double velocity_to_interpolate){
  // Only consider positive velocity, if velocity is negative it's corrected for in algo.h
  velocity_to_interpolate = fabs(velocity_to_interpolate);

  if(distance_to_interpolate < CALIBRATION_DISTANCE_START){
    distance_to_interpolate = CALIBRATION_DISTANCE_START;
  } else if(distance_to_interpolate > MAX_CALIBRATION_DISTANCE) {
    distance_to_interpolate = MAX_CALIBRATION_DISTANCE;
  }
  
  if(velocity_to_interpolate < CALIBRATION_VELOCITY_START){
    velocity_to_interpolate = CALIBRATION_VELOCITY_START;
  } else if(velocity_to_interpolate > MAX_CALIBRATION_VELOCITY) {
    velocity_to_interpolate = MAX_CALIBRATION_VELOCITY;
  }

  return gsl_spline2d_eval(spline, distance_to_interpolate, velocity_to_interpolate, xacc, yacc);
}

void init_interpolation_distance(){
  interp = gsl_interp_alloc(gsl_interp_linear, TOTAL_CALIBRATION_DISTANCE_POINTS);
  for(int i=0; i < TOTAL_CALIBRATION_DISTANCE_POINTS; i++){
    int cur_distance = CALIBRATION_DISTANCE_START + CALIBRATION_DISTANCE_JUMP*i;
    drop_distance[i] = cur_distance;
    drop_value[i] = fetch_calibration_value(DROP_CALIBRATION_MENU, i, DONT_CARE);  
  }
  interp = gsl_interp_alloc(gsl_interp_linear, TOTAL_CALIBRATION_DISTANCE_POINTS);
  gsl_interp_init(interp, drop_distance, drop_value, TOTAL_CALIBRATION_DISTANCE_POINTS);
}

double get_interpolation_distance(double distance_to_interpolate){
  if(distance_to_interpolate < CALIBRATION_DISTANCE_START){
    distance_to_interpolate = CALIBRATION_DISTANCE_START;
  } else if(distance_to_interpolate > MAX_CALIBRATION_DISTANCE) {
    distance_to_interpolate = MAX_CALIBRATION_DISTANCE;
  }

  return gsl_interp_eval(interp, drop_distance, drop_value, distance_to_interpolate, NULL);
}
