#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "Arduino.h"
#include "SD_process.h"
#include "Constants.h"
#include "Heaters.h"

extern bool intr_hap;
extern float fr;
extern long step_delay;
extern float px,py,pz,pe;
extern char mode_abs;

// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
} Axis;

typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int max_pin;
  int min_pin;
} Motor;

void pause(long ms);
void feedrate(float nfr);
void position(float npx,float npy,float npz, float npe);
void onestep(int motor);
void line(float newx,float newy,float newz, float newe);
static float atan3(float dy,float dx);
void output(char *code,float val);
void where();
void motor_setup();
void motor_calibration(int motor);
void motor_enable();
void motor_disable();
void Stop_Xmotor_Max();
void Stop_Xmotor_Min();
void Stop_Ymotor_Max();
void Stop_Ymotor_Min();

#endif