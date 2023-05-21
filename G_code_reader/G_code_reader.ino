#define BAUD                 (9600)                 // How fast is the Arduino talking?(BAUD Rate of Arduino)
//#define MAX_BUF              (64)                     // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (200)                    // depends on your stepper motor.  most are 200.
#define STEPS_PER_MM         (STEPS_PER_TURN*16/0.8)  // (400*16)/0.8 with a M5 spindle
#define MAX_FEEDRATE         (1000000)
#define MIN_FEEDRATE         (1)
#define NUM_AXIES            (3)
#define motorInterfaceType 1          // For the motor interfaces

#include <AccelStepper.h>
#include <Ewma.h> // exponential filter

Ewma adcFilter2(0.05);
#pragma once

/**
   Arduino Mega with RAMPS v1.4 pin assignments
*/
//
// Servos
//
#define SERVO0_PIN                        112
#define SERVO1_PIN                         6
#define SERVO2_PIN                         5
#define SERVO3_PIN                         4

//
// Limit Switches
//

#define X_MIN_PIN                          3
#define X_MAX_PIN                          2
#define Y_MIN_PIN                         14
#define Y_MAX_PIN                         15
#define Z_MIN_PIN                         18
#define Z_MAX_PIN                         19

//
// Steppers
//
#define X_STEP_PIN                         54
#define X_DIR_PIN                          55
#define X_ENABLE_PIN                       38
#define X_CS_PIN                           53

#define Y_STEP_PIN                         60
#define Y_DIR_PIN                          61
#define Y_ENABLE_PIN                       56
#define Y_CS_PIN                           49

#define Z_STEP_PIN                         46
#define Z_DIR_PIN                          48
#define Z_ENABLE_PIN                       62
#define Z_CS_PIN                           40


#define E0_STEP_PIN                        26
#define E0_DIR_PIN                         28
#define E0_ENABLE_PIN                      24
#define E0_CS_PIN                          42


#define E1_STEP_PIN                        36
#define E1_DIR_PIN                         34
#define E1_ENABLE_PIN                      30
#define E1_CS_PIN                          44

//
// Temperature Sensors
//

#define TEMP_0_PIN                         13  // Analog Input
#define TEMP_1_PIN                         15  // Analog Input
#define TEMP_BED_PIN                       14  // Analog Input

//
// Heaters / Fans
//
#define RAMPS_D8_PIN                        8
#define RAMPS_D9_PIN                        9
#define RAMPS_D10_PIN                      10

//
// I2C
//
#define SDA_PIN                           20 
#define SCL_PIN                           21 

//////////////////////////
// LCDs and Controllers //
//////////////////////////

// DOGM SPI LCD Support
#define DOGLCD_CS                       16
#define DOGLCD_MOSI                     17
#define DOGLCD_SCK                      23

// REPRAP_DISCOUNT_SMART_CONTROLLER
#define D31                             31  // BTN_EN1
#define D33                             33  // BTN_EN2
#define D35                             35  // BTN_ENC
#define BEEPER_PIN                      37 
#define SD_DETECT_PIN                   49 
#define KILL_PIN                        41
#define LCD_BACKLIGHT_PIN               39

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
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
  int limit_switch_pin;
} Motor;

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];
float Temp_C  = 0;

// speeds
float fr=0;  // human version
long step_delay;  // machine version

float px,py,pz;  // position

// settings
char mode_abs=1;  // absolute mode?

long line_number=0;

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
  Serial.print("Pause: ");
  Serial.println(ms);
}

/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  nfr = nfr*STEPS_PER_MM/60; 
  Serial.print("feedrate: ");
  Serial.println(nfr);
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = MAX_FEEDRATE/nfr;
  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy,float npz) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
}

/**
 * Supports movement with both styles of Motor Shield
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void onestep(int motor) {
#ifdef VERBOSE
  char *letter="XYZE";
  Serial.print(letter[]);
#endif
  
  digitalWrite(motors[motor].step_pin,HIGH);
  digitalWrite(motors[motor].step_pin,LOW);
}

/*
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 *
 */
void line(float newx,float newy,float newz) {
  a[0].delta = (newx-px)*STEPS_PER_MM;
  a[1].delta = (newy-py)*STEPS_PER_MM;
  a[2].delta = (newz-pz)*STEPS_PER_MM;
  
  long i,j,maxsteps=0;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].absdelta = abs(a[i].delta);
    a[i].over=0;
    if( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    // set the direction once per movement
    digitalWrite(motors[i].dir_pin,a[i].delta>0?HIGH:LOW);
  }
  
  long dt = MAX_FEEDRATE/5000;
  long accel = 1;
  long steps_to_accel = dt - step_delay;
  if(steps_to_accel > maxsteps/2 ) 
    steps_to_accel = maxsteps/2;
    
  long steps_to_decel = maxsteps - steps_to_accel;

  Serial.print("START ");
  Serial.println(dt);
  Serial.print("STOP ");
  Serial.println(step_delay);
  
  Serial.print("accel until ");
  Serial.println(steps_to_accel);  
  Serial.print("decel after ");
  Serial.println(steps_to_decel);  
  Serial.print("total ");
  Serial.println(maxsteps);  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  for( i=0; i<maxsteps; ++i ) {
    for(j=0;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps) {
        a[j].over -= maxsteps;
        
        digitalWrite(motors[j].step_pin,HIGH);
        digitalWrite(motors[j].step_pin,LOW);
      }
    }

    if(i<steps_to_accel) {
      dt -= accel;
    }
    if(i>=steps_to_decel) {
      dt += accel;
    }
    delayMicroseconds(dt);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  position(newx,newy,newz);

  where();
}

// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}

/*
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 */
float parseNumber(char code,float val, String text) {
  int index_of_char = text.indexOf(code);
  if (index_of_char != -1) {
    int index_of_space = text.indexOf(" ",index_of_char);
    String out = text.substring(index_of_char+1, index_of_space);
    return out.toFloat();
  }
  return val;  // end reached, nothing found, return default val.
}

/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.print(val);
  Serial.print(" ");
}

/*

 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("Z",pz);
  output("F",fr/STEPS_PER_MM*60);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("3d printer demo"));
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 X/Y/Z(steps) F(feedrate); - linear move"));
  Serial.println(F("G04 Pseconds; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 X/Y/Z(steps); - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M17; - enable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand(String command) {
  int cmd = parseNumber('G',-1, command);
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parseNumber('F',fr, command));
    line( parseNumber('X',(mode_abs?px:0), command) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0), command) + (mode_abs?0:py),
          parseNumber('Z',(mode_abs?pz:0), command) + (mode_abs?0:pz) );
    break;
    }
  case  2:
  case  4:  pause(parseNumber('P',0, command)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0, command),
              parseNumber('Y',0, command),
              parseNumber('Z',0, command) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1, command);
  switch(cmd) {
  case  17:  motor_enable();  break;
  case  18:  motor_disable();  break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}

/**
 * set up the pins for each motor
 * Pins fits a CNCshieldV3.xx
 */
void motor_setup() {
  motors[0].step_pin=X_STEP_PIN;
  motors[0].dir_pin=X_DIR_PIN;
  motors[0].enable_pin=X_ENABLE_PIN;
  motors[0].limit_switch_pin=9;

  motors[1].step_pin=Y_STEP_PIN;
  motors[1].dir_pin=Y_DIR_PIN;
  motors[1].enable_pin=Y_ENABLE_PIN;
  motors[1].limit_switch_pin=10;

  motors[2].step_pin=Z_STEP_PIN;
  motors[2].dir_pin=Z_DIR_PIN;
  motors[2].enable_pin=Z_ENABLE_PIN;
  motors[2].limit_switch_pin=11;

  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
  }
  //AccelStepper motors[0] = AccelStepper(motorInterfaceType, X_STEP_PIN, X_DIR_PIN);

}


void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,LOW);
  }
  Serial.println("Motors enable");
}


void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,HIGH);
  }
  Serial.println("Motors disable");
}

void setup() {
  Serial.begin(BAUD);  // open coms

  motor_setup();
  motor_enable();
  
  where();  // for debugging purposes
  help();  // say hello
  position(0,0,0);  // set starting position
  feedrate(1000);  // set default speed
}


void loop() {
  // listen for serial commands
  /*
  while(Serial.available() > 0) {  // if something is available
    String cmd=Serial.readStringUntil('\n');  // get it
    Serial.println(cmd);  // repeat it back so I know you got the message
    processCommand(cmd);  // do something with the command
    
    //delay(5000)
  }
  */
  //onestep(0);
}
