#define BAUD                 (9600)                 // How fast is the Arduino talking?(BAUD Rate of Arduino)
//#define MAX_BUF              (64)                     // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (200)                    // depends on your stepper motor.  most are 200.
#define STEPS_PER_MM         (STEPS_PER_TURN*16/0.8)  // (400*16)/0.8 with a M5 spindle
#define STEPS_PER_MM_X         STEPS_PER_MM
#define STEPS_PER_MM_Y         STEPS_PER_MM
#define STEPS_PER_MM_Z         STEPS_PER_MM
#define STEPS_PER_MM_E         STEPS_PER_MM*0.01
#define MAX_FEEDRATE         (1000000)
#define MIN_FEEDRATE         (1)
#define NUM_AXIES            (4)
#define motorInterfaceType 1          // For the motor interfaces

#include <AccelStepper.h>
#include <Ewma.h> // exponential filter
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
//#include <U8g2lib.h>     /* LCD */
#include "SD_process.h"

Ewma adcFilter2(0.05);
//U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, 3, 1, 0);
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
//#define X_CS_PIN                           53

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
  int max_pin;
  int min_pin;
} Motor;

typedef struct {
  int therm_pin;
  int heating_pin;
  float temp;
  float reach_temp;
} Heater;

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];
Heater heaters[2];

SD_process SD_card;


bool intr_hap = 0;
// speeds
float fr=0;  // human version
long step_delay;  // machine version

float px,py,pz,pe;  // position

// settings
char mode_abs=1;  // absolute mode?

long line_number=0;

String cmd_line, cmd;

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
void position(float npx,float npy,float npz, float npe) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
  pe=npe;
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
void line(float newx,float newy,float newz, float newe) {
  a[0].delta = (newx-px)*STEPS_PER_MM_X;
  a[1].delta = (newy-py)*STEPS_PER_MM_Y;
  a[2].delta = (newz-pz)*STEPS_PER_MM_Z;
  a[3].delta = (newe-pe)*STEPS_PER_MM_E;
  
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
  if (intr_hap == 0){
    position(newx,newy,newz,newe);
  }
  else if(intr_hap ==1){
    for(int i=0;i<1;++i) {  
    // set the motor pin & scale
    //motor_calibration(i);
    }
  }
  intr_hap = 0;

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
  output("E",pe);
  output("F",fr/STEPS_PER_MM*60);
  Serial.println(mode_abs?"ABS":"REL");
  Serial.println();
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
  Serial.println(F("M17; - enable motors"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M106; - turn on hotend fan"));
  Serial.println(F("M107; - turn off hotend fan"));
  Serial.println(F("M104 S(temp); - set hotend temperature"));
  Serial.println(F("M109 S(temp); - set hotend temperature and wait"));
  Serial.println(F("M140 S(temp); - set bed temperature"));
  Serial.println(F("M190 S(temp); - set bed temperature and wait"));
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
    feedrate(parseNumber('F',fr/STEPS_PER_MM*60, command));
    line( parseNumber('X',(mode_abs?px:0), command) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0), command) + (mode_abs?0:py),
          parseNumber('Z',(mode_abs?pz:0), command) + (mode_abs?0:pz),
          parseNumber('E',(mode_abs?pe:0), command) + (mode_abs?0:pe) );
    break;
    }
  case  2:
  case  4:  pause(parseNumber('P',0, command)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0, command),
              parseNumber('Y',0, command),
              parseNumber('Z',0, command),
              parseNumber('E',0, command) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1, command);
  switch(cmd) {
  case  17:  motor_enable();  break;
  case  18:  motor_disable();  break;
  case 106: fan_turnon(); break; // turn on the hotend fan
  case 107: fan_turnoff(); break; // turn off the hotend fan
  case 104: {  // set hotend temperature
    heaters[0].reach_temp = parseNumber('S',0,command);
    Temp_control(0);
    break;
  }
  case 109: {  // set hotend temperature and wait
    heaters[0].reach_temp = parseNumber('S',0,command);
    Temp_control_wait(0);
    break;
  }
  case 140: {  // set bed temperature
    heaters[1].reach_temp = parseNumber('S',0,command);
    Temp_control(1);
    break;
  }
  case 190: {  // set bed temperature and wait
    heaters[1].reach_temp = parseNumber('S',0,command);
    Temp_control_wait(1);
    break;
  }
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}
/*
 * set up the hotend, hotbed and hotend fan
*/
void heater_setup(){
  heaters[0].therm_pin = TEMP_0_PIN;
  heaters[0].heating_pin = RAMPS_D10_PIN;
  heaters[0].temp = 0;
  heaters[0].reach_temp = 0;
  pinMode(heaters[0].therm_pin, INPUT_PULLUP);
  pinMode(heaters[0].heating_pin, OUTPUT);

  heaters[1].therm_pin = TEMP_1_PIN;
  heaters[1].heating_pin = RAMPS_D8_PIN;
  heaters[1].temp = 0;
  heaters[1].reach_temp = 0;
  pinMode(heaters[0].therm_pin, INPUT_PULLUP);
  pinMode(heaters[1].heating_pin, OUTPUT);

  pinMode(RAMPS_D9_PIN, OUTPUT);
}
/**
 * set up the pins for each motor
 * Pins fits a CNCshieldV3.xx
 */
void motor_setup() {
  motors[0].step_pin=X_STEP_PIN;
  motors[0].dir_pin=X_DIR_PIN;
  motors[0].enable_pin=X_ENABLE_PIN;
  motors[0].max_pin = X_MAX_PIN;
  motors[0].min_pin = X_MIN_PIN;

  motors[1].step_pin=Y_STEP_PIN;
  motors[1].dir_pin=Y_DIR_PIN;
  motors[1].enable_pin=Y_ENABLE_PIN;
  motors[1].max_pin = Z_MAX_PIN;
  motors[1].min_pin = Z_MIN_PIN;

  motors[2].step_pin=Z_STEP_PIN;
  motors[2].dir_pin=Z_DIR_PIN;
  motors[2].enable_pin=Z_ENABLE_PIN;
  motors[2].max_pin = Y_MAX_PIN;
  motors[2].min_pin = Y_MIN_PIN;

  motors[3].step_pin=E1_STEP_PIN;
  motors[3].dir_pin=E1_DIR_PIN;
  motors[3].enable_pin=E1_ENABLE_PIN;
  //motors[3].max_pin = Y_MAX_PIN;
  //motors[3].min_pin = Y_MIN_PIN;

  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
    pinMode(motors[i].max_pin, INPUT_PULLUP);
    pinMode(motors[i].min_pin, INPUT_PULLUP);
  }

  for(int i=0;i<1;++i) {  
    // set the motor pin & scale
    //motor_calibration(i);
  }
  //AccelStepper motors[0] = AccelStepper(motorInterfaceType, X_STEP_PIN, X_DIR_PIN);

  attachInterrupt(digitalPinToInterrupt(X_MIN_PIN),Stop_Xmotor_Min,FALLING);
  attachInterrupt(digitalPinToInterrupt(X_MAX_PIN),Stop_Xmotor_Max,FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN),Stop_Ymotor_Min,FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN),Stop_Ymotor_Max,FALLING);
  //attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN),Stop_motors,FALLING);
  //attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN),Stop_motors,FALLING);
}

void motor_calibration(int motor){
  noInterrupts();
  Serial.println("Calibration start");
  digitalWrite(motors[motor].dir_pin,LOW);
  int loop = digitalRead(motors[motor].min_pin);
  while(loop != 0){
    digitalWrite(motors[motor].step_pin,HIGH);
    digitalWrite(motors[motor].step_pin,LOW);
    loop = digitalRead(motors[motor].min_pin);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  
  Serial.println("Calibration done");
  interrupts(); 
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

void Stop_Xmotor_Max(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[0].max_pin) == 0){
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void Stop_Xmotor_Min(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[0].min_pin) == 0){
    digitalWrite(motors[0].step_pin,HIGH);
    digitalWrite(motors[0].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void Stop_Ymotor_Max(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[1].max_pin) == 0){
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void Stop_Ymotor_Min(){
  noInterrupts();
  for(int i=0;i<NUM_AXIES;++i) {
    if (digitalRead(motors[i].dir_pin) == 0){
      digitalWrite(motors[i].dir_pin,HIGH);
    }
    else if (digitalRead(motors[i].dir_pin) == 1){
      digitalWrite(motors[i].dir_pin,LOW);
    }
  }
  while (digitalRead(motors[1].min_pin) == 0){
    digitalWrite(motors[1].step_pin,HIGH);
    digitalWrite(motors[1].step_pin,LOW);
    delayMicroseconds(MAX_FEEDRATE/5000);
  }
  position(0,0,0,0);
  line(0,0,0,0);
  intr_hap = 1;
  Serial.println("interrupt");
  interrupts();
}

void fan_turnon(){
  digitalWrite(RAMPS_D9_PIN, HIGH);
}

void fan_turnoff(){
  digitalWrite(RAMPS_D9_PIN, LOW);
}

// Read from thermistor
void Thermistor(int heater){
  //float filtered = adcFilter2.filter(analogRead(heaters[heater].therm_pin));
  float filtered = analogRead(heaters[heater].therm_pin);
  float R1 = 4700.;   // This resistor is fixed on the RAMPS PCB.
  float Ro = 100000.; // This is from the thermistor spec sheet
  float beta = 3950.; // This is from the thermistor spec sheet
  float To = 298.15;     // This is from the thermistor spec sheet
  float R;
  float Temp;
  R  = R1 / ((1023. / filtered) - 1.);
  Temp = 1 / ((1 / beta) * log(R / Ro) + 1 / To);
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  heaters[heater].temp = Temp; // deg C
}

void Temp_control(int heater){
  Thermistor(heater);
  if (heaters[heater].temp > heaters[heater].reach_temp) {
    digitalWrite(heaters[heater].heating_pin, LOW);
    Serial.print("off");
  } else if (heaters[heater].temp < heaters[heater].reach_temp) {
    digitalWrite(heaters[heater].heating_pin, HIGH);
    Serial.print("on");
  }
  Serial.println(heaters[heater].temp);
}

void Temp_control_wait(int heater){
  bool loop = 1, higher;
  Thermistor(heater);
  if (heaters[heater].temp > heaters[heater].reach_temp) {
    higher = 0;
    digitalWrite(heaters[heater].heating_pin, LOW);
  } else if (heaters[heater].temp < heaters[heater].reach_temp) {
    higher = 1;
    digitalWrite(heaters[heater].heating_pin, HIGH);
  }
  while(loop == 1){
    Thermistor(heater);
    if ((higher == 0)&&(heaters[heater].temp < heaters[heater].reach_temp)) {
      loop = 0;
    } else if ((higher == 1)&&(heaters[heater].temp > heaters[heater].reach_temp)) {
      loop = 0;
    }
    Serial.println(heaters[heater].temp);
    delay(2000);
  }
  Serial.println("Control goal reached");
}

void setup() {
  Serial.println("code Started");
  noInterrupts();
  Serial.begin(BAUD);  // open coms
  Serial.println();
  Serial.println("started");
  Serial.println();
  motor_setup();
  motor_enable();

  heater_setup();
  

  where();  // for debugging purposes
  help();  // say hello
  //position(0,0,0);  // set starting position
  feedrate(5);  // set default speed
  delay(1000);
  interrupts();
  //Serial.println();
  
  cmd_line = SD_card.readActiveLine();
  //Serial.println(cmd_line);
  //SD_card.printDirectory(SD_card.root,0);
  //SD_card.readFromSD();
}


void loop() {
  // listen for serial commands
  //while(Serial.available() > 0) {  // if something is available
    //Serial.print(cmd_line);
    //while(cmd_line != "EOF"){
    //while(cmd_line != "EOF"){
    cmd=Serial.readStringUntil('\n');  // get it
    if (cmd != ""){
      Serial.println(cmd);  // repeat it back so I know you got the message
      processCommand(cmd);  // do something with the command
      //cmd_line = SD_card.readActiveLine();
    }
    delay(400);
    //}
  //}
}
