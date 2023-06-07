#ifndef CONSTANTS_H
#define CONSTANTS_H


#include "Arduino.h"
#include "SD_process.h"
#include "Heaters.h"
#include "Dynamics.h"

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

// Servos
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

#endif