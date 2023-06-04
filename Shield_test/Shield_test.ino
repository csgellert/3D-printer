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



#define motorInterfaceType 1
// Create two new instances of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, X_STEP_PIN, X_DIR_PIN);
float Temp_C  = 0;
double step = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(RAMPS_D10_PIN, OUTPUT);
  pinMode(RAMPS_D9_PIN, OUTPUT);
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1200);
  attachInterrupt(digitalPinToInterrupt(X_MIN_PIN),Interrupt1,FALLING);
  attachInterrupt(digitalPinToInterrupt(X_MAX_PIN),Interrupt1,FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN),Interrupt1,FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN),Interrupt1,FALLING);
  attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN),Interrupt1,FALLING);
  attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN),Interrupt1,FALLING);
  digitalWrite(X_ENABLE_PIN, LOW);

  //digitalWrite(RAMPS_D10_PIN, HIGH);
  //digitalWrite(RAMPS_D9_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  //position1(step);
  step = step + 3000;
  delay(1000);
  //emp_C = Thermistor(TEMP_0_PIN);        // read temperature from thermistor
  Serial.println(digitalRead(X_MIN_PIN));
  //Serial.println(digitalRead(X_MAX_PIN));
  
  //stepper1.setSpeed(-300);
  //stepper1.runSpeed(); 
  //Serial.println(stepper1.currentPosition());
}

void Interrupt1 (){
  Serial.println("Interrupt1");
}

void position1(double k)
{
  stepper1.moveTo(k);
  stepper1.runToPosition();
}

float Thermistor(int pin){
  float filtered = adcFilter2.filter(analogRead(pin));
  float R1 = 4700.;   // This resistor is fixed on the RAMPS PCB.
  float Ro = 100000.; // This is from the thermistor spec sheet
  float beta = 3950.; // This is from the thermistor spec sheet
  float To = 298.15;     // This is from the thermistor spec sheet
  float R;
  float Temp;
  R  = R1 / ((1023. / filtered) - 1.);
  Temp = 1 / ((1 / beta) * log(R / Ro) + 1 / To);
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  return Temp; // deg C
}
