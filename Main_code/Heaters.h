#ifndef HEATERS_H
#define HEATERS_H

//#include <U8g2lib.h>     /* LCD */
#include "Arduino.h"
#include "SD_process.h"
#include "Constants.h"
#include "Dynamics.h"
#include <U8g2lib.h>     /* LCD */

//extern U8G2_ST7920_128X64_F_SW_SPI u8g2();

typedef struct {
  int therm_pin;
  int heating_pin;
  float temp;
  float reach_temp;
} Heater;

void heater_setup();
void fan_turnon();
void fan_turnoff();
void Thermistor(int heater);
void Temp_control(int heater);
void Temp_control_wait(U8G2_ST7920_128X64_F_SW_SPI u8g2, int heater);
void printToLCD(U8G2_ST7920_128X64_F_SW_SPI u8g2, String ki, int x, int y);

#endif