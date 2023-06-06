#include <AccelStepper.h>
#include <Ewma.h> // exponential filter
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>     /* LCD */
#include "SD_process.h"
#include "Constants.h"
#include "Heaters.h"
#include "Dynamics.h"

Ewma adcFilter2(0.05);
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 23, 17, 16);

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern Axis a[NUM_AXIES];
extern Axis atemp;
extern Motor motors[NUM_AXIES];
extern Heater heaters[2];

SD_process SD_card;
bool intr_hap = 0;
// speeds
float fr=0;  // human version
long step_delay;  // machine version
float px,py,pz,pe;  // position
// settings
char mode_abs=1;  // absolute mode?

String cmd_line, cmd;

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
    Temp_control_wait(u8g2,0);
    break;
  }
  case 140: {  // set bed temperature
    heaters[1].reach_temp = parseNumber('S',0,command);
    Temp_control(1);
    break;
  }
  case 190: {  // set bed temperature and wait
    heaters[1].reach_temp = parseNumber('S',0,command);
    Temp_control_wait(u8g2, 1);
    break;
  }
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
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
  SD_card.Startup();
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setFontPosTop();
  //cmd_line = SD_card.readActiveLine();
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
      printToLCD(u8g2,cmd,0,24);
      processCommand(cmd);  // do something with the command
      //cmd_line = SD_card.readActiveLine();
    }
    Temp_control(0);
    printToLCD(u8g2, "T= " + String(heaters[0].temp)+ " C",0,48);
    //Serial.print(String(heaters[0].temp));
    delay(400);
    //}
  //}
}
