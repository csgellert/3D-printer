/*

  Listfiles

  This example shows how print out the files in a

  directory on a SD card.Pin numbers reflect the default

  SPI pins for Uno and Nano models

  The circuit:

   SD card attached to SPI bus as follows:

 ** SDO - pin 11

 ** SDI - pin 12

 ** CLK - pin 13

 ** CS - depends on your SD card shield or module.

        Pin 10 used here for consistency with other Arduino examples

    (for MKRZero SD: SDCARD_SS_PIN)

  created   Nov 2010

  by David A. Mellis

  modified 9 Apr 2012

  by Tom Igoe

  modified 2 Feb 2014

  by Scott Fitzgerald

  modified 24 July 2020

  by Tom Igoe



  This example code is in the public domain.

*/
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>     /* LCD */
#include "SD_process.h"

//const int chipSelect = 4;
U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, 3, 1, 0);
//File root;

void setup() {

 // Open serial communications and wait for port to open:

  Serial.begin(9600);
  u8g2.begin();
  // wait for Serial Monitor to connect. Needed for native USB port boards only:

  while (!Serial);

  Serial.print("Initializing SD card...");
  SD_process SD_card;
  SD_card.printDirectory(SD_card.root,0);
  //SD_card.readFromSD();
  //Serial.print(SD_card.readActiveLine());
  //Serial.print(SD_card.readActiveLine());
  //Serial.print(SD_card.readActiveLine());
  //Serial.print(SD_card.readActiveLine());
  //enum {BufSize=24}; // If a is short use a smaller number, eg 5 or 6 
  // char buf[BufSize];
  //  sprintf (buf, BufSize,SD_card.readActiveLine());
  //char *forma = SD_card.readActiveLine().c_str();
}

void loop() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,24,"aaaa");
    //u8g2.print(SD_card.readActiveLine());
    //u8g2.print(SD_card.readActiveLine());
  }while ( u8g2.nextPage() );
  // Put in a slight delay to help debounce the reading
  delay(10);
  // nothing happens after setup finishes.
}
/*
void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    if (entry.isDirectory()) {
      Serial.print(entry.name());
      Serial.println("/");
    } else {
        if (isGcode(entry.name()))
      {
        for (uint8_t i = 0; i < numTabs; i++) {
          Serial.print('\t');
        }
        Serial.print(entry.name());
      }
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();

  }
}
bool isGcode(const char* filename) {
  int8_t len = strlen(filename);
  bool result;
  String filen(filename); 
  //Serial.println(filen);
  int idx = filen.indexOf('.');
  result = false;
  if (idx > -1)
  {
    String exte = filen.substring(idx);
    exte.toLowerCase();
    //Serial.println(exte);
    result = exte==".txt";
  }
  return result;
}
void readFromSD(){
  File dataFile = SD.open("test.txt");
  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.println(dataFile.readStringUntil('\n'));
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}*/
