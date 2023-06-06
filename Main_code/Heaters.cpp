#include "Heaters.h"

//U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 23, 17, 16);
Heater heaters[2];
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
  pinMode(heaters[1].therm_pin, INPUT_PULLUP);
  pinMode(heaters[1].heating_pin, OUTPUT);

  pinMode(RAMPS_D9_PIN, OUTPUT);
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

void Temp_control_wait(U8G2_ST7920_128X64_F_SW_SPI u8g2, int heater){
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
    printToLCD(u8g2, "T= " + String(heaters[0].temp)+ " C",0,48);
    delay(2000);
  }
  Serial.println("Control goal reached");
}

void printToLCD(U8G2_ST7920_128X64_F_SW_SPI u8g2, String ki, int x, int y)
{
    /*int str_len = ki.length() + 1;
    char char_array[str_len];
    ki.toCharArray(char_array, str_len);
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvR08_tr);
      u8g2.drawStr(x,y,char_array);
    } while ( u8g2.nextPage() );
    */
    u8g2.setDrawColor(0);
    u8g2.drawBox(x,y,128,y+24);
    u8g2.setDrawColor(1);
    u8g2.setCursor(x, y);
    u8g2.print(ki);
    u8g2.updateDisplay(); 
}