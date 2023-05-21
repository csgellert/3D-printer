
#include <SD.h>

#define PIN_SPI_CS 4

File file;
char buf[] = "M17\nG04 P1\nG90\nM18";
String sbuf(buf);
//String test_buffer[] = {"M","1","7","\n","G","0","4"," ","P","1","\n","G","9","0"};

void setup() {
  Serial.begin(9600);
  Serial.print("\n \n");
  Serial.println(sbuf);
  CountLines(sbuf);
}

void loop() {
}

int CountLines(String buf){
  int size = sizeof(buf);
  int count = 0;
  int index_of_char = text.indexOf(code);
  if (index_of_char != -1) {
    int index_of_space = text.indexOf(" ",index_of_char);
    String out = text.substring(index_of_char+1, index_of_space);
    return out.toFloat();
  }
  Serial.print("size:");
  Serial.println(size); 
  for (int i = 0; i < size; i++){
    if (buf[i] == "\n"){
      count++;
    }
  }
  Serial.print("count:");
  Serial.println(count); 
}


String ReadLine(char buf,int line){


}