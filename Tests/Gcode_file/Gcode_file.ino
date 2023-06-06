
#include <SD.h>

#define PIN_SPI_CS 4

File file;
char buf1[] = ";TIME:3373\nM82 ;absolute extrusion mode\nM17 ;command\nG92 E0\n;sajt\nG92 E0\nG04 P1\nG90\nM18\n;comment";
char buf2[] = "M82 \nM17\nG04 P1\nG90\nM18";
String sbuf(buf1);


void setup() {
  Serial.begin(9600);
  //Serial.print("\n \n");
  //Serial.println(sbuf);
  Serial.print("\n \n");
  //String file = CutComments(sbuf);
  Serial.println(sbuf);
  Serial.print("Number of lines: ");
  Serial.println(CountLines(sbuf));
}

void loop() {
}

String CutComments(String buf){
  int index_of_enter;
  String nbuf = "";
  int index_of_comma = buf.indexOf(';');
  nbuf.concat(buf.substring(0, index_of_comma));
  while (index_of_comma != -1){
    index_of_enter = buf.indexOf("\n",index_of_comma);
    index_of_comma = buf.indexOf(';',index_of_enter);
    nbuf.concat(buf.substring(index_of_enter, index_of_comma-1));
  }
  return nbuf;
}

int CountLines(String buf){
  int count = 0, index_of_enter = 0, temp = 0;
  while (index_of_enter != -1){
    index_of_enter = buf.indexOf("\n",temp+1);
    count++;
    temp = index_of_enter;
  }
  return count;
}

/*
String MakeArray(String buf){
  int lines = CountLines(buf), count = 0, index_of_enter = 0, temp = 0;
  String array[lines];
  for (int i = 0; i<lines; i++){
    index_of_enter = buf.indexOf("\n",temp+1);
    array[i] = buf.substring(temp, index_of_enter);
    temp = index_of_enter;
  }
  return array;
}
*/

