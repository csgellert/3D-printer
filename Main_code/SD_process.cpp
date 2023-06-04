#include "SD_process.h"



SD_process::SD_process(/* args */)
{
    if (!SD.begin(this->chipSelect)) {
        Serial.println("initialization failed. Things to check:");
        while (true);
    }
  this->finished = false;
  Serial.println("initialization done.");
  this->root = SD.open("/");
}

SD_process::~SD_process()
{
    if (this->dataFile) {  this->dataFile.close();  Serial.println("Destructor completed");}
}

void SD_process::printDirectory(File dir, int numTabs) {
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

bool SD_process::isGcode(const char* filename) {
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
void SD_process::readFromSD(){
  this->dataFile = SD.open("test.txt");
  // if the file is available, write to it:
  if (this->dataFile) {
    while (this->dataFile.available()) {
      Serial.println(dataFile.readStringUntil('\n'));
    }
    this->dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
String SD_process::readActiveLine(){
  if (this->finished == 0){
    String sor;
    if (! this->dataFile) {
        this->dataFile = SD.open("test.txt");
    }
    // if the file is available, write to it:
    if (this->dataFile) {
        sor = dataFile.readStringUntil('\n');
    }
    // if the file isn't open, pop up an error:
    else {
        sor = "error opening datalog.txt";
    }
    if(! this->dataFile.available()){
        this->finished = 1;
        this->dataFile.close();
    }
    return sor;
  }
  else return "EOF";
}
