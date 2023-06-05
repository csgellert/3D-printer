#ifndef SD_PROCESS_H
#define SD_PROCESS_H

#include "Arduino.h"
#include <SD.h>

class SD_process
{
private:
    /* data */
    const int chipSelect = 53;
    //char * active_file = "test.txt";
    File dataFile;
public:
    File root;
    SD_process(/* args */);
    ~SD_process();
    void printDirectory(File dir, int numTabs);
    bool isGcode(const char* filename);
    bool finished;
    void readFromSD();
    String readActiveLine();
    void SD_process::Startup();
};

#endif
