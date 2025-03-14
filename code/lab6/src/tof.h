#ifndef TOF_H
#define TOF_H

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#define SHUTDOWN_PIN 4
#define INTERRUPT_PIN 3 // Does not matter

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void setupTOF(void);

// blocking function to read from TOF1
void blockReadTOF1(int * res);

// blocking function to read from TOF2
void blockReadTOF2(int * res); 

// non-blocking function to read from TOF1
// returns 0 on successful reads, 1 otherwise
int nonBlockReadTOF1(int* res);

// non-blocking function to read from TOF2
// returns 0 on successful reads, 1 otherwise
int nonBlockReadTOF2(int* res);

#endif

