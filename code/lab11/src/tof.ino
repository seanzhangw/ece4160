
#include "tof.h"

void setupTOF(void)
{
  Wire.begin();
  Serial.println("VL53L1X Qwiic Test");
  pinMode(SHUTDOWN_PIN, OUTPUT);

  // Set the address of TOF1 to 0xf5
  digitalWrite(SHUTDOWN_PIN, LOW); // Shut down TOF2
  distanceSensor1.setI2CAddress(0xf5);
  digitalWrite(SHUTDOWN_PIN, HIGH); // Restart TOF2

  while (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("TOF1 failed to begin. Please check wiring. Freezing...");
  }

  while (distanceSensor2.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("TOF2 failed to begin. Please check wiring. Freezing...");
  }
  Serial.println("TOF sensors online!");

  
  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
}

void blockReadTOF1(int *res)
{
  while (!distanceSensor1.checkForDataReady())
  {
    delay(1);
  }
  int distance1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();
  *res = distance1;
  distanceSensor1.startRanging();
}

void blockReadTOF2(int *res)
{
  while (!distanceSensor2.checkForDataReady())
  {
    delay(1);
  }
  int distance2 = distanceSensor2.getDistance();
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();
  *res = distance2;
  distanceSensor2.startRanging();
}

int nonBlockReadTOF1(int *res)
{
  if (distanceSensor1.checkForDataReady())
  {
    int distance1 = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    *res = distance1;
    distanceSensor1.startRanging();
    return 0;
  }
  return 1;
}

int nonBlockReadTOF2(int *res)
{
  if (distanceSensor2.checkForDataReady())
  {
    int distance2 = distanceSensor2.getDistance();
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();
    *res = distance2;
    distanceSensor2.startRanging();
    return 0;
  }
  return 1;
}
