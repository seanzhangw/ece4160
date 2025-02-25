#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"

extern ICM_20948_I2C myICM;

// void gyroPitchYawRoll(ICM_20948_I2C * sensor);
void setupIMU();
// Function prototypes
float getAccelerometerPitch(ICM_20948_I2C *sensor);
float getAccelerometerRoll(ICM_20948_I2C *sensor);
float getAccelPitchLowPass(ICM_20948_I2C *sensor);
float getAccelRollLowPass(ICM_20948_I2C *sensor);

void calibrateAccelerometer(
    const float pitchConversion,
    const float rollConversion,
    const float pitchOffset,
    const float rollOffset);

float getGyroscopePitch(ICM_20948_I2C *sensor);
float getGyroscopeRoll(ICM_20948_I2C *sensor);
float getGyroscopeYaw(ICM_20948_I2C *sensor);

float accelGyroPitch(ICM_20948_I2C *sensor);
float accelGyroRoll(ICM_20948_I2C *sensor);

#endif // LAB2_H
