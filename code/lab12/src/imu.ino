/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/

#include "imu.h"

// #define USE_SPI       // Uncomment this to use SPI

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// for 5Hz cutoff frequency
#define LOWPASS_ALPHA 0.13825904214
#define COMPLIMENTARY_ALPHA 0.8

// these should be adjusted by our teleop system
float GYRO_PITCH_CONVERSION = 3.8;
float GYRO_ROLL_CONVERSION = 3.8;
float GYRO_YAW_CONVERSION = 3.8;

// these should be adjusted by our teleop system
float ACCEL_PITCH_CONVERSION = 1;
// might need to make negative
float ACCEL_ROLL_CONVERSION = 1;

// these should be changed by our teleop system
// might need to be negative
float ACCEL_PITCH_OFFSET = 0;
float ACCEL_ROLL_OFFSET = 0;

void setupIMU()
{
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(500);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(500);
  }
  // wait for a second

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    // ============================= BEGIN: DMP SETUP =============================
    bool success = true;
    // Initialize the DMP
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // Enable the DMP Game Rotation Vector sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
    // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 2) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO queue
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (!success)
    {
      Serial.println("Enabling DMP failed!");
      while (1)
      {
        // Freeze
      }
    }
    // ============================= END: DMP SETUP =============================

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

float getAccelerometerPitch(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();
  float acc_x = sensor->accX();
  float acc_z = sensor->accZ();

  float pitch = atan2_custom(acc_x, acc_z) * ACCEL_PITCH_CONVERSION + ACCEL_PITCH_OFFSET;
  return addPadding(pitch, ACCEL_PITCH_OFFSET);
}

float getAccelerometerRoll(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();
  float acc_y = sensor->accY();
  float acc_z = sensor->accZ();
  float roll = atan2_custom(acc_y, acc_z) * ACCEL_ROLL_CONVERSION + ACCEL_ROLL_OFFSET;
  return addPadding(roll, ACCEL_ROLL_OFFSET);
}

void calibrateAccelerometer(float pitchConversion, float rollConversion, float pitchOffset, float rollOffset)
{
  ACCEL_PITCH_CONVERSION = pitchConversion;
  ACCEL_ROLL_CONVERSION = rollConversion;
  ACCEL_PITCH_OFFSET = pitchOffset;
  ACCEL_ROLL_OFFSET = rollOffset;
  Serial.println("Accelerometer calibrated");
}

float getGyroscopePitch(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();

  static float gyro_y = 0;
  static float pitch = 0;
  static long last_time = millis();

  gyro_y = sensor->gyrY() * GYRO_PITCH_CONVERSION;
  float dt = (millis() - last_time) / 1000.0;
  pitch += gyro_y * dt;
  last_time = millis();

  return pitch;
}

float getGyroscopeRoll(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();

  static float gyro_x = 0;
  static float roll = 0;
  static long last_time = millis();

  gyro_x = sensor->gyrX() * GYRO_ROLL_CONVERSION;
  float dt = (millis() - last_time) / 1000.0;
  roll += gyro_x * dt;
  last_time = millis();

  return roll;
}

float getGyroscopeYaw(ICM_20948_I2C *sensor)
{
  static float gyro_z = 0;
  static float yaw = 0;
  static long last_time = millis();

  if (millis() - last_time > 1000)
  {
    last_time = millis();
  }

  gyro_z = sensor->gyrZ() * GYRO_YAW_CONVERSION;
  float dt = (millis() - last_time) / 1000.0;
  yaw += gyro_z * dt;
  last_time = millis();

  return yaw;
}

float accelGyroPitch(ICM_20948_I2C *sensor)
{
  // NOTE: static variables means that these variables will retain their values between function calls
  // set initial time
  static long last_time = millis();

  if (millis() - last_time > 1000)
  {
    last_time = millis();
  }

  // get the gyro values
  float gyro_y = sensor->gyrY() * GYRO_PITCH_CONVERSION;

  // calculate the change in pitch, roll, and yaw
  float dt = (millis() - last_time) / 1000.0;
  // theta_g values
  static float gyroPitch = 0;
  gyroPitch += gyro_y * dt;

  // acceleramotor
  float acc_x = sensor->accX();
  float acc_z = sensor->accZ();

  float pitch = atan2_custom(acc_x, acc_z) * ACCEL_PITCH_CONVERSION + ACCEL_PITCH_OFFSET;

  pitch = addPadding(pitch, ACCEL_PITCH_OFFSET);

  static float lastAccelPitch = pitch;

  float filteredAccelPitch = LOWPASS_ALPHA * pitch + (1. - LOWPASS_ALPHA) * lastAccelPitch;
  lastAccelPitch = filteredAccelPitch;

  // complementary filter
  static float compFilteredPitch = 0;
  compFilteredPitch = (compFilteredPitch + gyroPitch) * (1 - COMPLIMENTARY_ALPHA) + COMPLIMENTARY_ALPHA * filteredAccelPitch;

  // update the last time
  last_time = millis();
  return compFilteredPitch;
}

float accelGyroRoll(ICM_20948_I2C *sensor)
{
  // NOTE: static variables means that these variables will retain their values between function calls
  // set initial time
  static long last_time = millis();

  if (millis() - last_time > 1000)
  {
    last_time = millis();
  }

  // get the gyro values
  float gyro_x = sensor->gyrX() * GYRO_ROLL_CONVERSION;

  // calculate the change in pitch, roll, and yaw
  float dt = (millis() - last_time) / 1000.0;
  // theta_g values
  static float gyroRoll = 0;

  gyroRoll += gyro_x * dt;

  // acceleramotor
  float acc_y = sensor->accY();
  float acc_z = sensor->accZ();

  float roll = atan2_custom(acc_y, acc_z) * ACCEL_ROLL_CONVERSION + ACCEL_ROLL_OFFSET;

  roll = addPadding(roll, ACCEL_ROLL_OFFSET);

  static float lastAccelRoll = roll;

  float filteredAccelRoll = LOWPASS_ALPHA * roll + (1. - LOWPASS_ALPHA) * lastAccelRoll;
  lastAccelRoll = filteredAccelRoll;

  // complementary filter
  static float compFilteredRoll = 0;
  compFilteredRoll = (compFilteredRoll + gyroRoll) * (1 - COMPLIMENTARY_ALPHA) + COMPLIMENTARY_ALPHA * filteredAccelRoll;

  // update the last time
  last_time = millis();
  return compFilteredRoll;
}

void gyroPitchYawRoll(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();
  // NOTE: static variables means that these variables will retain their values between function calls
  // set initial time
  static long last_time = millis();

  // if (millis() - last_time > 1000) {
  //   last_time = millis();
  // }

  // set pitch, roll, and yaw to 0 initially
  static float pitch = 0;
  static float roll = 0;
  static float yaw = 0;

  // get the gyro values
  float gyro_x = sensor->gyrX() * GYRO_ROLL_CONVERSION;
  float gyro_y = sensor->gyrY() * GYRO_PITCH_CONVERSION;
  float gyro_z = sensor->gyrZ() * GYRO_YAW_CONVERSION;

  // calculate the change in pitch, roll, and yaw
  float dt = (millis() - last_time) / 1000.0;
  // unfiltered
  pitch += gyro_y * dt;
  roll += gyro_x * dt;
  yaw += gyro_z * dt;

  // print the values
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(millis());
  Serial.println();

  // update the last time
  last_time = millis();
}

float getAccelPitchLowPass(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();

  float acc_x = sensor->accX();
  float acc_z = sensor->accZ();

  float pitch = atan2_custom(acc_x, acc_z) * ACCEL_PITCH_CONVERSION + ACCEL_PITCH_OFFSET;

  pitch = addPadding(pitch, ACCEL_PITCH_OFFSET);

  static float lastPitch = pitch;

  float pitchFiltered = LOWPASS_ALPHA * pitch + (1. - LOWPASS_ALPHA) * lastPitch;

  lastPitch = pitchFiltered;
  return pitchFiltered;
}

float getAccelRollLowPass(ICM_20948_I2C *sensor)
{
  sensor->getAGMT();

  float acc_y = sensor->accY();
  float acc_z = sensor->accZ();

  float roll = atan2_custom(acc_y, acc_z) * ACCEL_ROLL_CONVERSION + ACCEL_ROLL_OFFSET;
  roll = addPadding(roll, ACCEL_ROLL_OFFSET);
  static float lastRoll = roll;
  float rollFiltered = LOWPASS_ALPHA * roll + (1. - LOWPASS_ALPHA) * lastRoll;
  lastRoll = rollFiltered;
  return rollFiltered;
}

// // Below here are some helper functions to print the data nicely!

// void printPaddedInt16b(int16_t val)
// {
//   if (val > 0)
//   {
//     SERIAL_PORT.print(" ");
//     if (val < 10000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (val < 1000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (val < 100)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (val < 10)
//     {
//       SERIAL_PORT.print("0");
//     }
//   }
//   else
//   {
//     SERIAL_PORT.print("-");
//     if (abs(val) < 10000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (abs(val) < 1000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (abs(val) < 100)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (abs(val) < 10)
//     {
//       SERIAL_PORT.print("0");
//     }
//   }
//   SERIAL_PORT.print(abs(val));
// }

// void printRawAGMT(ICM_20948_AGMT_t agmt)
// {
//   SERIAL_PORT.print("RAW. Acc [ ");
//   printPaddedInt16b(agmt.acc.axes.x);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.acc.axes.y);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.acc.axes.z);
//   SERIAL_PORT.print(" ], Gyr [ ");
//   printPaddedInt16b(agmt.gyr.axes.x);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.gyr.axes.y);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.gyr.axes.z);
//   SERIAL_PORT.print(" ], Mag [ ");
//   printPaddedInt16b(agmt.mag.axes.x);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.mag.axes.y);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.mag.axes.z);
//   SERIAL_PORT.print(" ], Tmp [ ");
//   printPaddedInt16b(agmt.tmp.val);
//   SERIAL_PORT.print(" ]");
//   SERIAL_PORT.println();
// }

// void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
// {
//   float aval = abs(val);
//   if (val < 0)
//   {
//     SERIAL_PORT.print("-");
//   }
//   else
//   {
//     SERIAL_PORT.print(" ");
//   }
//   for (uint8_t indi = 0; indi < leading; indi++)
//   {
//     uint32_t tenpow = 0;
//     if (indi < (leading - 1))
//     {
//       tenpow = 1;
//     }
//     for (uint8_t c = 0; c < (leading - 1 - indi); c++)
//     {
//       tenpow *= 10;
//     }
//     if (aval < tenpow)
//     {
//       SERIAL_PORT.print("0");
//     }
//     else
//     {
//       break;
//     }
//   }
//   if (val < 0)
//   {
//     SERIAL_PORT.print(-val, decimals);
//   }
//   else
//   {
//     SERIAL_PORT.print(val, decimals);
//   }
// }

// #ifdef USE_SPI
// void printScaledAGMT(ICM_20948_SPI *sensor)
// {
// #else
// void printScaledAGMT(ICM_20948_I2C *sensor)
// {
// #endif
//   SERIAL_PORT.print("Scaled. Acc (mg) [ ");
//   printFormattedFloat(sensor->accX(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->accY(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->accZ(), 5, 2);
//   SERIAL_PORT.print("] Time : ");
//   Serial.print(millis());
//   Serial.println();
//   SERIAL_PORT.print(" ], Gyr (DPS) [ ");
//   printFormattedFloat(sensor->gyrX(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->gyrY(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->gyrZ(), 5, 2);
//   SERIAL_PORT.print(" ], Mag (uT) [ ");
//   printFormattedFloat(sensor->magX(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->magY(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->magZ(), 5, 2);
//   SERIAL_PORT.print(" ], Tmp (C) [ ");
//   printFormattedFloat(sensor->temp(), 5, 2);
//   SERIAL_PORT.print(" ]");
//   SERIAL_PORT.println();
// }

// BEGIN: HELPER FUNCTIONS
#define PI_ON_TWO (PI / 2)

float atan2_custom(float x, float y)
{
  if (x > 0.0)
  {
    return atan(y / x);
  }
  if (x < 0.0)
  {
    if (y >= 0.0)
    {
      return PI + atan(y / x);
    }
    else
    {
      return -PI + atan(y / x);
    }
  }
  if (y > 0.0)
  { // x == 0
    return PI_ON_TWO;
  }
  if (y < 0.0)
  {
    return -PI_ON_TWO;
  }
  return 0.0; // Undefined
}

float addPadding(float val, float OFFSET)
{
  if (OFFSET > 1)
  {
    if (180 < val && val < 360 - OFFSET)
    {
      return -360 + val;
    }
  }
  else if (OFFSET < 1)
  {
    if (-180 > val && val > -(360 - OFFSET))
    {
      return 360 + val;
    }
  }
  return val;
}
