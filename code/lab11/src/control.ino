#include "control.h"

// Try PI initially
// void forwardControlled(int tof_target, ) {
//   static int startTime = millis();

//   // TODO: read from TOF
//   err = NULL;

//   P_GAIN * err

//
// }

void moveForward(int speed)
{
  analogWrite(LEFT_A, speed);
  analogWrite(LEFT_B, 1);
  analogWrite(RIGHT_A, 1);
  if (speed * CALIBRATION_FACTOR > 255)
  {
    analogWrite(RIGHT_B, 255);
  }
  else
  {
    analogWrite(RIGHT_B, speed * CALIBRATION_FACTOR);
  }
}

void moveCustom(int left_speed, int right_speed)
{
  analogWrite(LEFT_A, left_speed);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, right_speed);
  Serial.println("Moving custom");
  Serial.println(left_speed);
  Serial.println(right_speed);
}

void turnRight(int left_speed, int right_speed)
{
  analogWrite(LEFT_A, left_speed);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, right_speed);
  analogWrite(RIGHT_B, 0);
}

void turnLeft(int left_speed, int right_speed)
{
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, left_speed);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, right_speed);
}

void moveBackward(int speed)
{
  if (speed * CALIBRATION_FACTOR > 255)
  {
    analogWrite(RIGHT_A, 255);
  }
  else
  {
    analogWrite(RIGHT_A, speed * CALIBRATION_FACTOR);
  }
  analogWrite(RIGHT_B, 1);
  analogWrite(LEFT_A, 1);
  analogWrite(LEFT_B, speed);
}

void moveBackwardDelay(int speed, int left_delay)
{
  if (speed * CALIBRATION_FACTOR > 255)
  {
    analogWrite(RIGHT_A, 255);
  }
  else
  {
    analogWrite(RIGHT_A, speed * CALIBRATION_FACTOR);
  }
  analogWrite(RIGHT_B, 1);
  delay(left_delay);
  analogWrite(LEFT_A, 1);
  analogWrite(LEFT_B, speed);
}

void stop()
{
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 0);
}

void overcomeStaticFriction()
{
  analogWrite(LEFT_A, 255);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 255);
  delay(100);
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 0);
}