#include "control.h"

void moveForward() {
  analogWrite(LEFT_A, 95 * CALIBRATION_FACTOR);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 95);
  delay(time_ms);
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 0);  
}

void turnRight() {
  analogWrite(LEFT_A, 150 * CALIBRATION_FACTOR);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 95);
  delay(time_ms);
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 0);
}

void turnLeft() {
  analogWrite(LEFT_A, 95 * CALIBRATION_FACTOR);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 150);
  delay(time_ms);
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 0);
}

void moveBackward() {
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 95 * CALIBRATION_FACTOR);
  analogWrite(RIGHT_A, 95);
  analogWrite(RIGHT_B, 0);
  delay(time_ms);
  analogWrite(LEFT_A, 0);
  analogWrite(LEFT_B, 0);
  analogWrite(RIGHT_A, 0);
  analogWrite(RIGHT_B, 0);
}