#ifndef CONTROL_H
#define CONTROL_H

#define LEFT_A 3
#define LEFT_B 5
#define RIGHT_A 14
#define RIGHT_B 15

#define time_ms 750

#define CALIBRATION_FACTOR 1.27

#define RIGHT_TURN_CALIBRATION_FACTOR 1.67

#define LEFT_TURN_CALIBRATION_FACTOR 1.67

// TODO: figure out suitable initial values
// float P_GAIN = 1;
// float I_GAIN = 1;

void moveForward(int speed);

void moveCustom(int left_speed, int right_speed);

void turnRight(int left_speed, int right_speed);

void turnLeft(int left_speed, int right_speed);

void moveBackward(int speed);

void stop();

void overcomeStaticFriction();

#endif