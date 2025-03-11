#ifndef CONTROL_H
#define CONTROL_H

#define LEFT_A 13
#define LEFT_B 14
#define RIGHT_A 15
#define RIGHT_B 16

#define time_ms 750

#define CALIBRATION_FACTOR 1.27

// TODO: figure out suitable initial values
// float P_GAIN = 1;
// float I_GAIN = 1;

void moveForward(int speed);

void moveCustom(int left_speed, int right_speed);

void turnRight();

void turnLeft();

void moveBackward(int speed);

void stop();

void overcomeStaticFriction();

#endif