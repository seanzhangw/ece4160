#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <stdlib.h>
#include "tof.h"
#include "control.h"
#include "imu.h"
#include <ArduinoBLE.h>

/* ==================== BEGIN: BLUETOOTH DEFINES ============================ */
//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "6d0b658d-0be2-4ddf-818e-897b0fdf98fb"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

char buffer[50];
/* ==================== END: BLUETOOTH DEFINES ============================== */

// ==================== BEGIN: YAW TIMER SETUP ============================== */
double yaw;
// ==================== END: YAW TIMER SETUP ================================ */

#define RESOLUTION_BITS 16
#define DATA_BUFFER_SIZE 3000

// global variables for storing tof measurements
int distance1;
int distance2;

// struct for storing tof data
struct TOFPacket
{
  long time;
  int distance1;
  int distance2;
};

TOFPacket TOFDataBuffer[DATA_BUFFER_SIZE];

// struct for storing control data
struct ControlPacket
{
  long time;
  int speed;
  int P_err;
  int I_err;
  int tof_distance;
  int dt;
};

ControlPacket ControlDataBuffer[DATA_BUFFER_SIZE];

bool tofAvailable[DATA_BUFFER_SIZE];

struct YawControlPacket
{
  long time;
  int speed;
  int P_err;
  double D_err;
  double yaw;
  int dt;
};

YawControlPacket YawControlDataBuffer[DATA_BUFFER_SIZE];

/*
 Log data to the buffer for a control packet
*/
void logControlData(int i, long time, int speed, int P_err, int I_err, int tof_distance, int dt)
{
  ControlDataBuffer[i].time = time;
  ControlDataBuffer[i].speed = speed;
  ControlDataBuffer[i].P_err = P_err;
  ControlDataBuffer[i].I_err = I_err;
  ControlDataBuffer[i].tof_distance = tof_distance;
  ControlDataBuffer[i].dt = dt;
}

/*
 Send control data to the BLE characteristic
*/
void sendControlData(int i)
{
  tx_estring_value.clear();
  sprintf(buffer, "%ld", ControlDataBuffer[i].time);
  tx_estring_value.append(buffer);
  tx_estring_value.append("|");
  tx_estring_value.append(ControlDataBuffer[i].speed);
  tx_estring_value.append("|");
  tx_estring_value.append(ControlDataBuffer[i].P_err);
  tx_estring_value.append("|");
  tx_estring_value.append(ControlDataBuffer[i].I_err);
  tx_estring_value.append("|");
  tx_estring_value.append(ControlDataBuffer[i].tof_distance);
  tx_estring_value.append("|");
  tx_estring_value.append(ControlDataBuffer[i].dt);
  tx_estring_value.append("|");
  tx_estring_value.append(tofAvailable[i]);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

/*
  Read the YAW value from the FIFO. This function needs to be called at a high
  frequency to avoid overflowing the FIFO and to get the most recent YAW value.
*/
void readDMPYaw(double &yaw)
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  // Is valid data available?
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    // We have asked for GRV data so we should receive Quat6
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      double q_x = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q_y = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q_z = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q_w = sqrt(1.0 - ((q_x * q_x) + (q_y * q_y) + (q_z * q_z)));

      double siny_cosp = 2 * (q_w * q_z + q_x * q_y);
      double cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z);

      if (cosy_cosp != 0)
      {
        yaw = std::atan2(siny_cosp, cosy_cosp) * 180 / M_PI;
      }

      if (yaw < 0)
      {
        yaw = (double)360 + yaw;
      }
    }
  }
}

/*
 Send Yaw Control Data to the BLE Characteristic
*/
void sendYawControlData(int i)
{
  tx_estring_value.clear();
  sprintf(buffer, "%ld", YawControlDataBuffer[i].time);
  tx_estring_value.append(buffer);
  tx_estring_value.append("|");
  tx_estring_value.append(YawControlDataBuffer[i].speed);
  tx_estring_value.append("|");
  tx_estring_value.append(YawControlDataBuffer[i].P_err);
  tx_estring_value.append("|");
  tx_estring_value.append(YawControlDataBuffer[i].D_err);
  tx_estring_value.append("|");
  tx_estring_value.append(YawControlDataBuffer[i].yaw);
  tx_estring_value.append("|");
  tx_estring_value.append(YawControlDataBuffer[i].dt);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.println("SENT STRING: ");
  Serial.println(tx_estring_value.c_str());

  Serial.println(yaw);
}
// global tracker for buffer index
int dataIndex = 0;

// =========================== BEGIN: PID TOF CONTROL ==========================
// adjustable from jupyter
float P_GAIN;
float I_GAIN;
int target_tof;
int MAX_SPEED;

// control variables
int err = 0;
int integral_err = 0;
int speed = 0;
int last_time;
int start_time;
int dt = 0;

int timingBudget;
int left_speed;
int right_speed;
int delay_time;

int last_tof;
int last_last_tof;
int time_between_tof;
int last_tof_time;

// enum for direction
enum Direction
{
  Still = 0,
  Forward = 1,
  Backward = 2,
};

// track last direction. Changing direction requires greater pwm output to overcome static friction
Direction lastDirection = Still;

#define BACKWARD_MULTIPLIER 6

#define STATIC_TO_FORWARD_SPEED 240
#define STATIC_TO_FORWARD_DELAY 140

// =========================== END: PID TOF CONTROL ==========================

// =========================== BEGIN: PID ORIENTATION CONTROL =================

// flag for determining if we want to start yaw control
int CONTROL_YAW = 0;
int RECORD_PID = 0;

// updated for calcs
long current_time;
long prev_time;

// changeable during runtime
int MAX_CLOCKWISE_SPEED = 80;
int MIN_CLOCKWISE_SPEED = 40;
int MAX_COUNTER_CLOCKWISE_SPEED = 80;
int MIN_COUNTER_CLOCKWISE_SPEED = 40;
int target_angle;

// derivative terms
double prev_yaw_err = 0.;
double yaw_err_d = 0.;
double filtered_yaw_err_d = 0.;
float D_GAIN;
float alpha = 0.1;

double yaw_err;

// =========================== END: PID ORIENTATION CONTROL =================

enum CommandTypes
{
  PING,
  GET_TOF1,
  GET_TOF2,
  GET_TOF_300,
  FORWARD_CONTROLLED,
  FORWARD_EXTRAPOLATE,
  FORWARD,
  BACKWARD,
  YAW_CONTROL,
  TURN_RIGHT,
  TURN_LEFT,
  RECORD_PID_DATA,
  SEND_PID_DATA,
  STOP
};

void handle_command()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(), rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success)
  {
    return;
  }
  int time;

  // Handle the command type accordingly
  switch (cmd_type)
  {
  case PING:
    tx_estring_value.clear();
    tx_estring_value.append("PONG");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.print("Sent back: ");
    Serial.println(tx_estring_value.c_str());
    break;
  case GET_TOF1:
    blockReadTOF1(&distance1);
    tx_estring_value.clear();
    tx_estring_value.append(distance1);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.print("Sent back distance:");
    Serial.println(tx_estring_value.c_str());
    break;
  case GET_TOF2:
    blockReadTOF2(&distance2);

    tx_estring_value.clear();
    tx_estring_value.append(distance2);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.print("Sent back distance:");
    Serial.println(tx_estring_value.c_str());
    break;
  case GET_TOF_300:
    success = robot_cmd.get_next_value(timingBudget);
    if (!success)
      return;

    distanceSensor1.setTimingBudgetInMs(timingBudget);
    distanceSensor2.setTimingBudgetInMs(timingBudget);

    dataIndex = 0;
    blockReadTOF2(&distance2);
    while (dataIndex < DATA_BUFFER_SIZE)
    {
      blockReadTOF1(&distance1);
      blockReadTOF2(&distance2);
      TOFDataBuffer[dataIndex].distance1 = distance1;
      TOFDataBuffer[dataIndex].distance2 = distance2;
      TOFDataBuffer[dataIndex].time = millis();
      dataIndex++;
    }
    for (int i = 0; i < dataIndex; i++)
    {
      tx_estring_value.clear();
      sprintf(buffer, "%ld", TOFDataBuffer[i].time);
      tx_estring_value.append(buffer);
      tx_estring_value.append("|");
      tx_estring_value.append(TOFDataBuffer[i].distance1);
      tx_estring_value.append("|");
      tx_estring_value.append(TOFDataBuffer[i].distance2);

      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());
    }
    Serial.println("TOF handler end");
    break;
  case FORWARD_CONTROLLED:
    success = robot_cmd.get_next_value(P_GAIN);
    if (!success)
      return;
    success = robot_cmd.get_next_value(I_GAIN);
    if (!success)
      return;
    success = robot_cmd.get_next_value(MAX_SPEED);
    if (!success)
      return;
    success = robot_cmd.get_next_value(target_tof);
    if (!success)
      return;
    success = robot_cmd.get_next_value(timingBudget);
    if (!success)
      return;
    success = robot_cmd.get_next_value(delay_time);
    if (!success)
      return;

    tx_estring_value.clear();

    distanceSensor1.setTimingBudgetInMs(timingBudget);

    integral_err = 0;

    lastDirection = Still;
    start_time = millis();
    last_time = millis();
    while (millis() - start_time < delay_time)
    {
      if (nonBlockReadTOF1(&distance1) == 0)
      {
        unsigned long current_time = millis();
        dt = current_time - last_time;
        last_time = current_time;

        err = distance1 - target_tof;
        // Wind-up protection
        if (abs(err) < 20)
        {
          integral_err = 0;
        }
        else
        {
          integral_err = integral_err + (err * dt);
        }
        speed = (int)(err * P_GAIN + integral_err * I_GAIN);

        if (speed > MAX_SPEED)
        {
          speed = MAX_SPEED;
        }
        if (speed < -MAX_SPEED)
        {
          speed = -MAX_SPEED;
        }
        if (speed > 0)
        {
          if (lastDirection == Still)
          {
            moveForward(STATIC_TO_FORWARD_SPEED);
            delay(STATIC_TO_FORWARD_DELAY);
          }

          moveForward(speed);
          lastDirection = Forward;
        }
        else
        {
          moveBackward(-BACKWARD_MULTIPLIER * speed);
          lastDirection = Backward;
        }
        logControlData(dataIndex, current_time, speed, err, integral_err, distance1, dt);
        dataIndex++;
      }
    }

    // stop the motor
    stop();

    for (int i = 0; i < dataIndex; i++)
    {
      sendControlData(i);
    }
    dataIndex = 0;
    break;
  case FORWARD_EXTRAPOLATE:
    success = robot_cmd.get_next_value(P_GAIN);
    if (!success)
      return;
    success = robot_cmd.get_next_value(I_GAIN);
    if (!success)
      return;
    success = robot_cmd.get_next_value(MAX_SPEED);
    if (!success)
      return;
    success = robot_cmd.get_next_value(target_tof);
    if (!success)
      return;
    success = robot_cmd.get_next_value(timingBudget);
    if (!success)
      return;
    success = robot_cmd.get_next_value(delay_time);
    if (!success)
      return;

    tx_estring_value.clear();

    distanceSensor1.setTimingBudgetInMs(timingBudget);

    blockReadTOF1(&distance1);
    last_tof = distance1;
    last_last_tof = distance1;
    time_between_tof = 1;
    last_tof_time = millis();

    integral_err = 0;
    lastDirection = Still;
    start_time = millis();
    last_time = millis();
    while (millis() - start_time < delay_time)
    {
      unsigned long current_time = millis();
      dt = current_time - last_time;
      last_time = current_time;

      if (nonBlockReadTOF1(&distance1))
      {
        distance1 = (distance1 + (long)((current_time - last_tof_time) * (last_tof - last_last_tof)) / time_between_tof);
      }
      else
      {
        last_last_tof = last_tof;
        last_tof = distance1;
        time_between_tof = current_time - last_tof_time;
        last_tof_time = current_time;
      }
      ControlDataBuffer[dataIndex].tof_distance = distance1;

      err = distance1 - target_tof;
      integral_err = integral_err + (err * dt);

      // if (abs(err) < 10)
      // {
      //   integral_err = 0;
      // }

      // clamp integral error to only contribute to 1/2 of max speed
      if (abs(integral_err * I_GAIN) > (MAX_SPEED / 2))
      {
        integral_err *= 1 / (MAX_SPEED / (2 * I_GAIN));
      }
      speed = (int)(err * P_GAIN + integral_err * I_GAIN);

      if (speed > MAX_SPEED)
      {
        speed = MAX_SPEED;
      }
      if (speed < -MAX_SPEED)
      {
        speed = -MAX_SPEED;
      }
      if (speed > 0)
      {
        if (lastDirection == Still)
        {
          moveForward(STATIC_TO_FORWARD_SPEED);
          delay(STATIC_TO_FORWARD_DELAY);
        }
        moveForward(speed);
        lastDirection = Forward;
      }
      else
      {
        moveBackward(-BACKWARD_MULTIPLIER * speed);
        lastDirection = Backward;
      }
      logControlData(dataIndex, current_time, speed, err, integral_err, distance1, dt);
      dataIndex++;
    }
    stop();

    for (int i = 0; i < dataIndex; i++)
    {
      sendControlData(i);
    }
    dataIndex = 0;
    break;
  case FORWARD:
    success = robot_cmd.get_next_value(left_speed);
    if (!success)
      return;
    success = robot_cmd.get_next_value(right_speed);
    if (!success)
      return;
    // moveCustom(left_speed, right_speed);
    analogWrite(LEFT_A, 100);
    analogWrite(LEFT_B, 0);
    analogWrite(RIGHT_A, 0);
    analogWrite(RIGHT_B, 127);
    break;
  case STOP:
    stop();
    break;
  case BACKWARD:
    success = robot_cmd.get_next_value(left_speed);
    if (!success)
      return;
    moveBackward(left_speed);
    break;
  case YAW_CONTROL:
    // packet should be in the form of "yaw_control|target_yaw_value|max_clockwise_speed|min_clockwise_speed|max_counter_clockwise_speed|min_counter_clockwise_speed|P_GAIN|D_GAIN|alpha"
    success = robot_cmd.get_next_value(CONTROL_YAW);
    if (!success)
      return;
    success = robot_cmd.get_next_value(target_angle);
    if (!success)
      return;
    success = robot_cmd.get_next_value(MAX_CLOCKWISE_SPEED);
    if (!success)
      return;
    success = robot_cmd.get_next_value(MIN_CLOCKWISE_SPEED);
    if (!success)
      return;
    success = robot_cmd.get_next_value(MAX_COUNTER_CLOCKWISE_SPEED);
    if (!success)
      return;
    success = robot_cmd.get_next_value(MIN_COUNTER_CLOCKWISE_SPEED);
    if (!success)
      return;
    success = robot_cmd.get_next_value(P_GAIN);
    if (!success)
      return;
    success = robot_cmd.get_next_value(D_GAIN);
    if (!success)
      return;
    success = robot_cmd.get_next_value(alpha);
    if (!success)
      return;
    break;
  case TURN_RIGHT:
    success = robot_cmd.get_next_value(left_speed);
    if (!success)
      return;
    success = robot_cmd.get_next_value(right_speed);
    if (!success)
      return;
    turnRight(left_speed, right_speed);

    break;
  case TURN_LEFT:
    success = robot_cmd.get_next_value(left_speed);
    if (!success)
      return;
    success = robot_cmd.get_next_value(right_speed);
    if (!success)
      return;
    turnLeft(left_speed, right_speed);
    break;
  case RECORD_PID_DATA:
    success = robot_cmd.get_next_value(RECORD_PID);
    if (!success)
      return;
    break;
  case SEND_PID_DATA:
    Serial.println("attempting to send over data");
    for (int i = 0; i < dataIndex; i++)
    {
      sendYawControlData(i);
    }
    dataIndex = 0;
    break;
  default:
    Serial.print("Invalid Command Type: ");
    Serial.println(cmd_type);
    break;
  }
}

void setup()
{
  Serial.begin(115200);

  // motor control setup
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  // ADC READ SETUP
  analogReadResolution(RESOLUTION_BITS);

  // Bluetooth setup
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  tx_characteristic_float.writeValue(0.0);

  // Example using the EString
  tx_estring_value.clear();
  tx_estring_value.append("[->");
  tx_estring_value.append(9.0);
  tx_estring_value.append("<-]");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  // Setup TOF
  setupTOF();

  // Setup IMU
  setupIMU();
}

void write_data()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    tx_float_value += 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000)
    {
      tx_float_value = 0;
    }

    previousMillis = currentMillis;
  }
}

void read_data()
{
  if (rx_characteristic_string.written())
  {
    handle_command();
  }
}

void pid_yaw_control()
{
  current_time = millis(); // for data logging
  bool counter_clockwise = false;
  // calculate proportional term
  if (yaw < target_angle)
  {
    if (360 - target_angle + yaw < target_angle - yaw)
    {
      // clockwise case
      yaw_err = 360 - target_angle + yaw;
    }
    else
    {
      // counter-clockwise case
      yaw_err = target_angle - yaw;
      counter_clockwise = true;
    }
  }
  else
  {
    if (yaw - target_angle < 360 - yaw + target_angle)
    {
      // clockwise case
      yaw_err = yaw - target_angle;
    }
    else
    {
      yaw_err = 360 - yaw + target_angle;
      counter_clockwise = true;
    }
  }

  // calculate derivative term
  yaw_err_d = (yaw_err - prev_yaw_err) / 0.008; // 0.008 comes from calculations to characterize contorl loop frequency
  filtered_yaw_err_d = alpha * yaw_err_d + (1 - alpha) * filtered_yaw_err_d;
  prev_yaw_err = yaw_err;

  // calculate PWM signal
  speed = (int)(P_GAIN * yaw_err + D_GAIN * filtered_yaw_err_d);
  if (!counter_clockwise)
  {
    if (speed > MAX_CLOCKWISE_SPEED)
    {
      speed = MAX_CLOCKWISE_SPEED;
    }
    else if (speed < MIN_CLOCKWISE_SPEED)
    {
      speed = MIN_CLOCKWISE_SPEED;
    }
    turnRight(speed, speed);
  }
  else
  {
    if (speed > (MAX_COUNTER_CLOCKWISE_SPEED))
    {
      speed = MAX_COUNTER_CLOCKWISE_SPEED;
    }
    else if (speed < (MIN_COUNTER_CLOCKWISE_SPEED))
    {
      speed = MIN_COUNTER_CLOCKWISE_SPEED;
    }
    turnLeft(speed, speed);
  }
}

void record_yaw_pid_data(int i, int time, int speed, int P_err, double D_err, double yaw, int dt)
{
  YawControlDataBuffer[i].time = time;
  YawControlDataBuffer[i].speed = speed;
  YawControlDataBuffer[i].P_err = P_err;
  YawControlDataBuffer[i].D_err = D_err;
  YawControlDataBuffer[i].yaw = yaw;
  YawControlDataBuffer[i].dt = dt;
}

// void loop()
// {
//   BLEDevice central = BLE.central();
//   if (central)
//   {
//     Serial.print("Connected to: ");
//     Serial.println(central.address());

//     while (central.connected())
//     {
//       write_data();
//       read_data();
//     }

//     Serial.println("Disconnected");
//   }
// }

void loop()
{
  BLEDevice central = BLE.central();
  if (central)
  {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected())
    {
      write_data();
      read_data();
      readDMPYaw(yaw);
      if (CONTROL_YAW)
      {
        pid_yaw_control();
        if (RECORD_PID)
        {
          record_yaw_pid_data(dataIndex, current_time, speed, yaw_err, int(filtered_yaw_err_d), yaw, dt);
          dataIndex++;
        }
      }
    }
    Serial.println("Disconnected");
  }
  readDMPYaw(yaw);
}
