// COMMANDS:

// - calibrate gyro
// - calibrate accel

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <stdlib.h>
#include "lab2.h"
#include <ArduinoBLE.h>

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

#define RESOLUTION_BITS 16

#define DATA_BUFFER_SIZE 10000

struct AccelPacket
{
    long time;
    float pitch;
    float roll;
};

AccelPacket accelDataBuffer[DATA_BUFFER_SIZE];

struct GyroPacket
{
  long time;
  float pitch;
  float roll;
  float yaw;
};

GyroPacket gyroDataBuffer[DATA_BUFFER_SIZE];

bool record_data = false;
int dataIndex = 0;

enum CommandTypes
{
    PING,
    GET_ACCEL_PITCH,
    GET_ACCEL_ROLL,
    CALIBRATE_ACCEL,
    GET_250_ACCEL,
    GET_GYRO_DATA,
    GET_COMP_DATA,
    START_RECORD,
    STOP_RECORD
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

    char buffer[50];
    float pitch, roll, yaw; // Declare variables outside the switch statement
    int lowpass;
    int time;
    float pitchConversion, rollConversion, pitchOffset, rollOffset;

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

    case GET_ACCEL_PITCH:
        pitch = getAccelerometerPitch(&myICM);

        tx_estring_value.clear();
        tx_estring_value.append(pitch);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back accel pitch: ");
        Serial.println(tx_estring_value.c_str());
        break;

    case GET_ACCEL_ROLL:
        roll = getAccelerometerRoll(&myICM);

        tx_estring_value.clear();
        tx_estring_value.append(roll);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back accel roll: ");
        Serial.println(tx_estring_value.c_str());
        break;

    case CALIBRATE_ACCEL:
    {
        // extract values
        success = robot_cmd.get_next_value(pitchConversion);
        if (!success)
            return;
        success = robot_cmd.get_next_value(rollConversion);
        if (!success)
            return;
        success = robot_cmd.get_next_value(pitchOffset);
        if (!success)
            return;
        success = robot_cmd.get_next_value(rollOffset);
        if (!success)
            return;

        calibrateAccelerometer(pitchConversion, rollConversion, pitchOffset, rollOffset);
        break;
    }

    case GET_250_ACCEL:
    {
        success = robot_cmd.get_next_value(lowpass);
        if (!success) {
          Serial.println("151");
            return;
        }

        dataIndex = 0;

        while (dataIndex < DATA_BUFFER_SIZE)
        {
          Serial.print("dataIndex: ");
          Serial.println(dataIndex);
          if (!lowpass) {
            accelDataBuffer[dataIndex].time = millis();
            accelDataBuffer[dataIndex].pitch = getAccelerometerPitch(&myICM);
            accelDataBuffer[dataIndex].roll = getAccelerometerRoll(&myICM);
            dataIndex++;
            delay(1);
          } else {
            accelDataBuffer[dataIndex].time = millis();
            accelDataBuffer[dataIndex].pitch = getAccelPitchLowPass(&myICM);
            accelDataBuffer[dataIndex].roll = getAccelRollLowPass(&myICM);
            dataIndex++;
            delay(1);
          }
        }

        for (int i = 0; i < dataIndex; i++)
        {
          Serial.println("178");
            tx_estring_value.clear();
          Serial.println("180");
            sprintf(buffer, "%ld", accelDataBuffer[i].time);
          Serial.println("183");
            tx_estring_value.append(buffer);
          Serial.println("184");
            tx_estring_value.append("|");
          Serial.println("185");
            tx_estring_value.append(accelDataBuffer[i].pitch);
          Serial.println("186");
            tx_estring_value.append("|");
          Serial.println("187");
            tx_estring_value.append(accelDataBuffer[i].roll);

            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
        }
        break;
    }
    case GET_GYRO_DATA:
      dataIndex = 0;
      while (dataIndex < DATA_BUFFER_SIZE)
      {
        gyroDataBuffer[dataIndex].pitch = getGyroscopePitch(&myICM);
        gyroDataBuffer[dataIndex].roll = getGyroscopeRoll(&myICM);
        gyroDataBuffer[dataIndex].yaw = getGyroscopeYaw(&myICM);
        gyroDataBuffer[dataIndex].time = millis();
        dataIndex++;
      }
      for (int i = 0; i < dataIndex; i++)
      {
          tx_estring_value.clear();
          sprintf(buffer, "%ld", gyroDataBuffer[i].time);
          tx_estring_value.append(buffer);
          tx_estring_value.append("|");
          tx_estring_value.append(gyroDataBuffer[i].pitch);
          tx_estring_value.append("|");
          tx_estring_value.append(gyroDataBuffer[i].roll);
          tx_estring_value.append("|");
          tx_estring_value.append(gyroDataBuffer[i].yaw);

          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
      }
        break;

    case GET_COMP_DATA:
      dataIndex = 0;
      while (dataIndex < DATA_BUFFER_SIZE)
      {
        while (!myICM.dataReady());
        myICM.getAGMT();
        gyroDataBuffer[dataIndex].pitch = accelGyroPitch(&myICM);
        gyroDataBuffer[dataIndex].roll = accelGyroRoll(&myICM);
        gyroDataBuffer[dataIndex].yaw = getGyroscopeYaw(&myICM);
        gyroDataBuffer[dataIndex].time = millis();
        dataIndex++;
      }
      for (int i = 0; i < dataIndex; i++)
      {
          tx_estring_value.clear();
          sprintf(buffer, "%ld", gyroDataBuffer[i].time);
          tx_estring_value.append(buffer);
          tx_estring_value.append("|");
          tx_estring_value.append(gyroDataBuffer[i].pitch);
          tx_estring_value.append("|");
          tx_estring_value.append(gyroDataBuffer[i].roll);
          tx_estring_value.append("|");
          tx_estring_value.append(gyroDataBuffer[i].yaw);

          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
      }
      break;

    case START_RECORD:
      record_data = true;      
      break;
    case STOP_RECORD:
      record_data = false;
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
    // ADC READ SETUP
    analogReadResolution(RESOLUTION_BITS);

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

char time_buffer[50];

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
            if (record_data) {
              while (!myICM.dataReady() && dataIndex != DATA_BUFFER_SIZE);
              myICM.getAGMT();
              gyroDataBuffer[dataIndex].pitch = accelGyroPitch(&myICM);
              gyroDataBuffer[dataIndex].roll = accelGyroRoll(&myICM);
              gyroDataBuffer[dataIndex].yaw = getGyroscopeYaw(&myICM);
              gyroDataBuffer[dataIndex].time = millis();
              dataIndex += 1;
            }
            if (!record_data && dataIndex != 0) {
            for (int i = 0; i < dataIndex; i++)
            {
                tx_estring_value.clear();
                sprintf(time_buffer, "%ld", gyroDataBuffer[i].time);
                tx_estring_value.append(time_buffer);
                tx_estring_value.append("|");
                tx_estring_value.append(gyroDataBuffer[i].pitch);
                tx_estring_value.append("|");
                tx_estring_value.append(gyroDataBuffer[i].roll);
                tx_estring_value.append("|");
                tx_estring_value.append(gyroDataBuffer[i].yaw);

                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                Serial.print("Sent back: ");
                Serial.println(tx_estring_value.c_str());
            }
            dataIndex=0;
            }
        }

        Serial.println("Disconnected");
    }
}
