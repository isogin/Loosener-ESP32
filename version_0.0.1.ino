#include "ICM42670P.h"
#include "Wire.h"
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef12-3456-7890-1234-56789abcdef0"

BLECharacteristic *pCharacteristic;
ICM42670 IMU(Wire, 0x69);

const int pressureSensorPin_1 = 34; //圧力センサ用のアナログピン 1つ目
const int pressureSensorPin_2 = 35; //圧力センサ用のアナログピン 2つ目
bool isMeasureStart = false;

float pressureData_1[60];
float pressureData_2[60];
float velocityData[60];

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float gravityX = 0, gravityY = 0, gravityZ = 0;
float linear_accelerationX = 0, linear_accelerationY = 0, linear_accelerationZ = 0;
float kalAngleX = 0, kalAngleY = 0;
const float alpha = 0.8;
float firstTotalVelocity = 0;
unsigned long lastTime = 0, currentTime = 0;



void setup() {
  Serial.begin(115200);
  initializeSensors();
  initializeBLE();
}

void loop() {
  if (isMeasureStart) {
    isMeasureStart = false;
    Serial.print("パフォーマンスメジャメント");
    performMeasurement();
  }
}