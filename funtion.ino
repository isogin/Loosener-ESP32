class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value == "Start Measurement") {
            Serial.println("計測開始!!");
            isMeasureStart = true;
        }
    }
};

// センサーの初期化
void initializeSensors() {
  Wire.begin();
  if (IMU.begin() != 0) {
    Serial.println("Failed to detect and initialize ICM42670!");
    while (1);
  }
  IMU.startAccel(100, 16);  // 100 Hz, ±16G
  IMU.startGyro(100, 2000); // 100 Hz, ±2000 dps
  delay(50);
  lastTime = micros();
}

// BLEの初期化
void initializeBLE() {
  BLEDevice::init("ルーズナー");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
}



// 圧力センサ1の値を取得
float readPressureSensor_1() {
  int pressureValue_1 = analogRead(pressureSensorPin_1);
  return (4095.0 / pressureValue_1 * 600.0) - 600.0;
}

// 圧力センサ2の値を取得
float readPressureSensor_2() {
  int pressureValue_2 = analogRead(pressureSensorPin_2);
  return (4095.0 / pressureValue_2 * 600.0) - 600.0;
}


// 加速度センサとジャイロセンサの値を更新
void updateSensorData() {
  inv_imu_sensor_event_t imu_event;
  IMU.getDataFromRegisters(imu_event);

  accX = imu_event.accel[0];
  accY = imu_event.accel[1];
  accZ = imu_event.accel[2];
  gyroX = imu_event.gyro[0];
  gyroY = imu_event.gyro[1];
  gyroZ = imu_event.gyro[2];
}


void performMeasurement() {
  const int chunkSize = 2;  // 分割サイズ（2回ごと）
  const int totalSize = 40; // 全データ数

  for (int m = 0; m < totalSize; m++) {
    // 圧力センサ1の値を読み取る
    pressureData_1[m] = readPressureSensor_1();
    //圧力センサ2の値を読み取る
    pressureData_2[m] = readPressureSensor_2();
    // 加速度センサの値を更新
    updateSensorData();

    velocityData[m] = abs((accX + accY + accZ - 2750) * 0.01);

    // 0.1秒待機
    delay(100);

    // チャンクが完成したら送信
    if ((m + 1) % chunkSize == 0 || m == totalSize - 1) {
      String chunkData = buildChunkData(m - chunkSize + 1, min(m + 1, totalSize));
      pCharacteristic->setValue(chunkData.c_str());
      pCharacteristic->notify();
      delay(50);  // 必要に応じて適度な遅延
    }
  }
  Serial.print("計測終了");
  return;
}

// データをチャンクに分割して送信可能な形式に構築
String buildChunkData(int startIdx, int endIdx) {
  String chunkData = "";
  for (int n = startIdx; n < endIdx; n++) {
    //2進数に変換して結合
    chunkData += String(velocityData[n], 2) + "," + String(pressureData_1[n], 2) + String(pressureData_2[n], 2);
    if (n != endIdx - 1) {
      chunkData += ",";  // 各データの区切り
    }
  }
  //Serial.println("送信データ: " + chunkData);
  return chunkData;
}