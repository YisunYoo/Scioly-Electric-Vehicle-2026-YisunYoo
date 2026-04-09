//BNO_08X_yaw_tester 
//Using I2C mode, with esp32
//When testing with arduino nano esp32 SPI communication mode, change the pins, and change I2C mode to SPI mode

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "sh2.h" 

BNO08x myIMU;

#define BNO08X_SDA  19
#define BNO08X_SCL  18
#define BNO08X_ADDR 0x4B
#define BNO08X_USE_INT_RST 0

void printCalConfig() {
  uint8_t cfg = 0;
  int rc = sh2_getCalConfig(&cfg);   // 0 = success
  if (rc != 0) {
    Serial.print("sh2_getCalConfig failed, rc=");
    Serial.println(rc);
    return;
  }

  bool accelOn = (cfg & SH2_CAL_ACCEL);
  bool gyroOn  = (cfg & SH2_CAL_GYRO);
  bool magOn   = (cfg & SH2_CAL_MAG);

  Serial.print("Cal mask=0x");
  Serial.println(cfg, HEX);
  Serial.print("ACCEL="); Serial.print(accelOn);
  Serial.print(" GYRO="); Serial.print(gyroOn);
  Serial.print(" MAG=");  Serial.println(magOn);

  if (accelOn && gyroOn && !magOn) {
    Serial.println("OK: accel+gyro ON, mag OFF");
  } else {
    Serial.println("NOT as requested");
  }
}
bool beginIMURetry() {
  const int maxAttempts = 100;

  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    Serial.print("IMU begin attempt ");
    Serial.println(attempt);

    if (myIMU.begin(BNO08X_ADDR, Wire, -1, -1)) {
      Serial.println("BNO08x begin OK");
      return true;
    }

    delay(200);
  }

  return false;
}


bool configureReports() {
  return myIMU.enableGameRotationVector(10);
}

bool configureCalibration() {
  // accel+gyro ON, mag OFF
  return myIMU.setCalibrationConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}


if (!beginIMURetry()) {
  Serial.println("BNO08x begin failed after retries");
  while (1) delay(10);
}


  Wire.begin(BNO08X_SDA, BNO08X_SCL);
  Wire.setClock(100000);  // start conservative

  delay(200);

#if BNO08X_USE_INT_RST
  if (!myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST))
#else
  if (!myIMU.begin(BNO08X_ADDR, Wire, -1, -1))
#endif
  {
    Serial.println("BNO08x begin failed");
    while (1) delay(10);
  }

  if (!configureCalibration()) {
    Serial.println("Calibration config failed");
    while (1) delay(10);
  }
  printCalConfig();

  if (!configureReports()) {
    Serial.println("Enable report failed");
    while (1) delay(10);
  }

  Serial.println("BNO08x ready");
}

void loop() {
  if (myIMU.wasReset()) {
    configureCalibration();
    configureReports();
  }

  if (!myIMU.getSensorEvent()) return;
  if (myIMU.getSensorEventID() != SENSOR_REPORTID_GAME_ROTATION_VECTOR) return;

  float qi = myIMU.getGameQuatI();
  float qj = myIMU.getGameQuatJ();
  float qk = myIMU.getGameQuatK();
  float qr = myIMU.getGameQuatReal();
  float yaw = atan2f(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (qj * qj + qk * qk)) * RAD_TO_DEG;

  Serial.println(yaw, 3);
}
