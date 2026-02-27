#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

MPU6050 mpu;

bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[128];

Quaternion q;
VectorFloat gravity;
float ypr[3];

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(2000);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }
  Serial.println("MPU6050 connected");

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDLPFMode(3); // 42Hz low-pass — smoother accel corrections
    mpu.setRate(9);
    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready");
  } else {
    Serial.print("DMP init failed, code: ");
    Serial.println(devStatus);
  }
}

void loop() {
  if (!DMPReady) return;

  // polling instead of waiting for interrupt
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print("Yaw: ");   Serial.print((ypr[0] * 180 / M_PI) * 2);
    Serial.print("  Pitch: "); Serial.print((ypr[1] * 180 / M_PI) * 2);
    Serial.print("  Roll: ");  Serial.println((ypr[2] * 180 / M_PI) * 2);
  }
}