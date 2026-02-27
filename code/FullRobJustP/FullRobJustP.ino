#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;

// pin connections
const int dir_pin_1 = 17; // dir_pin_1ection pin
const int step_pin_1 = 16; // step_pin_1 pin
const int dir_pin_2 = 4;
const int step_pin_2 = 5;
unsigned long lastStep = 0;
float beta = 1;

// Raw sensor values
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Offset values (calculated during calibration)
int32_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int32_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

const int CALIBRATION_SAMPLES = 1000;
const float AccVal = 16384.0;
const float GyrVal = 131.0;

// Complementary filter variables
float angleX = 0.0, angleY = 0.0;
float alpha = 0.97; // Filter coefficient (0.96 = 96% gyro, 4% accel)
unsigned long lastTime = 0;

double Setpoint = 0, Input, Output;
double Kp = 18, Ki = 0, Kd = 0.9;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, dirECT);

void calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  Serial.println("Keep the sensor STILL and FLAT");

  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;

    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    delay(2);
  }

  // Average offsets
  ax_offset = ax_sum / CALIBRATION_SAMPLES;
  ay_offset = ay_sum / CALIBRATION_SAMPLES;
  az_offset = (az_sum / CALIBRATION_SAMPLES) - 16384; // remove gravity

  gx_offset = gx_sum / CALIBRATION_SAMPLES;
  gy_offset = gy_sum / CALIBRATION_SAMPLES;
  gz_offset = gz_sum / CALIBRATION_SAMPLES;

  Serial.println("Calibration complete");
}

float PIDFunction(float angle) {
  if (abs(angle) < 1.5) return 0;
  Input = angle;
  myPID.Compute();
  return Output;
}

void MotorMove(float angle) {
  if (abs(angle) < 1.5) return;
  float interval = (-1.94 * abs(angle)) + 1000; // Linear relation, should change to variable hyperbola depending on single variable.
  Serial.println(interval);
  if (angle < 0) {
    digitalWrite(dir_pin_2, LOW);
    digitalWrite(dir_pin_1, LOW);
  }
  else {
    digitalWrite(dir_pin_2, HIGH); 
    digitalWrite(dir_pin_1, HIGH);
  }
  if (micros() - lastStep >= interval) {
    digitalWrite(step_pin_2, HIGH);
    digitalWrite(step_pin_1, HIGH);
    digitalWrite(step_pin_2, LOW);
    digitalWrite(step_pin_1, LOW);
    lastStep = micros();
  }
}

void setup() {
  pinMode(dir_pin_1, OUTPUT);
  pinMode(step_pin_1, OUTPUT);
  pinMode(step_pin_2, OUTPUT);
  pinMode(dir_pin_2, OUTPUT);
  // set dir_pin_1ection of rotation to clockwise
  digitalWrite(dir_pin_1, HIGH);
  digitalWrite(dir_pin_2, HIGH);
  
  Serial.begin(9600);
  Wire.begin();

  //initialize the variables we're linked to
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-360, 360);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  mpu.setDLPFMode(3);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  calibrateMPU();
  lastTime = millis();
}

void loop() {
  // Calculate time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  // Read sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply offsets
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;

  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // Convert to physical units
  float accelX = ax / AccVal;
  float accelY = ay / AccVal;
  float accelZ = az / AccVal;

  float gyroX = gx / GyrVal;
  float gyroY = gy / GyrVal;
  float gyroZ = gz / GyrVal;

  // Calculate angles from accelerometer (in degrees)
  float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  // Integrate gyroscope data (gyro gives rate of change)
  angleX = alpha * (angleX + gyroX * dt) + (1.0 - alpha) * accelAngleX;
  angleY = alpha * (angleY + gyroY * dt) + (1.0 - alpha) * accelAngleY;

  // PID Function
  float NangleX = PIDFunction(angleX);

  Serial.print("PID Corrected Angle: ");
  Serial.println(NangleX);

  // Motor Reactions
  MotorMove(NangleX);

  // Print results
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print("° | Angle Y: ");
  Serial.print(angleY);
  Serial.print("° | Gyro Z: ");
  Serial.print(gyroZ);
  Serial.println("°/s");
}