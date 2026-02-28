//Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"
#include "PID_v1.h"

//MPU6050 Object
MPU6050 mpu;

//MPU6050 Variables for setup of DMP
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[128];

//MPU6050 Variables for getting YPR angles(Although we only need Roll).
Quaternion q;
VectorFloat gravity;
float ypr[3];

//Stepper Motor Pins
#define DIR_PIN_1  17
#define STEP_PIN_1 16
#define DIR_PIN_2  4
#define STEP_PIN_2 5

//Stepper speed variables
int motorSpeed = 0;
unsigned long lastStepTime1 = 0;
unsigned long lastStepTime2 = 0;

// PID variables
double setpoint = 0;  // Desired angle of 0 degrees
double input, output;

// PID gain control
double Kp = 525.0;
double Ki = 17.0;
double Kd = 19.0;

// PID controller object
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Timing variables
unsigned long lastPIDUpdate = 0;
const unsigned long PIDInterval = 15; // PID runs every 15 seconds

// Task handles
TaskHandle_t pidTaskHandle;

// Function prototypes
double pullAngle();
void pidLoop(void *parameter);
void motorControl();

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(2000);

  Serial.println("TMC2208 Stepper Motor Test");

  pinMode(DIR_PIN_1,  OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2,  OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);

  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(STEP_PIN_2, LOW);

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

  // Initialise PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-10000, 10000);

  // Create a task for the PID loop on core 0
  xTaskCreatePinnedToCore(pidLoop, "PID Task", 10000, NULL, 2, &pidTaskHandle, 0);
  delay(4000);
}

void loop() {
  motorControl();
}

void motorControl() {
  if (motorSpeed > 0) { 
        digitalWrite(DIR_PIN_2, LOW); 
      } else if (motorSpeed < 0) {
        digitalWrite(DIR_PIN_2, HIGH); 
      }
  delay(1);
  for (int i = 0; i < 50; i++) {
    digitalWrite(STEP_PIN_2, HIGH);
    digitalWrite(STEP_PIN_1, HIGH);
    delayMicroseconds(600);
    digitalWrite(STEP_PIN_2, LOW);
    digitalWrite(STEP_PIN_1, LOW);
    delayMicroseconds(600);
  }
}

void pidLoop(void *parameter) {
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDUpdate >= PIDInterval) {
      lastPIDUpdate = currentMillis;

      // Update MPU6050 angle
      double angle = pullAngle();  // Read the X-axis angle

      // Smoothing
      static double smoothedAngle = 0.0;
      smoothedAngle += 0.3 * (angle - smoothedAngle);

      // Update PID controller
      input = smoothedAngle;
      myPID.Compute();

      // Map PID output to motor speeds
      motorSpeed = (int)output;

      // Set motor directions based on PID output
      if (motorSpeed > 0) {
        digitalWrite(DIR_PIN_1, LOW);  
        digitalWrite(DIR_PIN_2, LOW); 
      } else if (motorSpeed < 0) {
        digitalWrite(DIR_PIN_1, HIGH);  
        digitalWrite(DIR_PIN_2, HIGH); 
      }

      Serial.print("Angle: ");
      Serial.print(smoothedAngle);
      Serial.print(" Motor Speed: ");
      Serial.println(motorSpeed);
    }

    delay(1);
  }
}

double pullAngle() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return (ypr[2] * 180 / M_PI) * 2;
  }
  return input; // Return last known good angle instead
}