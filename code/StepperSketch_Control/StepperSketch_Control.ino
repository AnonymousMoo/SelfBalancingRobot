#define DIR_PIN_1  17
#define STEP_PIN_1 16
#define DIR_PIN_2  4
#define STEP_PIN_2 5

#define STEPS_PER_REV 200
#define SPEED_DELAY   1000  // microseconds between steps (adjust for speed)

void stepMotor(int stepPin, int dirPin, bool direction, int steps) {
  digitalWrite(dirPin, direction ? HIGH : LOW);
  delay(1);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(SPEED_DELAY);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SPEED_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("TMC2208 Stepper Motor Test");

  pinMode(DIR_PIN_1,  OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2,  OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);

  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(STEP_PIN_2, LOW);
}

void loop() {
  Serial.println("Motor 1 - Forward 1 revolution");
  stepMotor(STEP_PIN_1, DIR_PIN_1, true, STEPS_PER_REV);
  delay(500);

  Serial.println("Motor 1 - Backward 1 revolution");
  stepMotor(STEP_PIN_1, DIR_PIN_1, false, STEPS_PER_REV);
  delay(500);

  Serial.println("Motor 2 - Forward 1 revolution");
  stepMotor(STEP_PIN_2, DIR_PIN_2, true, STEPS_PER_REV);
  delay(500);

  Serial.println("Motor 2 - Backward 1 revolution");
  stepMotor(STEP_PIN_2, DIR_PIN_2, false, STEPS_PER_REV);
  delay(500);

  Serial.println("Both motors - Forward simultaneously");
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, HIGH);
  delay(1);
  for (int i = 0; i < STEPS_PER_REV; i++) {
    digitalWrite(STEP_PIN_1, HIGH);
    digitalWrite(STEP_PIN_2, HIGH);
    delayMicroseconds(SPEED_DELAY);
    digitalWrite(STEP_PIN_1, LOW);
    digitalWrite(STEP_PIN_2, LOW);
    delayMicroseconds(SPEED_DELAY);
  }

  delay(1000);
  Serial.println("--- Cycle complete, repeating ---\n");
}
