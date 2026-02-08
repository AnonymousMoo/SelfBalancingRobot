// pin connections
const int dir = 16; // direction pin
const int step = 17; // step pin
const int dir2 = 5;
const int step2 = 4;
const int stepCount = 200 * 8; // step count
const int tim = 1000;
const int tim2 = 500;

void setup() {
  pinMode(dir, OUTPUT);
  pinMode(step, OUTPUT);
  pinMode(step2, OUTPUT);
  pinMode(dir2, OUTPUT);
  // set direction of rotation to clockwise
  digitalWrite(dir, HIGH);
  digitalWrite(dir2, HIGH);
}

void MotorMove(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(step, HIGH);
    delayMicroseconds(tim2);
    // pause before taking next step
    digitalWrite(step, LOW);
    delayMicroseconds(tim2);
    digitalWrite(step2, HIGH);
    delayMicroseconds(tim2);
    // pause before taking next step
    digitalWrite(step2, LOW);
    delayMicroseconds(tim2);
  }
}

void loop() {
  MotorMove(stepCount);
  delay(tim);
  digitalWrite(dir, LOW);
  digitalWrite(dir2, LOW);
  MotorMove(stepCount);
  delay(tim);
  digitalWrite(dir, HIGH);
  digitalWrite(dir2, HIGH);
}