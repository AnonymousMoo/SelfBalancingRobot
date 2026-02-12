// pin connections
const int dir = 17; // direction pin
const int step = 16; // step pin
const int dir2 = 4;
const int step2 = 5;
const int tim = 1000;

unsigned long lastStep = 0;

void setup() {
  pinMode(dir, OUTPUT);
  pinMode(step, OUTPUT);
  pinMode(step2, OUTPUT);
  pinMode(dir2, OUTPUT);
  // set direction of rotation to clockwise
  digitalWrite(dir, HIGH);
  digitalWrite(dir2, HIGH);
}

void MotorMove(int stepsI) {
  if (micros() - lastStep >= stepsI) {
    digitalWrite(step2, HIGH);
    digitalWrite(step2, LOW);
    lastStep = micros();
  }
}

void loop() {
  for (int stepCT = 500; stepCT <= 2500; stepCT + 10) {
    MotorMove(stepCT);
  }
}