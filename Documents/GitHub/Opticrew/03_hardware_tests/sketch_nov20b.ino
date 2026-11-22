const int stepPin = 6;           // DRV8825 STEP
const int dirPin  = 5;           // DRV8825 DIR
const int EN_PIN   = 7;           // DRV8825 /ENABLE (LOW = energized)

const int M1 = 4;
const int M2 = 3;
const int M3 = 2;

const int stepsPerRevolution = 200;

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(M3, HIGH);
}

void loop() {
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor slowly
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);  // Wait a second

  // Set motor direction counterclockwise
  digitalWrite(dirPin, LOW);

  // Spin motor quickly
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  delay(1000);  // Wait a second
}