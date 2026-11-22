#define USE_AVERAGING 0
#define USE_ANGULAR_NOISE 1
#define USE_MOTOR 1

#include <Stepper.h>
#include <math.h>

// ============================================================
// HALF-WAVE PLATE POLARIZATION DATA COLLECTOR
// Hardware inherited from the supplied sketches:
//   Photodiode 1: A4
//   Photodiode 2: A5
//   Laser control: D8 (LOW = OFF, HIGH = ON)
//   28BYJ-48-style stepper: D7, D6, D5, D4
//
// IMPORTANT: Before powering on, physically align the plate to
// your chosen 0-degree reference. There is no homing switch.
// ============================================================

const int PHOTODIODE_1 = A4;
const int PHOTODIODE_2 = A5;
const int LASER_PIN = 8;

const int MOTOR_PIN_1 = 7;
const int MOTOR_PIN_2 = 6;
const int MOTOR_PIN_3 = 5;
const int MOTOR_PIN_4 = 4;

const int STEPS_PER_REVOLUTION = 2048;
const float STEPS_PER_DEGREE = STEPS_PER_REVOLUTION / 360.0f;
const int MOTOR_RPM = 12;

const int BITS_TO_TRANSMIT = 11;
const int REPETITIONS_PER_BIT = 9;
const int TOTAL_PHYSICAL_BITS = BITS_TO_TRANSMIT * REPETITIONS_PER_BIT;

// Retains the logical message from rotate_half_wave_plate_MOTORNOWORK.ino
const byte LOGICAL_BITS[BITS_TO_TRANSMIT] =
  {1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1};

const int READINGS_PER_AVERAGE = 100;
const unsigned long SETTLE_MS = 300;
const float ANGULAR_NOISE_SD_DEGREES = 10.0f;

Stepper halfWavePlate(STEPS_PER_REVOLUTION,
                      MOTOR_PIN_1, MOTOR_PIN_3,
                      MOTOR_PIN_2, MOTOR_PIN_4);

float currentAngle = 0.0f;

// Calibration extrema for each detector
int pd1Min = 1023, pd1Max = 0;
int pd2Min = 1023, pd2Max = 0;

// Best measured angle for each transmitted state
float bestAngleFor1 = 0.0f;
float bestAngleFor0 = 0.0f;
float decisionBoundaryAngle = 0.0f;

struct DetectorReadings {
  float pd1;
  float pd2;
};

float wrapAngle(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float shortestAngleDelta(float target, float current) {
  float delta = wrapAngle(target) - wrapAngle(current);
  if (delta > 180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;
  return delta;
}

void moveMotorToAngle(float targetAngle) {
  targetAngle = wrapAngle(targetAngle);
  float delta = shortestAngleDelta(targetAngle, currentAngle);
  long steps = lround(delta * STEPS_PER_DEGREE);
  if (steps != 0) halfWavePlate.step(steps);
  currentAngle = wrapAngle(currentAngle + steps / STEPS_PER_DEGREE);
}

void laserOff() { digitalWrite(LASER_PIN, LOW); }
void laserOn()  { digitalWrite(LASER_PIN, HIGH); }

DetectorReadings readDetectors() {
  DetectorReadings result;
#if USE_AVERAGING
  unsigned long sum1 = 0;
  unsigned long sum2 = 0;
  for (int i = 0; i < READINGS_PER_AVERAGE; i++) {
    sum1 += analogRead(PHOTODIODE_1);
    sum2 += analogRead(PHOTODIODE_2);
  }
  result.pd1 = sum1 / (float)READINGS_PER_AVERAGE;
  result.pd2 = sum2 / (float)READINGS_PER_AVERAGE;
#else
  result.pd1 = analogRead(PHOTODIODE_1);
  result.pd2 = analogRead(PHOTODIODE_2);
#endif
  return result;
}

float normalizedPD1(float raw) {
  if (pd1Max <= pd1Min) return 0.0f;
  return constrain((raw - pd1Min) / (float)(pd1Max - pd1Min), 0.0f, 1.0f);
}

float normalizedPD2(float raw) {
  if (pd2Max <= pd2Min) return 0.0f;
  return constrain((raw - pd2Min) / (float)(pd2Max - pd2Min), 0.0f, 1.0f);
}

// Positive score favors bit 1; negative score favors bit 0.
float decisionScore(const DetectorReadings &r) {
  return normalizedPD1(r.pd1) - normalizedPD2(r.pd2);
}

void resetCalibration() {
  pd1Min = 1023; pd1Max = 0;
  pd2Min = 1023; pd2Max = 0;
}

void printCalibration() {
  Serial.println(F("\nCALIBRATION RESULTS"));
  Serial.println(F("Detector,Minimum,Maximum"));
  Serial.print(F("PD1,")); Serial.print(pd1Min); Serial.print(',');
  Serial.println(pd1Max);
  Serial.print(F("PD2,")); Serial.print(pd2Min); Serial.print(',');
  Serial.println(pd2Max);
  Serial.print(F("Best angle for bit 1 (deg): "));
  Serial.println(bestAngleFor1, 2);
  Serial.print(F("Best angle for bit 0 (deg): "));
  Serial.println(bestAngleFor0, 2);
  Serial.print(F("Measured decision-boundary angle (deg): "));
  Serial.println(decisionBoundaryAngle, 2);
}

// First complete 360-degree pass: determine min/max for both detectors.
void calibrationExtremaPass() {
  Serial.println(F("\nCALIBRATION PASS 1: measuring detector minima and maxima"));
  laserOn();
  delay(500);
  for (int degree = 0; degree < 360; degree++) {
    moveMotorToAngle((float)degree);
    delay(SETTLE_MS);
    DetectorReadings r = readDetectors();

    if (r.pd1 < pd1Min) pd1Min = (int)r.pd1;
    if (r.pd1 > pd1Max) pd1Max = (int)r.pd1;
    if (r.pd2 < pd2Min) pd2Min = (int)r.pd2;
    if (r.pd2 > pd2Max) pd2Max = (int)r.pd2;

    if (degree % 30 == 0) {
      Serial.print(F("Calibration angle: "));
      Serial.println(degree);
    }
  }
  laserOff();
  Serial.println(F("Pass 1 complete."));
}

// Second pass: with extrema known, find strongest 1/0 angles and
// the measured angle where normalized detector responses are closest.
void calibrationOptimizationPass() {
  Serial.println(F("\nCALIBRATION PASS 2: finding best state angles and boundary"));
  float highestScore = -999.0f;
  float lowestScore = 999.0f;
  float smallestBoundaryMagnitude = 999.0f;

  laserOn();
  delay(500);
  for (int degree = 0; degree < 360; degree++) {
    moveMotorToAngle((float)degree);
    delay(SETTLE_MS);
    DetectorReadings r = readDetectors();
    float score = decisionScore(r);

    if (score > highestScore) {
      highestScore = score;
      bestAngleFor1 = degree;
    }
    if (score < lowestScore) {
      lowestScore = score;
      bestAngleFor0 = degree;
    }
    if (fabs(score) < smallestBoundaryMagnitude) {
      smallestBoundaryMagnitude = fabs(score);
      decisionBoundaryAngle = degree;
    }
  }
  laserOff();
}

void calibrateSystem() {
  resetCalibration();
  calibrationExtremaPass();
  calibrationOptimizationPass();
  printCalibration();

  Serial.println(F("\nMoving plate to measured decision boundary..."));
  moveMotorToAngle(decisionBoundaryAngle);
  delay(SETTLE_MS);
  Serial.println(F("Calibration complete."));
}

float gaussianNoise(float standardDeviation) {
  // Box-Muller transform. randomSeed() is initialized in setup().
  float u1 = (random(1, 1000000) / 1000000.0f);
  float u2 = (random(1, 1000000) / 1000000.0f);
  return standardDeviation * sqrt(-2.0f * log(u1)) *
         cos(2.0f * PI * u2);
}

float targetAngleForBit(byte bitValue) {
  float target = bitValue ? bestAngleFor1 : bestAngleFor0;
#if USE_ANGULAR_NOISE
  target += gaussianNoise(ANGULAR_NOISE_SD_DEGREES);
#endif
  return wrapAngle(target);
}

void printCsvHeader() {
  Serial.println(F("\nphysical_index,logical_index,repeat_index,transmitted_bit,"
                   "target_angle_deg,actual_angle_deg,pd1_raw,pd2_raw,"
                   "pd1_normalized,pd2_normalized,received_bit"));
}

void collectData() {
  printCsvHeader();

  for (int physicalIndex = 0;
       physicalIndex < TOTAL_PHYSICAL_BITS;
       physicalIndex++) {

    int logicalIndex = physicalIndex / REPETITIONS_PER_BIT;
    int repeatIndex = physicalIndex % REPETITIONS_PER_BIT;
    byte transmittedBit = LOGICAL_BITS[logicalIndex];
    float targetAngle = targetAngleForBit(transmittedBit);

    // Required sequence: laser off -> move -> stop/settle -> laser on -> read.
    laserOff();
    moveMotorToAngle(targetAngle);
    delay(SETTLE_MS);

    laserOn();
    delay(100);
    DetectorReadings r = readDetectors();
    laserOff();

    float n1 = normalizedPD1(r.pd1);
    float n2 = normalizedPD2(r.pd2);
    byte receivedBit = (n1 >= n2) ? 1 : 0;

    Serial.print(physicalIndex + 1); Serial.print(',');
    Serial.print(logicalIndex + 1); Serial.print(',');
    Serial.print(repeatIndex + 1); Serial.print(',');
    Serial.print(transmittedBit); Serial.print(',');
    Serial.print(targetAngle, 2); Serial.print(',');
    Serial.print(currentAngle, 2); Serial.print(',');
    Serial.print(r.pd1, 2); Serial.print(',');
    Serial.print(r.pd2, 2); Serial.print(',');
    Serial.print(n1, 4); Serial.print(',');
    Serial.print(n2, 4); Serial.print(',');
    Serial.println(receivedBit);

    delay(200);
  }

  laserOff();
  Serial.println(F("\nDATA COLLECTION COMPLETE."));
  Serial.println(F("Arduino is finished; reset to recalibrate and run again."));
}

void setup() {
  Serial.begin(9600);
  pinMode(PHOTODIODE_1, INPUT);
  pinMode(PHOTODIODE_2, INPUT);
  pinMode(LASER_PIN, OUTPUT);
  laserOff();

  halfWavePlate.setSpeed(MOTOR_RPM);
  randomSeed(analogRead(A0));

  Serial.println(F("Half-wave plate data collector starting."));
  Serial.println(F("Assumption: plate is physically aligned to 0 degrees."));
  delay(1000);

#if USE_MOTOR
  calibrateSystem();
#else
  manualCalibrateSystem();
#endif

  collectData();
}

void loop() {
  // One complete run per reset.
}
