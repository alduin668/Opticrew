// ============================================================
// HALF-WAVE PLATE POLARIZATION DATA COLLECTOR  (rev 3)
//
// ONE sketch, eight modes. Flip the three switches below.
//
// Every photodiode reading, in every phase (calibration included), is
// printed as one CSV row the moment it is taken. All status text starts
// with "# ", so a whole Serial Monitor capture loads with:
//     pandas.read_csv("run.csv", comment="#")
//
// Serial Monitor: 9600 baud, line ending = "Newline" (commands and manual mode need it).
//
// COMMANDS (type the single letter + Enter at any time):
//     x  = abort the run: laser off, motor released, re-print all DATA collected so far, halt
//     d  = dump all DATA collected so far (phase = DATA_DUMP) and keep running
//   In motor mode the sketch checks for commands between measurements, so there can be a
//   short delay (up to the length of the current move) before it reacts.
//
// Bit convention (same as before): bit 1 = PD1 bright / PD2 dim,
//                                  bit 0 = PD1 dim / PD2 bright.
// All angles are PLATE angles in degrees (polarization turns twice as far).
// ============================================================
#define USE_MOTOR          1   // 1 = NEMA17 + DRV8825 turns the plate; 0 = you turn it by hand
#define USE_AVERAGING      0   // 1 = each measurement averages READINGS_PER_AVERAGE ADC reads
#define USE_ANGULAR_NOISE  0   // 1 = add Gaussian jitter to every bit's plate angle

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

// ---------------- Hardware ----------------
const int PHOTODIODE_1 = A4;
const int PHOTODIODE_2 = A5;
const int LASER_PIN    = 8;       // HIGH = laser on

const int STEP_PIN = 6;           // DRV8825 STEP
const int DIR_PIN  = 5;           // DRV8825 DIR
const int EN_PIN   = 7;           // DRV8825 /ENABLE (LOW = energized)

// DRV8825 microstep mode pins (from sketch_nov20b.ino)
const int M1_PIN = 4;
const int M2_PIN = 3;
const int M3_PIN = 2;

// ---------------- Motor geometry ----------------
const long  MOTOR_FULL_STEPS_PER_REV = 200;    // NEMA17, 1.8 deg per full step
const int   MICROSTEPS               = 16;     // Hardware pins below are set to 1/16 mode
                                               // M1=LOW, M2=LOW, M3=HIGH (matches Nov 20 sketch)
const float GEAR_RATIO               = 4.0f;   // motor turns per plate turn (20T pinion -> 80T ring)
                                               // set 1.0 if the plate is direct drive
const bool  INVERT_DIR               = false;  // flip if positive angles run the wrong way
const unsigned int STEP_PERIOD_US    = 800;    // time per microstep; raise it if the motor stalls
const long STEPS_PER_PLATE_REV =
  (long)(MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO + 0.5f);

// ---------------- Experiment ----------------
const int  BITS_TO_TRANSMIT    = 11;
const int  REPETITIONS_PER_BIT = 9;
const int  TOTAL_PHYSICAL_BITS = BITS_TO_TRANSMIT * REPETITIONS_PER_BIT;
const byte LOGICAL_BITS[BITS_TO_TRANSMIT] = {1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1};

const int  READINGS_PER_AVERAGE = 100;   // per measurement when USE_AVERAGING = 1
const int  CAL_READINGS         = 100;   // used by manual calibration mode
const unsigned long SETTLE_MS   = 300;
const unsigned long LASER_ON_MS = 100;

// Gaussian jitter, plate degrees. Proposal cases: 12.0 (clean maritime), 17.8 (urban).
const float ANGULAR_NOISE_SD_DEGREES = 10.0f;

#if USE_AVERAGING
const int MEASURE_READINGS = READINGS_PER_AVERAGE;
#else
const int MEASURE_READINGS = 1;
#endif

// ---------------- State ----------------
float pd1Min = 0, pd1Max = 1, pd2Min = 0, pd2Max = 1;   // normalization
float angleFor0 = 0, angleFor1 = 45;                    // plate angle that sends each bit
float currentAngle = 0;     // motor: exact position. manual: angle the operator reported.
long  stepPos = 0;          // motor: absolute plate position in microsteps

struct Reading {
  float pd1, pd2;           // mean over n samples
  float sd1, sd2;           // sample std-dev over n samples (0 when n == 1)
  int   n;
};

// ---------------- Small helpers ----------------
float wrapAngle(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}

// Shortest signed rotation from 'current' to 'target', in (-180, 180].
float angleDelta(float target, float current) {
  float d = wrapAngle(target) - wrapAngle(current);
  if (d > 180.0f)  d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

bool isNA(float x) { return x != x; }

void laserOn()  { digitalWrite(LASER_PIN, HIGH); }
void laserOff() { digitalWrite(LASER_PIN, LOW);  }

Reading readDetectors(int n) {
  uint64_t s1 = 0, s2 = 0, q1 = 0, q2 = 0;
  for (int i = 0; i < n; i++) {
    uint32_t a = analogRead(PHOTODIODE_1);
    uint32_t b = analogRead(PHOTODIODE_2);
    s1 += a;  s2 += b;
    q1 += (uint64_t)a * a;
    q2 += (uint64_t)b * b;
  }
  Reading r;
  r.n   = n;
  r.pd1 = (float)s1 / n;
  r.pd2 = (float)s2 / n;
  r.sd1 = 0.0f;
  r.sd2 = 0.0f;
  if (n > 1) {
    float denom = (float)n * (float)(n - 1);
    r.sd1 = sqrtf((float)((uint64_t)n * q1 - s1 * s1) / denom);
    r.sd2 = sqrtf((float)((uint64_t)n * q2 - s2 * s2) / denom);
  }
  return r;
}

float normalize(float raw, float lo, float hi) {
  if (hi <= lo) return 0.0f;
  return constrain((raw - lo) / (hi - lo), 0.0f, 1.0f);
}
float norm1(float raw) { return normalize(raw, pd1Min, pd1Max); }
float norm2(float raw) { return normalize(raw, pd2Min, pd2Max); }

float gaussianNoise(float sd) {   // Box-Muller
  float u1 = random(1, 1000001) / 1000000.0f;
  float u2 = random(1, 1000001) / 1000000.0f;
  return sd * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

// ---------------- CSV output ----------------
void fInt(bool show, long v)            { if (show) Serial.print(v);    Serial.print(','); }
void fFloat(bool show, float v, int d)  { if (show) Serial.print(v, d); Serial.print(','); }

void printCsvHeader() {
  Serial.println(F("phase,index,logical_index,repeat_index,tx_bit,ideal_angle_deg,noise_deg,"
                   "actual_angle_deg,n_avg,pd1_raw,pd2_raw,pd1_sd,pd2_sd,pd1_norm,pd2_norm,"
                   "rx_bit,t_ms"));
}

// One row per measurement, every phase. Pass -1 / NAN for fields that don't apply.
void printRow(const char *phase, long index, int logicalIdx, int repeatIdx, int txBit,
              float idealDeg, float noiseDeg, float actualDeg,
              const Reading &r, bool withNorm, int rxBit) {
  Serial.print(phase);                   Serial.print(',');
  fInt(true, index);
  fInt(logicalIdx >= 0, logicalIdx);
  fInt(repeatIdx >= 0, repeatIdx);
  fInt(txBit >= 0, txBit);
  fFloat(!isNA(idealDeg), idealDeg, 2);
  fFloat(!isNA(noiseDeg), noiseDeg, 2);
  fFloat(true, actualDeg, 2);
  fInt(true, r.n);
  fFloat(true, r.pd1, 2);
  fFloat(true, r.pd2, 2);
  fFloat(r.n > 1, r.sd1, 3);
  fFloat(r.n > 1, r.sd2, 3);
  fFloat(withNorm, norm1(r.pd1), 4);
  fFloat(withNorm, norm2(r.pd2), 4);
  fInt(rxBit >= 0, rxBit);
  Serial.println(millis());
}

// ---------------- Serial line input ----------------
char inBuf[24];
byte inLen = 0;
bool prevWasCR = false;

void flushInput() {
  while (Serial.available()) Serial.read();
  inLen = 0;
  prevWasCR = false;
}

// True when a full line has arrived; the text (possibly empty) is left in inBuf.
bool pollLine() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' && prevWasCR) { prevWasCR = false; continue; }   // CRLF counts once
    prevWasCR = (c == '\r');
    if (c == '\n' || c == '\r') {
      inBuf[inLen] = 0;
      inLen = 0;
      return true;
    }
    if (inLen < sizeof(inBuf) - 1) inBuf[inLen++] = c;
  }
  return false;
}

bool parseAngle(const char *s, float &out) {
  char *end;
  double v = strtod(s, &end);
  if (end == s) return false;
  while (*end == ' ') end++;
  if (*end != 0) return false;
  out = (float)v;
  return true;
}

// ---------------- Stored data, abort, dump ----------------
// Type  x  + Enter : stop the run, release hardware, re-print everything collected, halt.
// Type  d  + Enter : re-print everything collected so far and keep running.
struct Rec {
  Reading r;
  float ideal, noise, actual;
  byte  logicalIdx, repeatIdx, txBit, rxBit;
};
Rec stored[TOTAL_PHYSICAL_BITS];
int storedCount = 0;

void dumpStored() {
  Serial.println(F("# ---- DUMP OF STORED DATA (phase = DATA_DUMP; ignore these rows if you also have the live DATA rows) ----"));
  printCsvHeader();
  for (int i = 0; i < storedCount; i++) {
    const Rec &s = stored[i];
    printRow("DATA_DUMP", i + 1, s.logicalIdx, s.repeatIdx, s.txBit,
             s.ideal, s.noise, s.actual, s.r, true, s.rxBit);
  }
  Serial.print(F("# DUMP complete. Rows: "));
  Serial.println(storedCount);
}

void abortRun() {
  laserOff();
  digitalWrite(EN_PIN, HIGH);                 // release the motor
  Serial.println(F("# *** RUN ABORTED BY USER ***"));
  Serial.print(F("# DATA rows completed before abort: "));
  Serial.println(storedCount);
  dumpStored();
  Serial.println(F("# Reset the board to run again."));
  while (true) { }                            // halt
}

// Returns true if the line was a command (x never returns).
bool handleCommand(const char *s) {
  while (*s == ' ') s++;
  if ((s[0] == 'x' || s[0] == 'X') && s[1] == 0) abortRun();
  if ((s[0] == 'd' || s[0] == 'D') && s[1] == 0) { dumpStored(); return true; }
  return false;
}

// Motor mode: call between measurements to look for x / d typed in the Serial Monitor.
void checkCommands() {
  if (pollLine()) handleCommand(inBuf);
}

// ---------------- Calibration summary / sanity checks ----------------
bool contrastOk(float lo, float hi) { return hi > lo && (hi - lo) >= 0.3f * hi; }

void printCalibrationSummary() {
  Serial.println(F("# ---- CALIBRATION RESULT ----"));
  Serial.print(F("# angle_for_bit0_deg=")); Serial.print(angleFor0, 2);
  Serial.print(F("  angle_for_bit1_deg=")); Serial.println(angleFor1, 2);
  Serial.print(F("# pd1_min=")); Serial.print(pd1Min, 2);
  Serial.print(F("  pd1_max=")); Serial.print(pd1Max, 2);
  Serial.print(F("  pd2_min=")); Serial.print(pd2Min, 2);
  Serial.print(F("  pd2_max=")); Serial.println(pd2Max, 2);

  if (!contrastOk(pd1Min, pd1Max) || !contrastOk(pd2Min, pd2Max))
    Serial.println(F("# WARNING: dimmest and brightest readings are close together. "
                     "Check the angles you entered / the alignment."));

  float sep = fabsf(angleDelta(angleFor1, angleFor0));
  float m = fmodf(sep, 90.0f);
  if (fabsf(m - 45.0f) > 10.0f) {
    Serial.print(F("# WARNING: bit angles are "));
    Serial.print(sep, 1);
    Serial.println(F(" deg apart; a half-wave plate between S/P analyzers should give ~45 (+90k)."));
  }
}

// ---------------- MANUAL mode ----------------
// Streams one photodiode live until the operator types the plate angle where it is dimmest.
// Latches an averaged reading of both photodiodes at that moment.
float manualFindDimmest(int pd, Reading &latched) {
  float lowest = 1e9f;
  flushInput();
  Serial.print(F("# Turn the plate until PD")); Serial.print(pd);
  Serial.println(F(" is DIMMEST, then type that angle (deg) + Enter."));
  laserOn();
  delay(500);
  while (true) {
    Reading live = readDetectors(20);
    float v = (pd == 1) ? live.pd1 : live.pd2;
    if (v < lowest) lowest = v;
    Serial.print(F("# LIVE PD")); Serial.print(pd);
    Serial.print(F(" = "));       Serial.print(v, 1);
    Serial.print(F("   lowest so far = ")); Serial.println(lowest, 1);

    unsigned long t0 = millis();
    while (millis() - t0 < 250) {              // ~4 live updates per second
      if (pollLine()) {
        float ang;
        if (parseAngle(inBuf, ang)) {
          latched = readDetectors(CAL_READINGS);
          return wrapAngle(ang);
        }
        if (!handleCommand(inBuf))
          Serial.println(F("# Couldn't read that - type just the angle, e.g. 37.5"));
      }
    }
  }
}

void manualCalibrate() {
  while (true) {
    Reading atPd1Dim, atPd2Dim;
    Serial.println(F("# === MANUAL CALIBRATION (laser stays on) ==="));

    // PD1 dimmest -> all the light is in PD2's channel -> that is the bit-0 state.
    angleFor0 = manualFindDimmest(1, atPd1Dim);
    printRow("CAL_PD1_DIM", 1, -1, -1, -1, angleFor0, NAN, angleFor0, atPd1Dim, false, -1);

    // PD2 dimmest -> all the light is in PD1's channel -> that is the bit-1 state.
    angleFor1 = manualFindDimmest(2, atPd2Dim);
    printRow("CAL_PD2_DIM", 2, -1, -1, -1, angleFor1, NAN, angleFor1, atPd2Dim, false, -1);

    // The two latched readings also give each detector's min and max.
    pd1Min = atPd1Dim.pd1;  pd2Max = atPd1Dim.pd2;
    pd2Min = atPd2Dim.pd2;  pd1Max = atPd2Dim.pd1;

    laserOff();
    printCalibrationSummary();
    Serial.println(F("# Press Enter to accept, or type r + Enter to redo."));
    flushInput();
    do { while (!pollLine()) { } } while (handleCommand(inBuf));
    if (inBuf[0] != 'r' && inBuf[0] != 'R') break;
  }
  currentAngle = angleFor1;     // the plate was left at the bit-1 angle
}

// Asks the operator to set the plate. Skips the prompt if the plate is already there.
void manualSetPlate(float commanded) {
  if (fabsf(angleDelta(commanded, currentAngle)) < 0.005f) return;
  while (true) {
    flushInput();
    Serial.print(F("# SET plate to ")); Serial.print(commanded, 2);
    Serial.println(F(" deg. Enter = it's there, or type the angle you actually set + Enter."));
    while (!pollLine()) { }
    if (inBuf[0] == 0) { currentAngle = commanded; return; }
    if (handleCommand(inBuf)) continue;       // d re-prompts, x aborts
    float a;
    if (parseAngle(inBuf, a)) { currentAngle = wrapAngle(a); return; }
    Serial.println(F("# Couldn't read that."));
  }
}

// ---------------- MOTOR mode ----------------
#if USE_MOTOR
void motorPulse() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PERIOD_US / 2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_PERIOD_US / 2);
}

void moveMotorToAngle(float angleDeg) {
  long target = lround(wrapAngle(angleDeg) * (float)STEPS_PER_PLATE_REV / 360.0f)
                % STEPS_PER_PLATE_REV;
  long delta = target - stepPos;
  if (delta >  STEPS_PER_PLATE_REV / 2) delta -= STEPS_PER_PLATE_REV;   // take the short way round
  if (delta < -STEPS_PER_PLATE_REV / 2) delta += STEPS_PER_PLATE_REV;
  bool forward = (delta >= 0);
  digitalWrite(DIR_PIN, (forward != INVERT_DIR) ? HIGH : LOW);
  delayMicroseconds(5);
  for (long i = 0, n = labs(delta); i < n; i++) motorPulse();
  stepPos = target;
  currentAngle = (float)stepPos * 360.0f / (float)STEPS_PER_PLATE_REV;
}

// Enter previously measured calibration values through the Serial Monitor.
bool promptCalibrationValue(const char *label, float &value) {
  while (true) {
    flushInput();
    Serial.print(F("# Enter "));
    Serial.print(label);
    Serial.println(F(" and press Enter:"));
    while (!pollLine()) { }

    if (handleCommand(inBuf)) continue;

    if (parseAngle(inBuf, value)) return true;
    Serial.println(F("# Invalid value. Enter a number, for example 76 or 21.14."));
  }
}

void motorCalibrate() {
  Serial.println(F("# === MANUAL CALIBRATION VALUES ==="));
  Serial.println(F("# No motor calibration sweep will be performed."));
  Serial.println(F("# Enter the six values from a previous calibration."));

  while (true) {
    promptCalibrationValue("bit-0 plate angle (degrees)", angleFor0);
    promptCalibrationValue("bit-1 plate angle (degrees)", angleFor1);
    promptCalibrationValue("PD1 minimum", pd1Min);
    promptCalibrationValue("PD1 maximum", pd1Max);
    promptCalibrationValue("PD2 minimum", pd2Min);
    promptCalibrationValue("PD2 maximum", pd2Max);

    if (pd1Max > pd1Min && pd2Max > pd2Min) break;

    Serial.println(F("# ERROR: each maximum must be greater than its minimum."));
    Serial.println(F("# Please enter all six calibration values again."));
  }

  angleFor0 = wrapAngle(angleFor0);
  angleFor1 = wrapAngle(angleFor1);
  printCalibrationSummary();
}

#endif

// ---------------- Data collection ----------------
void collectData() {
  Serial.println(F("# === DATA COLLECTION ==="));
  int rows = 0;
  for (int p = 0; p < TOTAL_PHYSICAL_BITS; p++) {
    checkCommands();
    int  logicalIdx = p / REPETITIONS_PER_BIT;
    int  repeatIdx  = p % REPETITIONS_PER_BIT;
    byte txBit      = LOGICAL_BITS[logicalIdx];

    float ideal = txBit ? angleFor1 : angleFor0;
    float noise = 0.0f;
#if USE_ANGULAR_NOISE
    noise = gaussianNoise(ANGULAR_NOISE_SD_DEGREES);
#endif
    float commanded = wrapAngle(ideal + noise);

    // laser off -> move -> settle -> laser on -> read
    laserOff();
#if USE_MOTOR
    moveMotorToAngle(commanded);
#else
    manualSetPlate(commanded);
#endif
    delay(SETTLE_MS);

    laserOn();
    delay(LASER_ON_MS);
    Reading r = readDetectors(MEASURE_READINGS);
    laserOff();

    int rxBit = (norm1(r.pd1) >= norm2(r.pd2)) ? 1 : 0;

    // Keep a copy in RAM so d / x can re-print it later.
    stored[storedCount] = { r, wrapAngle(ideal), noise, currentAngle,
                            (byte)(logicalIdx + 1), (byte)(repeatIdx + 1), txBit, (byte)rxBit };
    storedCount++;

    printRow("DATA", p + 1, logicalIdx + 1, repeatIdx + 1, txBit,
             wrapAngle(ideal), noise, currentAngle, r, true, rxBit);
    rows++;
  }
  laserOff();
#if USE_MOTOR
  digitalWrite(EN_PIN, HIGH);          // release the motor
#endif
  Serial.print(F("# DONE. DATA rows printed: "));
  Serial.println(rows);
  Serial.println(F("# Reset the board to run again."));
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 10000) { }   // wait (up to 10 s) for the Serial Monitor
  delay(300);

  pinMode(PHOTODIODE_1, INPUT);
  pinMode(PHOTODIODE_2, INPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Select 1/16 microstepping, matching sketch_nov20b.ino:
  // M1 = LOW, M2 = LOW, M3 = HIGH.
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  digitalWrite(M1_PIN, LOW);
  digitalWrite(M2_PIN, LOW);
  digitalWrite(M3_PIN, HIGH);

  laserOff();
  digitalWrite(STEP_PIN, LOW);
  // Motor mode: energized. Manual mode: released, so you can turn the plate by hand.
  digitalWrite(EN_PIN, USE_MOTOR ? LOW : HIGH);

  randomSeed(analogRead(A0) ^ micros());

  Serial.println(F("# Half-wave plate data collector rev 3"));
  Serial.print(F("# mode: motor="));      Serial.print(USE_MOTOR);
  Serial.print(F(" averaging="));         Serial.print(USE_AVERAGING);
  Serial.print(F(" (n="));                Serial.print(MEASURE_READINGS);
  Serial.print(F(") angular_noise="));    Serial.print(USE_ANGULAR_NOISE);
  Serial.print(F(" (sd_deg="));           Serial.print(ANGULAR_NOISE_SD_DEGREES, 1);
  Serial.println(F(")"));
  Serial.println(F("# Commands: x + Enter = abort and dump stored data, d + Enter = dump and continue."));
#if USE_MOTOR
  Serial.println(F("# Assumes the plate is physically at your 0-degree reference right now."));
  Serial.println(F("# Calibration values will be entered manually through Serial Monitor."));
#else
  Serial.println(F("# Serial Monitor line ending must be 'Newline'."));
#endif
  printCsvHeader();

#if USE_MOTOR
  motorCalibrate();
#else
  manualCalibrate();
#endif

  collectData();
}

void loop() {
  // One complete run per reset.
}
