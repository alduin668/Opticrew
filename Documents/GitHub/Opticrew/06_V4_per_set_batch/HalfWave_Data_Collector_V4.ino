// ============================================================
// HALF-WAVE PLATE POLARIZATION DATA COLLECTOR  (V4, per-set batch)
//
// Edit the EXPERIMENT SETS table below, upload, open the Serial Monitor.
// One row of the table = one set. Sets run top to bottom; the runs inside
// a set run back to back. Every set has its own noise level, message,
// repetitions, calibration choice, averaging and timings.
//
// Every photodiode reading, in every phase (calibration included), is
// printed as one CSV row the moment it is taken. All status text starts
// with "# ", so a whole Serial Monitor capture loads with:
//     pandas.read_csv("run.csv", comment="#")
//
// CSV phases:  CAL (sweep points), DATA (live), DATA_DUMP (re-print from a d / x command)
// CAL rows carry the run_number of the run they come before.
// Each set prints a "# SET ..." line listing every setting it used.
//
// Serial Monitor: 9600 baud, line ending = "Newline".
//
// COMMANDS (type the single letter + Enter at any time):
//     x  = abort the batch: laser off, motor released, re-print the current run's DATA, halt
//     d  = dump the current run's DATA so far (phase = DATA_DUMP) and keep running
//   The sketch checks for commands between measurements, so there can be a
//   short delay (up to the length of the current move) before it reacts.
//
// Bit convention: bit 1 = PD1 bright / PD2 dim,
//                 bit 0 = PD1 dim / PD2 bright.
// All angles are PLATE angles in degrees (polarization turns twice as far).
// ============================================================

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// EXPERIMENT SETS  (edit this block)
// ============================================================
enum CalMethod { CAL_AUTO, CAL_TYPED };   // sweep automatically, or use the typed values below

struct ExperimentSet {
  float        noiseSdDeg;      // Gaussian SD in plate degrees (0 = no noise)
  int          runs;            // number of runs at this setting
  const char  *message;         // bits to send, e.g. "10101001101" (its length sets the logical bits)
  int          repsPerBit;      // physical bits per logical bit
  bool         calBeforeSet;    // calibrate before this set
  bool         calEveryRun;     // calibrate before every run in this set
  CalMethod    calMethod;       // CAL_AUTO or CAL_TYPED
  int          sweepPoints;     // CAL_AUTO only: points taken over the span
  float        sweepSpanDeg;    // CAL_AUTO only: plate degrees swept (180 is enough for a half-wave plate)
  int          calReadings;     // readings averaged per calibration point (CAL_AUTO)
  bool         averaging;       // average several readings per measurement
  int          readingsPerMeas; // used only when averaging = true
  unsigned int settleMs;        // wait after moving the plate, laser off
  unsigned int laserOnMs;       // laser-on time before the reading
};

// A set that does not calibrate (calBeforeSet and calEveryRun both false) reuses the last calibration.
// If the very first set has neither flag, it is calibrated anyway (there is nothing to reuse yet).
const ExperimentSet SETS[] = {
//  SD   runs  message          reps  calSet calRun method    pts  span  calRd  avg    rdgs settle laser
  {  0,   5,  "10101001101",     9,   true,  false, CAL_AUTO, 360, 360,  100,  false,  1,   300,   100 },
  { 10,   5,  "10101001101",     9,   false, false, CAL_AUTO, 360, 360,  100,  false,  1,   300,   100 },
  { 20,   3,  "10101001101",     9,   false, false, CAL_AUTO, 360, 360,  100,  false,  1,   300,   100 },
  { 30,   3,  "10101001101",     9,   false, false, CAL_AUTO, 360, 360,  100,  false,  1,   300,   100 },
  { 40,   3,  "10101001101",     9,   false, false, CAL_AUTO, 360, 360,  100,  false,  1,   300,   100 },
  { 45,   3,  "10101001101",     9,   false, false, CAL_AUTO, 360, 360,  100,  false,  1,   300,   100 },
};

// Used only by sets with CAL_TYPED (raw ADC units, same as the CAL rows report).
const float TYPED_ANGLE_BIT0 = 0.0f;
const float TYPED_ANGLE_BIT1 = 45.0f;
const float TYPED_PD1_MIN = 0.0f, TYPED_PD1_MAX = 1023.0f;
const float TYPED_PD2_MIN = 0.0f, TYPED_PD2_MAX = 1023.0f;

// ============================================================
// HARDWARE  (fixed; change only if the wiring or mechanics change)
// ============================================================
const int PHOTODIODE_1 = A4;
const int PHOTODIODE_2 = A5;
const int LASER_PIN    = 8;       // HIGH = laser on

const int STEP_PIN = 6;           // DRV8825 STEP
const int DIR_PIN  = 5;           // DRV8825 DIR
const int EN_PIN   = 7;           // DRV8825 /ENABLE (LOW = energized)

// DRV8825 microstep pins: 1/16 mode = M1 low, M2 low, M3 high
const int M1_PIN = 4;
const int M2_PIN = 3;
const int M3_PIN = 2;

const long  MOTOR_FULL_STEPS_PER_REV = 200;    // NEMA17, 1.8 deg per full step
const int   MICROSTEPS               = 16;     // must match the M1/M2/M3 levels set in setup()
const float GEAR_RATIO               = 4.0f;   // motor turns per plate turn (20T pinion -> 80T ring)
                                               // set 1.0 if the plate is direct drive
const bool  INVERT_DIR               = false;  // flip if positive angles run the wrong way
const unsigned int STEP_PERIOD_US    = 800;    // time per microstep; raise it if the motor stalls
const long STEPS_PER_PLATE_REV =
  (long)(MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO + 0.5f);

// Buffer sizes (RAM). The table is checked against these at startup.
const int MAX_PHYSICAL_BITS = 200;   // largest message length x repsPerBit allowed in any set
const int MAX_SWEEP_POINTS  = 360;   // largest sweepPoints allowed in any set

// ---------------- State ----------------
float pd1Min = 0, pd1Max = 1, pd2Min = 0, pd2Max = 1;   // normalization
float angleFor0 = 0, angleFor1 = 45;                    // plate angle that sends each bit
float currentAngle = 0;     // exact plate position
long  stepPos = 0;          // absolute plate position in microsteps

// Which set / run the CSV rows belong to.
int   activeSetIndex  = 0;
int   activeRunNumber = 0;
float activeNoiseSD   = 0.0f;

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
  Serial.println(F("phase,set_index,run_number,gaussian_sd_deg,index,logical_index,repeat_index,"
                   "tx_bit,ideal_angle_deg,noise_deg,actual_angle_deg,n_avg,pd1_raw,pd2_raw,"
                   "pd1_sd,pd2_sd,pd1_norm,pd2_norm,rx_bit,t_ms"));
}

// One row per measurement, every phase. Pass -1 / NAN for fields that don't apply.
void printRow(const char *phase, long index, int logicalIdx, int repeatIdx, int txBit,
              float idealDeg, float noiseDeg, float actualDeg,
              const Reading &r, bool withNorm, int rxBit) {
  Serial.print(phase);                   Serial.print(',');
  fInt(true, activeSetIndex);
  fInt(true, activeRunNumber);
  fFloat(true, activeNoiseSD, 2);
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

// ---------------- Stored data, abort, dump ----------------
// Holds the CURRENT run only (cleared at the start of each run).
struct Rec {
  Reading r;
  float ideal, noise, actual;
  byte  logicalIdx, repeatIdx, txBit, rxBit;
};
Rec stored[MAX_PHYSICAL_BITS];
int storedCount = 0;

void dumpStored() {
  Serial.println(F("# ---- DUMP OF STORED DATA, CURRENT RUN (phase = DATA_DUMP; ignore these rows if you also have the live DATA rows) ----"));
  Serial.print(F("# "));                    // header goes out as a comment so pandas skips it
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
  Serial.println(F("# *** BATCH ABORTED BY USER ***"));
  Serial.print(F("# Aborted in set ")); Serial.print(activeSetIndex);
  Serial.print(F(", run "));            Serial.println(activeRunNumber);
  Serial.print(F("# DATA rows completed in this run before abort: "));
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

// Call between measurements to look for x / d typed in the Serial Monitor.
void checkCommands() {
  if (pollLine()) handleCommand(inBuf);
}

// ---------------- Table check (runs once at startup) ----------------
void failSet(int row, const __FlashStringHelper *msg) {
  laserOff();
  Serial.print(F("# ERROR in SETS row ")); Serial.print(row + 1);
  Serial.print(F(": "));                   Serial.println(msg);
  Serial.println(F("# Fix the table and re-upload. Nothing was run."));
  while (true) { }
}

const int NUM_SETS = sizeof(SETS) / sizeof(SETS[0]);

void validateSets() {
  bool usesTyped = false;
  for (int i = 0; i < NUM_SETS; i++) {
    const ExperimentSet &s = SETS[i];
    if (s.noiseSdDeg < 0.0f)                failSet(i, F("noiseSdDeg must be 0 or more"));
    if (s.runs < 1)                         failSet(i, F("runs must be at least 1"));
    if (s.message == NULL || s.message[0] == 0) failSet(i, F("message is empty"));
    for (const char *c = s.message; *c; c++)
      if (*c != '0' && *c != '1')           failSet(i, F("message may only contain 0 and 1"));
    if (s.repsPerBit < 1)                   failSet(i, F("repsPerBit must be at least 1"));
    if ((long)strlen(s.message) * s.repsPerBit > MAX_PHYSICAL_BITS)
                                            failSet(i, F("message length x repsPerBit is over MAX_PHYSICAL_BITS"));
    if (s.averaging && s.readingsPerMeas < 1) failSet(i, F("readingsPerMeas must be at least 1"));
    if (s.calMethod == CAL_AUTO) {
      if (s.sweepPoints < 2 || s.sweepPoints > MAX_SWEEP_POINTS)
                                            failSet(i, F("sweepPoints must be 2..MAX_SWEEP_POINTS"));
      if (s.sweepSpanDeg <= 0.0f)           failSet(i, F("sweepSpanDeg must be above 0"));
      if (s.calReadings < 1)                failSet(i, F("calReadings must be at least 1"));
    } else {
      usesTyped = true;
    }
  }
  if (usesTyped && (TYPED_PD1_MAX <= TYPED_PD1_MIN || TYPED_PD2_MAX <= TYPED_PD2_MIN))
    failSet(0, F("typed calibration: each max must be above its min"));
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
                     "Check the alignment / the typed values."));

  float sep = fabsf(angleDelta(angleFor1, angleFor0));
  float m = fmodf(sep, 90.0f);
  if (fabsf(m - 45.0f) > 10.0f) {
    Serial.print(F("# WARNING: bit angles are "));
    Serial.print(sep, 1);
    Serial.println(F(" deg apart; a half-wave plate between S/P analyzers should give ~45 (+90k)."));
  }
}

// ---------------- Motor ----------------
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

// ---------------- Calibration ----------------
static float calPd1[MAX_SWEEP_POINTS], calPd2[MAX_SWEEP_POINTS];

// One sweep over the set's span. Every point is printed as it is taken.
void autoCalibrate(const ExperimentSet &s) {
  Serial.println(F("# === CALIBRATION SWEEP ==="));
  const float step = s.sweepSpanDeg / s.sweepPoints;
  laserOn();
  delay(500);
  for (int i = 0; i < s.sweepPoints; i++) {
    checkCommands();
    float a = i * step;
    moveMotorToAngle(a);
    delay(s.settleMs);
    Reading r = readDetectors(s.calReadings);
    calPd1[i] = r.pd1;
    calPd2[i] = r.pd2;
    printRow("CAL", i + 1, -1, -1, -1, a, NAN, currentAngle, r, false, -1);
  }
  laserOff();

  pd1Min = pd1Max = calPd1[0];
  pd2Min = pd2Max = calPd2[0];
  int i1min = 0;
  for (int i = 1; i < s.sweepPoints; i++) {
    if (calPd1[i] < pd1Min) { pd1Min = calPd1[i]; i1min = i; }
    if (calPd1[i] > pd1Max)   pd1Max = calPd1[i];
    if (calPd2[i] < pd2Min)   pd2Min = calPd2[i];
    if (calPd2[i] > pd2Max)   pd2Max = calPd2[i];
  }
  angleFor0 = i1min * step;                    // PD1 dimmest -> bit 0

  // PD2 dimmest, but only within 60 deg of the bit-0 angle. The curves repeat every 90 deg,
  // so this picks the nearest null and keeps the plate travel between bits to ~45 deg.
  int i2min = i1min;
  for (int i = 0; i < s.sweepPoints; i++) {
    if (fabsf(angleDelta(i * step, angleFor0)) <= 60.0f && calPd2[i] < calPd2[i2min]) i2min = i;
  }
  angleFor1 = i2min * step;                    // PD2 dimmest -> bit 1

  printCalibrationSummary();
}

void typedCalibrate() {
  Serial.println(F("# === TYPED CALIBRATION VALUES (no sweep) ==="));
  angleFor0 = wrapAngle(TYPED_ANGLE_BIT0);
  angleFor1 = wrapAngle(TYPED_ANGLE_BIT1);
  pd1Min = TYPED_PD1_MIN;  pd1Max = TYPED_PD1_MAX;
  pd2Min = TYPED_PD2_MIN;  pd2Max = TYPED_PD2_MAX;
  printCalibrationSummary();
}

void calibrate(const ExperimentSet &s) {
  if (s.calMethod == CAL_AUTO) autoCalibrate(s);
  else                         typedCalibrate();
}

// ---------------- One run ----------------
void runOnce(const ExperimentSet &s) {
  const int nLogical = (int)strlen(s.message);
  const int total    = nLogical * s.repsPerBit;
  const int measN    = s.averaging ? s.readingsPerMeas : 1;
  int errors = 0;
  storedCount = 0;                      // d / x apply to this run only

  Serial.println(F("# === DATA COLLECTION ==="));
  for (int p = 0; p < total; p++) {
    checkCommands();
    int  logicalIdx = p / s.repsPerBit;
    int  repeatIdx  = p % s.repsPerBit;
    byte txBit      = (s.message[logicalIdx] == '1') ? 1 : 0;

    float ideal = txBit ? angleFor1 : angleFor0;
    float noise = 0.0f;
    if (s.noiseSdDeg > 0.0f) noise = gaussianNoise(s.noiseSdDeg);
    float commanded = wrapAngle(ideal + noise);

    // laser off -> move -> settle -> laser on -> read
    laserOff();
    moveMotorToAngle(commanded);
    delay(s.settleMs);

    laserOn();
    delay(s.laserOnMs);
    Reading r = readDetectors(measN);
    laserOff();

    int rxBit = (norm1(r.pd1) >= norm2(r.pd2)) ? 1 : 0;
    if (rxBit != txBit) errors++;

    // Keep a copy in RAM so d / x can re-print it later.
    stored[storedCount] = { r, wrapAngle(ideal), noise, currentAngle,
                            (byte)(logicalIdx + 1), (byte)(repeatIdx + 1), txBit, (byte)rxBit };
    storedCount++;

    printRow("DATA", p + 1, logicalIdx + 1, repeatIdx + 1, txBit,
             wrapAngle(ideal), noise, currentAngle, r, true, rxBit);
  }
  laserOff();

  Serial.print(F("# RUN ")); Serial.print(activeRunNumber);
  Serial.print(F(" DONE. DATA rows: ")); Serial.print(total);
  Serial.print(F("  raw bit errors: "));  Serial.print(errors);
  Serial.print(F(" / "));                 Serial.println(total);
}

// ---------------- Batch ----------------
void printSetBanner(int si, const ExperimentSet &s) {
  Serial.println(F("# ----------------------------------------"));
  Serial.print(F("# SET "));            Serial.print(si + 1);
  Serial.print(F(" / "));              Serial.print(NUM_SETS);
  Serial.print(F(" | noise_sd="));      Serial.print(s.noiseSdDeg, 2);
  Serial.print(F(" runs="));           Serial.print(s.runs);
  Serial.print(F(" message="));        Serial.print(s.message);
  Serial.print(F(" reps="));           Serial.println(s.repsPerBit);
  Serial.print(F("#   calBeforeSet=")); Serial.print(s.calBeforeSet);
  Serial.print(F(" calEveryRun="));    Serial.print(s.calEveryRun);
  Serial.print(F(" calMethod="));      Serial.print(s.calMethod == CAL_AUTO ? "auto" : "typed");
  Serial.print(F(" sweepPoints="));    Serial.print(s.sweepPoints);
  Serial.print(F(" sweepSpan="));      Serial.print(s.sweepSpanDeg, 1);
  Serial.print(F(" calReadings="));    Serial.println(s.calReadings);
  Serial.print(F("#   averaging="));   Serial.print(s.averaging);
  Serial.print(F(" readingsPerMeas="));Serial.print(s.averaging ? s.readingsPerMeas : 1);
  Serial.print(F(" settleMs="));       Serial.print(s.settleMs);
  Serial.print(F(" laserOnMs="));      Serial.println(s.laserOnMs);
}

void runBatch() {
  int  globalRun  = 0;
  bool calibrated = false;

  for (int si = 0; si < NUM_SETS; si++) {
    const ExperimentSet &s = SETS[si];
    activeSetIndex = si + 1;
    activeNoiseSD  = s.noiseSdDeg;
    printSetBanner(si, s);

    bool justCalibrated = false;
    if (s.calBeforeSet || !calibrated) {
      if (!s.calBeforeSet)
        Serial.println(F("# No calibration exists yet, so this set is calibrated anyway."));
      activeRunNumber = globalRun + 1;          // CAL rows belong to the run that follows
      calibrate(s);
      calibrated = true;
      justCalibrated = true;
    }

    for (int run = 0; run < s.runs; run++) {
      globalRun++;
      activeRunNumber = globalRun;
      if (s.calEveryRun && !justCalibrated) calibrate(s);
      justCalibrated = false;

      digitalWrite(EN_PIN, LOW);                // make sure the motor is energized
      Serial.print(F("# START RUN ")); Serial.print(globalRun);
      Serial.print(F(" (set "));       Serial.print(si + 1);
      Serial.print(F(", run "));       Serial.print(run + 1);
      Serial.print(F(" of "));         Serial.print(s.runs);
      Serial.print(F(") noise_sd="));  Serial.println(s.noiseSdDeg, 2);
      runOnce(s);
    }
  }

  laserOff();
  digitalWrite(EN_PIN, HIGH);                   // release the motor after the last run
  Serial.println(F("# ========================================"));
  Serial.print(F("# BATCH COMPLETE. Runs: ")); Serial.println(globalRun);
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
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  digitalWrite(M1_PIN, LOW);                     // 1/16 microstepping
  digitalWrite(M2_PIN, LOW);
  digitalWrite(M3_PIN, HIGH);
  laserOff();
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, LOW);                     // energized

  randomSeed(analogRead(A0) ^ micros());

  long totalRuns = 0, totalMeasurements = 0;
  for (int i = 0; i < NUM_SETS; i++) {
    totalRuns         += SETS[i].runs;
    totalMeasurements += (long)SETS[i].runs * (long)strlen(SETS[i].message) * SETS[i].repsPerBit;
  }

  Serial.println(F("# Half-wave plate data collector V4 (per-set batch)"));
  Serial.print(F("# sets: "));            Serial.print(NUM_SETS);
  Serial.print(F("  runs: "));            Serial.print(totalRuns);
  Serial.print(F("  DATA measurements: "));Serial.println(totalMeasurements);
  Serial.println(F("# Commands: x + Enter = abort and dump current run, d + Enter = dump current run and continue."));
  Serial.println(F("# Assumes the plate is physically at your 0-degree reference right now."));
  validateSets();
  printCsvHeader();

  runBatch();
}

void loop() {
  // One complete batch per reset.
}
