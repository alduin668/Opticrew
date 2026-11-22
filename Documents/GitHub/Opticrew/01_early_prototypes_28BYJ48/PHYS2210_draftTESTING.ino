#include <Stepper.h>

const int photodiode1 = A6;
const int photodiode2 = A5;
const int laser = 0;

const int maxResponsePhoto1 = 1024;
const int maxResponsePhoto2 = 1024;

const float thetaRange = 10; 

const int averageRuns = 100;

int position = 0;

#define OUTPUT1   7                
#define OUTPUT2   6
#define OUTPUT3   5
#define OUTPUT4   4 

const int stepsPerRotation = 2048; 

Stepper halfWavePlate(stepsPerRotation, OUTPUT1, OUTPUT3, OUTPUT2, OUTPUT4);  

double getGaussianRandom(double mean, double stdDev) {
  static double z1;
  static bool use_last = false;

  if (use_last) {
    use_last = false;
    return mean + z1 * stdDev;
  }

  double u1, u2, z0;
  
  do {
    u1 = random(1, 1000000) / 1000000.0;
    u2 = random(1, 1000000) / 1000000.0;
  } while (u1 <= 0.0000001); // Protective bound against floating point precision errors

  double log_u1 = log(u1);
  double sqrt_term = sqrt(-2.0 * log_u1);
  double angle_term = 2.0 * PI * u2;

  z0 = sqrt_term * cos(angle_term);
  z1 = sqrt_term * sin(angle_term); 

  use_last = true;
  
  return mean + z0 * stdDev;
}

double getUniformRandom(double range) {
  return random(-1*range, range);
}

void setup() {
  Serial.begin(9600);
  
  randomSeed(analogRead(A0)); 

  pinMode(photodiode1, INPUT);
  pinMode(photodiode2, INPUT);
  
  halfWavePlate.setSpeed(12); // check
  
  pinMode(laser, OUTPUT);

  digitalWrite(laser, LOW);
}

void loop() {

  digitalWrite(laser, LOW); // is this actually necessary??? 

  long thetaSpin = getGaussianRandom(0, 5);
  position = position - thetaSpin;
  myStepper.step(position);
  Serial.println("Angle of half-wave-plate: \n" + String(position, 2));
  
  digitalWrite(laser, HIGH);

  float photoDiode1Avg = 0;
  float photoDiode2Avg = 0;
  for (int i = 0; i < averageRuns; i++) {
    photoDiode1Avg += analogRead(photodiode1);
    photoDiode2Avg += analogRead(photodiode2);
  }
  photoDiode1Avg /= averageRuns;
  photoDiode2Avg /= averageRuns;

  photoDiode1Output = photoDiode1Avg / maxResponsePhoto1;
  photoDiode2Output = photoDiode2Avg / maxResponsePhoto2;

  Serial.println("Photodiode 1: \n" + String(photoDiode1Output, 2));
  Serial.println("Photodiode 2: \n" + String(photoDiode2Output, 2));

}
