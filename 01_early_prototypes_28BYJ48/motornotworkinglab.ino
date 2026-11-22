#include <Stepper.h>
#include <Vector.h>

const int photodiode1 = A4;
const int photodiode2 = A5;
const int laser = 8;

const int maxResponsePhoto1 = 1023;
const int maxResponsePhoto2 = 1023;

const float thetaRange = 10; 

const int averageRuns = 100;

float position = 0;

const int maxElements = 108;
bool storageArray[maxElements];
Vector<bool> data(storageArray);

#define OUTPUT1   7                
#define OUTPUT2   6
#define OUTPUT3   5
#define OUTPUT4   4 

const int stepsPerRotation = 2048; 

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

float rotatePlate(){
  Serial.println("Current Angle of half-wave-plate: \n" + String(position));
  float thetaSpin = getGaussianRandom(0, 5);
  position = position + thetaSpin;
  Serial.println("Please rotate angle by " + String(thetaSpin) + " to " + String(position) + ". You have 10 seconds.");
  delay(10000);
  Serial.println("Rotation should be complete. Continuing program.");
  return(position);
}

void setup() {
  Serial.begin(9600);
  
  randomSeed(analogRead(A0)); 

  pinMode(photodiode1, INPUT);
  pinMode(photodiode2, INPUT);
    
  pinMode(laser, OUTPUT);

  digitalWrite(laser, LOW);
  
  rotatePlate();
}

void loop() {

  float dark1 = 0;
  float dark2 = 0;
  for (int i = 0; i < averageRuns; i++) {
    dark1 += analogRead(photodiode1);
    dark2 += analogRead(photodiode2);
    delay(1000/averageRuns);
  }
  dark1 /= averageRuns;
  dark2 /= averageRuns;
  dark1 /= maxResponsePhoto1;
  dark2 /= maxResponsePhoto2;


  digitalWrite(laser, HIGH);
  delay(100);
  float photoDiode1Avg = 0;
  float photoDiode2Avg = 0;
  for (int i = 0; i < averageRuns; i++) {
    photoDiode1Avg += analogRead(photodiode1);
    photoDiode2Avg += analogRead(photodiode2);
    delay(1000/averageRuns);
  }
  photoDiode1Avg /= averageRuns;
  photoDiode2Avg /= averageRuns;

  float photoDiode1Output = photoDiode1Avg / maxResponsePhoto1;
  float photoDiode2Output = photoDiode2Avg / maxResponsePhoto2;

  Serial.println("Photodiode 1: \n" + String(photoDiode1Output) + " - " + String(dark1));
  Serial.println("Photodiode 2: \n" + String(photoDiode2Output) + " - " + String(dark2));
  bool bit = photoDiode1Output - dark1 >= photoDiode2Output - dark2;
  data.push_back(bit);
  if(data.size()%9==0){
    String print = "Logical bit complete. Current physical bit sequence: \n";
    for(bool bit : data){
      String temp = bit ? "1" : "0";
      print = print + temp;
      }
    Serial.println(print);
  }
  digitalWrite(laser, LOW);
  rotatePlate();
  delay(100);


}
