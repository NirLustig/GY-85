#include <Wire.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <math.h>
#include <HMC5883L.h>

ITG3200 gyro;
ADXL345 accel;
HMC5883L magno;

float rollMeasured; //Rotation x
float pitchMeasured; //Rotation y
float yaw=0; //Rotation z

float rollOld=0;
float rollNew; 
float pitchOld=0; 
float pitchNew; 

float PitchFusedRad;
float RollFusedRad;

float rollVelGyro=0;
float pitchVelGyro=0;
float yawVelGyro=0;
float dt;

float RollFused=0;
float PitchFused=0;
float YawFused=0;

float Xmag;
float Ymag;
unsigned long millisOld;

bool justTurnedOn = true; // Boolean to check if Arduino just turned on
float offsetX = 0, offsetY = 0, offsetZ = 0;
float axOffset = 0, ayOffset = 0, azOffset = 0;
float offsetMx=0,offsetMy=0,offsetMz=0;
int numReadings = 500; // Number of readings to take for calibration

void setup() {
  Serial.begin(9600);
  Wire.begin();
  gyro.initialize();
  accel.initialize();
  millisOld=millis();
  magno.initialize();
}

void loop() {
  //IF first started it runs the calibrations
  if (justTurnedOn) {
    CalibrateSensors();
    justTurnedOn = false; // Set to false after calibration
  }
  
  //Get info from Gyro and Acc
  int16_t gx, gy, gz;
  int16_t ax, ay, az;
  int16_t magnoX, magnoY, magnoZ;
  gyro.getRotation(&gx, &gy, &gz);
  accel.getAcceleration(&ax, &ay, &az);
  magno.getHeading(&magnoX, &magnoY, &magnoZ);


  //Calculates the real values 
  float correctedGx = (gx / 14.375) - offsetX;
  float correctedGy = (gy / 14.375) - offsetY;
  float correctedGz = (gz / 14.375) - offsetZ;

  float correctedAx = (ax / 256.0 - axOffset);
  float correctedAy = (ay / 256.0 - ayOffset);
  float correctedAz = (az / 256.0 - azOffset);

  float correctedMx = (magnoX - axOffset);
  float correctedMy = (magnoY - ayOffset);
  float correctedMz = (magnoZ - azOffset);


  //Calculates roll & pitch using accelemeter
  rollMeasured=atan2(correctedAy,correctedAz)*(180/3.14);
  pitchMeasured=atan2(correctedAx,correctedAz)*(180/3.14);

  //Uses low pass filter 
  rollNew=.9*rollOld+.1*rollMeasured;
  pitchNew=.9*pitchOld+.1*pitchMeasured;
  
  //Calculating roll & Pitch using Gyro (Angular Velocity)
  dt=(millis()-millisOld)/1000.;
  millisOld=millis(); // for next loop
  rollVelGyro=rollVelGyro+correctedGx*dt;
  pitchVelGyro=pitchVelGyro+correctedGy*dt;

  //Sensor Fusion
  RollFused=(RollFused+correctedGx*dt)*0.95+rollMeasured*0.05;
  PitchFused=(PitchFused+correctedGy*dt)*0.95+pitchMeasured*0.05;
 
  //Calculates Yaw using magnometer
  PitchFusedRad=pitchMeasured*(3.14/180);
  RollFusedRad=rollMeasured*(3.14/180);

  Xmag=correctedMx*cos(PitchFusedRad)-correctedMy*sin(PitchFusedRad)*sin(RollFusedRad)-correctedMz*sin(PitchFusedRad)*cos(RollFusedRad);
  Ymag=correctedMy*cos(RollFusedRad)+correctedMz*sin(RollFusedRad);
  yaw=atan2(magnoY,magnoX)*(180/3.14);

  Serial.print(rollMeasured);
  Serial.print(",");
  Serial.print(pitchMeasured);
  Serial.print(",");
  Serial.println(yaw);

  //Place old value as the new for the next run.
  rollOld=rollNew;
  pitchOld=pitchNew;
  delay(200);
}

void CalibrateSensors() {
  long sumGx = 0, sumGy = 0, sumGz = 0;
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumMx = 0, sumMy = 0, sumMz = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) { // Run this loop for 5 seconds
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    int16_t mx, my, mz;
    gyro.getRotation(&gx, &gy, &gz);
    accel.getAcceleration(&ax, &ay, &az);
    magno.getHeading(&mx, &my, &mz);
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += mx;
    sumGy += my;
    sumGz += mz;
    delay(10);
  }

  offsetMx = sumMx/ (float)numReadings;
  offsetMy = sumMy/ (float)numReadings;
  offsetMz = sumMz/ (float)numReadings;
  offsetX = sumGx / (float)numReadings / 14.375;
  offsetY = sumGy / (float)numReadings / 14.375;
  offsetZ = sumGz / (float)numReadings / 14.375;
  axOffset = sumAx / (float)numReadings / 256.0;
  ayOffset = sumAy / (float)numReadings / 256.0;
  azOffset = sumAz / (float)numReadings / 256.0 - 1.0; // Adjust for gravity on Z-axis (assuming 1g = 256 LSB)
}
