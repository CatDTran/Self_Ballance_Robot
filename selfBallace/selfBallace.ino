//Author: Cat Tran
//email: trandinhcat@gmail.com
//Algorithm for self ballancing robot
//PID equation: Drive = kP*Error + kI*cumulativeError + kD*dP/

#include "MPU6050.h"
#include <PID_v1.h>

#define kP  0//proporitonal gain (calibratable)
#define kI  0//integral gain (calibratable)
#define kD  0//derivatives gain (calibratable)
#define dt  .01//the time step over which integration is taken(in seccond)
#define setAngle  90//value to approach
#define SAMPLE_TIME 200
//==================DECLARE SOME GLOBAL VARIABLES===================//
int PWM;//pulse width modulation that will be send to controll motors' speed
float static gyroscopeLastValue = 0;
float static gyroscopeCurrentValue = 0;
float static gyroscopeFilteredAngle = 0;
float static complementaryAngle;//the current measured angle
float static previousAngle;//store value read from previous loop
MPU6050 mpu6050Sensor;
//PID controllerPID(mpu6050Sensor.getAccelerationY(), );
//===================SETUP=========================================//
void setup()
{
  mpu6050Sensor.initializeMPU6050();//waking up mpu6050
  mpu6050Sensor.setSampleTime(SAMPLE_TIME);
  Serial.begin(9600);
}
//=======================LOOP=======================================//
void loop()
{ 
  float angleY = mpu6050Sensor.getAngleY();
  Serial.println(angleY);
  delay(SAMPLE_TIME);
}
