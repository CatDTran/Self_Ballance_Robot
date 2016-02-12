//Author: Cat Tran
//email: trandinhcat@gmail.com
//Algorithm for self ballancing robot
//PID equation: Drive = kP*Error + kI*cumulativeError + kD*dP/

#include "MPU6050.h"
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
//===================SETUP=========================================//
void setup()
{
  mpu6050Sensor = MPU6050();
  mpu6050Sensor.initializeMPU6050();//waking up mpu6050
  mpu6050Sensor.initializeAngleX();
  mpu6050Sensor.initializeAngleY();
  mpu6050Sensor.initializeAngleZ();
  mpu6050Sensor.setSampleTime(SAMPLE_TIME);
  Serial.begin(9600);
}
//=======================LOOP=======================================//
void loop()
{  
  float x = mpu6050Sensor.getAccelerationX();
  float y = mpu6050Sensor.getAccelerationY();
  float z = mpu6050Sensor.getAccelerationZ();
  float angleX = mpu6050Sensor.getAngleX();
  float angleY = mpu6050Sensor.getAngleY();
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.print("  ||  ");
  Serial.println(angleY);
  delay(SAMPLE_TIME);
}
