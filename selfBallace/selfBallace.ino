//Author: Cat Tran
//email: trandinhcat@gmail.com
//Algorithm for self ballancing robot
//PID equation: Drive = kP*Error + kI*cumulativeError + kD*dP/

#include "MPU6050.h"
#include <PID_v1.h>

#define Kp  0.0//proporitonal gain (calibratable)
#define Ki  0.0//integral gain (calibratable)
#define Kd  0.0//derivatives gain (calibratable)
#define SAMPLE_TIME 200
//==================DECLARE SOME GLOBAL VARIABLES===================//
double pwm;
double setAngle = 0;
double angleY;
//struct for motors pins out
struct Motor{
  public:
    int directionControl_1;
    int directionControl_2;
    int speedPWM;
};
//initialize pins out for both motors
MPU6050 mpu6050Sensor;
  struct Motor motorA = {2,3,9}, motorB = {4,5,10};
//===================SETUP=========================================//
void setup()
{  
  mpu6050Sensor.initializeMPU6050();//waking up mpu6050
  mpu6050Sensor.setSampleTime(SAMPLE_TIME);//set sample time for complementary filter

  //set pins mode; all are output, obviously!!
  pinMode(motorA.directionControl_1, OUTPUT);
  pinMode(motorA.directionControl_2, OUTPUT);
  pinMode(motorA.speedPWM, OUTPUT);
  pinMode(motorB.directionControl_1, OUTPUT);
  pinMode(motorB.directionControl_2, OUTPUT);
  pinMode(motorA.speedPWM, OUTPUT);

  PID controllerPID(&angleY, &pwm, &setAngle, Kp, Ki, Kd, DIRECT);//instantiate pid controller
  Serial.begin(9600);
}
//=======================LOOP=======================================//
void loop()
{ 
  digitalWrite(motorA.directionControl_1, HIGH);
  digitalWrite(motorA.directionControl_2, LOW);
  analogWrite(motorA.speedPWM, 100);

  digitalWrite(motorB.directionControl_1, HIGH);
  digitalWrite(motorB.directionControl_2, LOW);
  analogWrite(motorB.speedPWM, 100);

  float angleY = mpu6050Sensor.getAngleY();
  Serial.println(angleY);
  delay(SAMPLE_TIME);
}
