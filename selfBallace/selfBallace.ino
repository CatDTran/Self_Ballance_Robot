//Author: Cat Tran
//email: trandinhcat@gmail.com
//Algorithm for self ballancing robot
//PID equation: Drive = kP*Error + kI*cumulativeError + kD*dP/

#include "MPU6050.h"
#include <PID_v1.h>

#define Kp  10//proporitonal gain (calibratable)
#define Ki  0.0//integral gain (calibratable)
#define Kd  0.0//derivatives gain (calibratable)
#define SAMPLE_TIME 100
#define PID_SAMPLE_TIME 200
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
PID controllerPID(&angleY, &pwm, &setAngle, Kp, Ki, Kd, DIRECT);
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
  //Initialize pid controller
  controllerPID.SetOutputLimits(0, 255);
  controllerPID.SetMode(AUTOMATIC);
  controllerPID.SetSampleTime(PID_SAMPLE_TIME);
  controllerPID.SetControllerDirection(DIRECT);
  Serial.begin(9600);
}
//=======================LOOP=======================================//
bool computed;
void loop()
{ 
  angleY = (double) mpu6050Sensor.getAngleY();
  computed = controllerPID.Compute();
  //drive motor A
  digitalWrite(motorA.directionControl_1, HIGH);
  digitalWrite(motorA.directionControl_2, LOW);
  analogWrite(motorA.speedPWM, pwm);
  //drive motor B
  digitalWrite(motorB.directionControl_1, HIGH);
  digitalWrite(motorB.directionControl_2, LOW);
  analogWrite(motorB.speedPWM, pwm);
  if(computed)
    Serial.print("computed");
  Serial.print("  ");
  Serial.print(pwm);
  Serial.print( "   ")  ;
  Serial.println(angleY);
  delay(SAMPLE_TIME);
}
