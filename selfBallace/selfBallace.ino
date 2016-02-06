//Algorithm for self ballancing robot
//PID equation: Drive = kP*Error + kI*cumulativeError + kD*dP/dt
#include "MPU6050.h"
#define kP  0//proporitonal gain (calibratable)
#define kI  0//integral gain (calibratable)
#define kD  0//derivatives gain (calibratable)
#define dt  .01//the time step over which integration is taken(in seccond)
#define setAngle  90//value to approach
//------------------DEFINE SOME GLOBAL VERIABLES-------------------//
int16_t integral = 0;
int16_t error = 0;
int16_t P = 0;
int16_t I = 0;
int16_t D = 0;
int16_t integralThreshold;
int PWM;//pulse width modulation that will be send to controll motors' speed
float static gyroscopeLastValue = 0;
float static gyroscopeCurrentValue = 0;
float static gyroscopeFilteredAngle = 0;
float static complementaryAngle;//the current measured angle
float static previousAngle;//store value read from previous loop
//-------------------SETUP-----------------------------------------//
MPU6050 mpu6050Sensor;
void setup(){
  mpu6050Sensor = MPU6050();
  mpu6050Sensor.initializeMPU6050();
  Serial.begin(9600);
}
//-----------------------LOOP---------------------------------------//
void loop()
{  
  float x = mpu6050Sensor.getAccelerationX();
  float y = mpu6050Sensor.getAccelerationY();
  float z = mpu6050Sensor.getAccelerationZ();
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);
  delay(500);
}
