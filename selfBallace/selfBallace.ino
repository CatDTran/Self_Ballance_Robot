//Algorithm for self ballancing robot
//PID equation: Drive = kP*Error + kI*cumulativeError + kD*dP/dt
#include "MPU6050.h"
#define kP  0//proporitonal gain (calibratable)
#define kI  0//integral gain (calibratable)
#define kD  0//derivatives gain (calibratable)
#define dt  .01//the time step over which integration is taken(in seccond)
#define setAngle  90//value to approach
//-------------------FUNCITONS PROTOTYPES--------------------------//
void complementaryFilter();
void PIDcontroller(int* drive, float* previous, float current);
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
void setup(){
  MPU6050 mpu6050Sensor();  
  Serial.begin(9600);
}
//-----------------------LOOP---------------------------------------//
void loop()
{
  
  float x = mpu6050Sensor.getAccelerationX();
  delay(1000);
}