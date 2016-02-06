//this header file include a class definition for MPU6050 accelerometer which makes
//reading data from MPU6050 accelerometer much easier

#include <Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
class MPU6050{
	private:
		int16_t AcX;
		int16_t AcY;
		int16_t AcZ;
		int16_t Tmp;
		int16_t GyX;
		int16_t GyY;
		int16_t GyZ;//range from -32,768 to 32,767
		float accelerationX, accelerationY, accelerationZ;
		float angleX, angleY, angleZ;
		float static gyroscopeLastValue;
		float static gyroscopeCurrentValue;
		float static gyroscopeFilteredAngle;
		float static complementaryAngle;//the current measured angle
		float static previousAngle;//store value read from previous loop
		float dt;
	public:
		//CONSTRUCTOR
		MPU6050()
		{
			  Wire.begin();
			  Wire.beginTransmission(MPU);
			  Wire.write(0x6B);  // PWR_MGMT_1 register
			  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
			  Wire.endTransmission(false);
			  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
			  //Read data from MPU6050
			  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
			  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
			  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
			  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
			  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
			  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
			  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
			  //converting data to measurement
			  accelerationX = (float)AcX/(float)16384;
			  accelerationY = (float)AcY/(float)16384;
			  accelerationZ = (float)AcZ/(float)16384;
			  angleX = (float)GyX / 182.0;
			  angleY = (float)GyY / 182.0;
			  angleZ = (float)GyZ / 182.0;
		}

		//getters
		float getAccelerationX()
		{
			AcX=Wire.read()<<8|Wire.read();
			accelerationX = (float)AcX/(float)16384;
			return accelerationX;
		}

		void complementaryFilter()
		{
			  gyroscopeCurrentValue = GyZ;//get raw value from gyroscope sensor  
			  float gyroscopeAngleDifference = (((float)gyroscopeCurrentValue - (float)gyroscopeLastValue)/131)*dt;//then compute the difference berween last and current value, and convert to degree
			  float accelerometerAngle = (AcZ /182.0);//calculate angle from accelerometer raw data
			  gyroscopeFilteredAngle += gyroscopeAngleDifference;
			  complementaryAngle = 0.95*(gyroscopeFilteredAngle) + 0.05*accelerometerAngle;
			  gyroscopeLastValue = gyroscopeCurrentValue;
		}

};