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
  			Wire.beginTransmission(MPU_addr);
			Wire.write(0x6B);  // PWR_MGMT_1 register
			Wire.write(0);     // set to zero (wakes up the MPU-6050)
			Wire.endTransmission(true);
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