//this header file include a class definition for MPU6050 accelerometer which makes
//reading data from MPU6050 accelerometer much easier
//Author: Cat Tran
//email: trandinhcat@gmail.com
#include <Wire.h>
#include <math.h>
#define PI  3.14159265359
const int ADDRESS_MPU=0x68;  // I2C address of the MPU-6050
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
		float initialAngleX, initialAngleY, initialAngleZ;
		float filteredAngleX, filteredAngleY, filteredAngleZ;
		float static gyroscopeLastValue;
		float static gyroscopeCurrentValue;
		float static gyroscopeFilteredAngle;
		float static complementaryAngle;//the current measured angle
		float static previousAngle;//store value read from previous loop
		float dt;
	public:
		//CONSTRUCTOR------//

    	//INITIALIZE MPU6050
	    void initializeMPU6050()
	    {
	      //Waking up MPU6050
	      Wire.begin();
	      Wire.beginTransmission(ADDRESS_MPU);
	      Wire.write(0x6B);  // PWR_MGMT_1 register
	      Wire.write(0);     // set to zero (wakes up the MPU-6050)
	      Wire.endTransmission(true);
	    }
		//------GETTERS------//
		//Get and convert acceleration in X direction
		float getAccelerationX()
		{
			readMPU6050();
			accelerationX = (float)AcX / (float)16384;
			return accelerationX;
		}
		//Get and convert acceleration in Y direction
		float getAccelerationY()
		{
			readMPU6050();
			accelerationY = (float)AcY / (float)16384;
			return accelerationY;
		}
		//Get and convert acceleration in Z direction
		float getAccelerationZ()
		{
			readMPU6050();
			accelerationZ = (float)AcZ / (float)16384;
			return accelerationZ;
		}

		//Get and convert angular in X
		float getAngleX()
		{
			readMPU6050();
			complementaryFilterX();
			return angleX;
		}

		//Get and convert angle in Y
		float getAngleY()
		{
			readMPU6050();
			//angleY = angleY + ( (float) GyY / 131.0 ) * dt;
			//angleY = (float) GyY/131.0;
			complementaryFilterY();
			return angleY;
		}
		//Get and convert angle in Z
		float getAngleZ()
		{
			readMPU6050();
			complementaryFilterZ();
			return angleZ;
		}
		//Get raw gyro data
		int16_t getAngleRawX()
		{
			readMPU6050();
			return GyX;
		}
		int16_t getAngleRawY()
		{
			readMPU6050();
			return GyY;
		}
		int16_t getAngleRawZ()
		{
			readMPU6050();
			return GyZ;
		}
		//------SETTER------------//
		//Set sampletime
		void setSampleTime(float time)
		{
			dt = (float) time / 1000.0;
		}
		//Set initial X angle
		void initializeAngleX()
		{
			angleX = 0;
		}
		//Set initial Y angle
		void initializeAngleY()
		{
			//initialAngleY = getAngleY();
			angleY = 0;
		}
		//Set initial Z angle
		void initializeAngleZ()
		{
			angleZ = 0;
		}
		//--------EXTRA MEMEBER FUNCTIONS-----------//
		void complementaryFilterX()
		{
			float angleAccelerationTerm = atan2((double) getAccelerationZ(), (double) getAccelerationY()) * 180/ PI;
			angleX = 0.98 *(angleAccelerationTerm + (GyX * dt)/131.0) + 0.02 * angleAccelerationTerm;
		}
		void complementaryFilterY()
		{
			float angleAccelerationTerm = atan2((double) getAccelerationX(), (double) getAccelerationZ()) * 180/ PI;
			angleY = 0.98 *(angleAccelerationTerm + (GyY * dt)/131.0) + 0.02 * angleAccelerationTerm;
		}
		void complementaryFilterZ()
		{
			float angleAccelerationTerm = atan2((double) getAccelerationX(), (double) getAccelerationY()) * 180/ PI;
			angleZ = 0.98 *(angleAccelerationTerm + (GyZ * dt)/131.0) + 0.02 * angleAccelerationTerm;
		}
		//Called whenever need the data from MPU6050
		void readMPU6050()
		{
			Wire.beginTransmission(ADDRESS_MPU);
			Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
			Wire.endTransmission(false);
			Wire.requestFrom(ADDRESS_MPU,14,true);  // request a total of 14 registers
			//Read data from MPU6050
			AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
			AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
			AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
			Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
			GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
			GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
			GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
		}
};
