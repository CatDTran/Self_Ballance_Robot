//PID controller class
//
class PICController{
private:
	float mPropotionalCoeffient = 0;//K
	float mIntegralCoefficient = 0;//I
	float mDerivativeCoefficient = 0;//P
	float mOutput;
	float mError;
	float dT;
public:
	//public constructor
	PICController(float PropotionalCoeffient,float IntegralCoefficient,float DerivativeCoefficient)
	{
		mPropotionalCoeffient = PropotionalCoeffient;
		mIntegralCoefficient = IntegralCoefficient;
		mDerivativeCoefficient = DerivativeCoefficient;
	}
	//Propotional setter
	void setProportinal(float coefficient)
	{
		mPropotionalCoeffient = coefficient;
	}
	//Integral setter
	void setIntegral(float coefficient)
	{
		mIntegralCoefficient = coefficient;
	}
	//Derivative setter
	void setDerivative(float coefficient)
	{
		mDerivativeCoefficient = coefficient;
	}
	//Error setter
	void setError(float error)
	{
		mError = error;
	}
	//Get output
	float getOutput()
	{
		return (mPropotionalCoeffient * mError) + (mIntegralCoefficient * mError) + ()
	}
};