#include "Arduino.h"
#include "MPU9250RPY.h"
#include "Wire.h"
#include "math.h"

MPU9250RPY::MPU9250RPY()
{
	aOffset[0] = 0;
	aOffset[1] = 0;
	aOffset[2] = 0;
	aBias[0] = 1;
	aBias[1] = 1;
	aBias[2] = 1;

	gOffset[0] = 0;
	gOffset[1] = 0;
	gOffset[2] = 0;
	gBias[0] = 1;
	gBias[1] = 1;
	gBias[2] = 1;

	mOffset[0] = 0;
	mOffset[1] = 0;
	mOffset[2] = 0;
	mBias[0] = 1;
	mBias[1] = 1;
	mBias[2] = 1;

	accelRange = ACCEL_RANGE;
	gyroRange = GYRO_RANGE;
	magRange = MAG_RANGE;

	rollComplimentaryFactor = 0.98;
	pitchComplimentaryFactor = 0.98;
	yawComplimentaryFactor = 0.98;

	gyroDivisionFactor = GYRO_500_FACTOR;
	magneticDeclination = 0;
}

void MPU9250RPY::timeCalculator(void) {
	previousTime = presentTime;  //sets the past time equal to present time of the last loop calculation
	presentTime = millis();  //gets the present time
	elapsedTime = (presentTime - previousTime) / 1000;  //gets the elapsed time, converts from milliseconds to seconds
}

void MPU9250RPY::I2Cread(int Address, uint8_t Register, int Nbytes, uint8_t* Data)
{
	Wire.beginTransmission(Address);
	Wire.write(Register);
	Wire.endTransmission(false);
	Wire.requestFrom(Address, Nbytes, true);
	uint8_t index = 0;
	while (Wire.available())
		Data[index++] = Wire.read();
}

void MPU9250RPY::I2CwriteByte(int Address, uint8_t Register, uint8_t Data)
{
	Wire.beginTransmission(Address);
	Wire.write(Register);
	Wire.write(Data);
	Wire.endTransmission();
}

void MPU9250RPY::mpu9250RawData(void)
{
	uint8_t mpuBuffer[14];
	I2Cread(MPU_ADDR, 0x3B, 14, mpuBuffer);
	AcRaw[0] = -(mpuBuffer[0] << 8 | mpuBuffer[1]);
	AcRaw[1] = -(mpuBuffer[2] << 8 | mpuBuffer[3]);
	AcRaw[2] = mpuBuffer[4] << 8 | mpuBuffer[5];
	Tmp = mpuBuffer[6] << 8 | mpuBuffer[7];
	GyRaw[0] = -(mpuBuffer[8] << 8 | mpuBuffer[9]);
	GyRaw[1] = -(mpuBuffer[10] << 8 | mpuBuffer[11]);
	GyRaw[2] = mpuBuffer[12] << 8 | mpuBuffer[13];

	uint8_t ST1;
	I2CwriteByte(MAG_ADDR, 0x0A, 0x01);
	do
	{
		I2Cread(MAG_ADDR, 0x02, 1, &ST1);
	} while (!(ST1 & 0x01));

	uint8_t magBuffer[6];
	I2Cread(MAG_ADDR, 0x03, 7, magBuffer);
	MgRaw[0] = -(magBuffer[3] << 8 | magBuffer[2]);
	MgRaw[1] = -(magBuffer[1] << 8 | magBuffer[0]);
	MgRaw[2] = magBuffer[5] << 8 | magBuffer[4];
}

void MPU9250RPY::getGyroAngle()
{
	static double x, y, z;

	//filter gyro data with a recursive filter
	rawFilter(GYRO_FILTER, x, y, z, GyRaw, gOffset, gBias, gyroRange, gyroDivisionFactor);

	//update roll
	gyroAngleX += (-x * elapsedTime);
	//update pitch, considers yaw changes which might affect the pitch
	gyroAngleY += ((y * cos(accelAngleX * PI / 180)) + (z * sin(accelAngleX * PI / 180))) * elapsedTime;
	//update the yaw
	gyroAngleZ += ((z * cos(accelAngleX * PI / 180)) - (y * sin(accelAngleX * PI / 180))) * elapsedTime;
}

void MPU9250RPY::getAccelAngle()
{	
	static double x, y, z;

	//filter accel data with a recursive filter
	rawFilter(ACCEL_FILTER, x, y, z, AcRaw, aOffset, aBias, accelRange, 0);

	//******************************ROLL***********************************
	//checks if the z acceleration is 0 to prevent division by 0
	if (z == 0)
	{
		if (y < 0)
		{
			accelAngleX = 90;
		}
		else if (y > 0)
		{
			accelAngleX = -90;
		}
		else
		{
			accelAngleX = accelAngleX;  //keeps previous angle if y and z are 0, pitch is either vertical up or down
		}
	}
	//if the z acceleration is not 0 and a reading is possible
	else
	{
		accelAngleX = atan2(-y, z);
	}

	//******************************PITCH**********************************
	//checks if z acceleration is 0 to prevent division by 0
	if ((z == 0) and (y == 0))
	{
		if (x > 0)
		{
			accelAngleY = -180;
		}
		else
		{
			accelAngleY = 180;
		}
	}
	//if the z acceleration is not 0 and a reading is possible
	else
	{
		if (fabs(z) > fabs(y))
		{
			accelAngleY = -atan(x / fabs(z));  //pitch calculation
		}
		else
		{
			accelAngleY = atan(-x / fabs(y));  //reading from x and y accelerations
		}
	}

	//converting radians into degrees
	accelAngleX = accelAngleX * (180.0 / PI);
	accelAngleY = accelAngleY * (180.0 / PI);
}

void MPU9250RPY::getYawAngle()
{	
	static double x, y, z;

	//filter magnetometer data with a recursive filter
	rawFilter(MAG_FILTER, x, y, z, MgRaw, mOffset, mBias, magRange, 0);
	
	magAngleZ = atan2((z * sin(roll * PI / 180) - y * cos(roll * PI / 180)), (x * cos(-pitch * PI / 180) + y * sin(-pitch * PI / 180) * sin(roll * PI / 180) + z * sin(-pitch * PI / 180) * cos(roll * PI / 180))) * (180 / PI);
}

void MPU9250RPY::rawFilter(float filterValue, double& x, double& y, double& z, int raw[3], float offset[3], float bias[3], double range, float divFact)
{
	double lastx = x;
	double lasty = y;
	double lastz = z;

	x = ((raw[0] + offset[0]) * (bias[0]));
	y = ((raw[1] + offset[1]) * (bias[1]));
	z = ((raw[2] + offset[2]) * (bias[2]));

	if (range != 0)
	{
		if (x > range)
			x = range;
		else if (x < -range)
			x = -range;

		if (y > range)
			y = range;
		else if (y < -range)
			y = -range;

		if (z > range)
			z = range;
		else if (z < -range)
			z = -range;
	}

	if (divFact != 0)
	{
		x /= divFact;
		y /= divFact;
		z /= divFact;
	}

	x = (filterValue * x + (1 - filterValue) * lastx);
	y = (filterValue * y + (1 - filterValue) * lasty);
	z = (filterValue * z + (1 - filterValue) * lastz);
}

void MPU9250RPY::setOrientation()
{
	//sets the roll and pitch angles
	roll = ((rollComplimentaryFactor * gyroAngleX) + ((1 - rollComplimentaryFactor) * accelAngleX));
	pitch = ((pitchComplimentaryFactor * gyroAngleY) + ((1 - pitchComplimentaryFactor) * accelAngleY));

	getYawAngle();
	quadrantCheck(magAngleZ, gyroAngleZ, lastQuadYaw, currentQuadYaw);

	yaw = ((yawComplimentaryFactor * gyroAngleZ) + ((1 - yawComplimentaryFactor) * magAngleZ));

	//prevents the roll from exceeding the limits [-180,180] degrees
	if ((roll > 180) or (roll < -180))
		roll = accelAngleX;
	if ((yaw > 180) or (yaw < -180))
		yaw = magAngleZ;

	//sets the gyro angles to equal the new filtered values
	gyroAngleY = pitch;
	gyroAngleX = roll;
	gyroAngleZ = yaw;
}

void MPU9250RPY::quadrantCheck(float refAngle, float& angle, int& lastQuad, int& currentQuad)
{
	if ((refAngle >= 0) and (refAngle <= 90))  //checks if in quad 1
	{
		currentQuad = 1;
	}

	else if ((refAngle >= 90) and (refAngle <= 180))  //checks if in quad 4
	{
		currentQuad = 4;
	}

	else if ((refAngle >= -180) and (refAngle <= -90))  //checks if in quad 3
	{
		currentQuad = 3;
	}

	if ((refAngle >= -90) and (refAngle <= 0))  //checks if in quad 2
	{
		currentQuad = 2;
	}
	


	if ((lastQuad == 3) and (currentQuad == 4))
	{
		angle += 360;
	}
	else if ((lastQuad == 4) and (currentQuad == 3))
	{
		angle -= 360;
	}
}

void MPU9250RPY::masterAngleUpdate()
{
	//Read the raw data
	mpu9250RawData();

	//Acceleration Angle Update
	getAccelAngle();

	//Set the last quadrant before update
	lastQuadRoll = currentQuadRoll;
	lastQuadYaw = currentQuadYaw;

	//Time
	timeCalculator();

	//Gyro Angle Update
	getGyroAngle();

	//Get quadrant for roll of plane
	quadrantCheck(accelAngleX, gyroAngleX, lastQuadRoll, currentQuadRoll);

	//Pitch and Roll Update
	setOrientation();
}

void MPU9250RPY::initialize(void)
{
	Wire.begin();
	//configure gyro
	I2CwriteByte(MPU_ADDR, 0x1B, GYRO_500);
	//confugure accel
	I2CwriteByte(MAG_ADDR, 0x1C, ACCEL_2);
	//configure mag
	I2CwriteByte(MPU_ADDR, 0x37, 0x02);
	I2CwriteByte(MAG_ADDR, 0x0A, 0x01);
}

void MPU9250RPY::filterValue(float pFact, float rFact, float yFact)
{
	pitchComplimentaryFactor = pFact;
	rollComplimentaryFactor = rFact;
	yawComplimentaryFactor = yFact;
}

void MPU9250RPY::setGyroError(float x, float y, float z)
{
	gOffset[0] = x; gOffset[1] = y;	gOffset[2] = z;
}

void MPU9250RPY::setMagError(float xoff, float yoff, float zoff, float xbias, float ybias, float zbias)
{
	mOffset[0] = xoff; mOffset[1] = yoff; mOffset[2] = zoff;
	mBias[0] = xbias; mBias[1] = ybias; mBias[2] = zbias;
}

void MPU9250RPY::setMagneticDeclination(double value)
{
	magneticDeclination = value;
}

void MPU9250RPY::updateOrientation(void)
{
	masterAngleUpdate();
}

float MPU9250RPY::returnRoll(void)
{
	return roll;
}

float MPU9250RPY::returnPitch(void)
{
	return pitch;
}

float MPU9250RPY::returnYaw(void)
{
	return (yaw + 180 + magneticDeclination);
}

bool MPU9250RPY::found(void)
{
	bool dev1Avail;
	bool dev2Avail;

	//check if mpu is found
	Wire.beginTransmission(MPU_ADDR);
	if (Wire.endTransmission() == 0)
		dev1Avail = true;
	else
		dev1Avail = false;

	//check if mag is found
	Wire.beginTransmission(MAG_ADDR);
	if (Wire.endTransmission() == 0)
		dev2Avail = true;
	else
		dev2Avail = false;

	if ((dev1Avail == true) and (dev2Avail == true))
		return true;
	else
		return false;
}