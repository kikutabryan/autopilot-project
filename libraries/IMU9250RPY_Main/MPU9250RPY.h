#ifndef MPU9250RPY_h
#define MPU9250RPY_h

#include "Arduino.h"

#define GYRO_250 0b00000000
#define GYRO_500 0b00001000
#define GYRO_1000 0b00010000

#define GYRO_250_FACTOR 131.0
#define GYRO_500_FACTOR 65.5
#define GYRO_1000_FACTOR 32.8

#define ACCEL_2 0b00000000
#define ACCEL_4 0b00001000
#define ACCEL_8 0b00010000

#define ACCEL_RANGE 32768
#define GYRO_RANGE 32768
#define MAG_RANGE 0

#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C

#define ACCEL_FILTER 0.70
#define GYRO_FILTER 0.98
#define MAG_FILTER 0.50

class MPU9250RPY
{
  public:
    MPU9250RPY();
    void updateOrientation(void);
    float returnRoll(void);
    float returnPitch(void);
    float returnYaw(void);
    void initialize();
    void filterValue(float pFact, float rFact, float yFact);
    void setGyroError(float x, float y, float z);
    void setMagError(float xoff, float yoff, float zoff, float xbias, float ybias, float zbias);
    void setMagneticDeclination(double value);
    bool found();

   private:
    double presentTime, previousTime, elapsedTime;
    int16_t Tmp;
    int16_t AcRaw[3], GyRaw[3], MgRaw[3];
    float gyroAngleX, gyroAngleY, gyroAngleZ;
    float accelAngleX, accelAngleY;
    float magAngleZ;
    float pitch, roll, yaw;
    float magneticDeclination;
    float rollComplimentaryFactor, pitchComplimentaryFactor, yawComplimentaryFactor;
    int currentQuadRoll, lastQuadRoll;
    int currentQuadYaw, lastQuadYaw;
    float aOffset[3], aBias[3], gOffset[3], gBias[3], mOffset[3], mBias[3];
    float gyroDivisionFactor;
    double accelRange, gyroRange, magRange;

    void masterAngleUpdate();
    void getYawAngle();
    void mpu9250RawData();
    void getAccelAngle();
    void quadrantCheck(float refAngle, float& angle, int& lastQuad, int& currentQuad);
    void timeCalculator();
    void getGyroAngle();
    void setOrientation();
    void I2Cread(int Address, uint8_t Register, int Nbytes, uint8_t* Data);
    void I2CwriteByte(int Address, uint8_t Register, uint8_t Data);
    void rawFilter(float filterValue, double& x, double& y, double& z, int raw[3], float offset[3], float bias[3], double range, float divFact);
};

#endif