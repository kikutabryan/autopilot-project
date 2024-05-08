#ifndef PID_h
#define PID_h

class PIDClass
{
  public:
    PIDClass();
    float PID(float target, float current, float upperLimit, float lowerLimit, float intUpperLimit, float intLowerLimit, bool circle);
    void kConstants(float p, float i, float d);
    void resetPID(void);

  private:
    float Kp, Ki, Kd;
    
    double currentTime;
    float current;
    float integralTerm;

    float proportionalFun(float Kp, float error);
    float integralFun(float Ki, float error, float elapsed);
    float derivativeFun(float Kd, float error, float lastError, float elapsed);
    float elapsedTimeFun();
};

#endif