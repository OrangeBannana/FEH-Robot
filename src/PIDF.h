#pragma once

class PIDF {
    public:
    
    PIDF(float p, float i, float d, float f);
    void setPIDF(float p, float i, float d, float f);
    void setTarget(float target);
    void update(float value, float time); // Time in ms
    
    float targetValue;
    float kP, kI, kD, kF;
    float integralSum;
    float output;

    float previousTime = 0;
    float previousValue = 0;
};