#include "PDFL.h"

PDFL::PDFL(float kP, float kD, float kF, float kL) {
    this->kP = kP;
    this->kD = kD;
    this->kF = kF;
    this->kL = kL;
}

void PDFL::setPIDF(float kP, float kD, float kF, float kL) {
    this->kP = kP;
    this->kD = kD;
    this->kF = kF;
    this->kL = kL;
}

void PDFL::setTarget(float target) {
    this->target = target;
}

void PDFL::update(float value) {

    error = target - value;
    time = TimeNow() * S_TO_MS;

    deltaError = error - errorPrevious;
    deltaTime = time - timePrevious;

    deltaErrorDeltaTime = deltaError / deltaTime;
    
    P = kP * error;
    D = (deltaTime != 0) ? kD * deltaErrorDeltaTime : 0;

    output = P + D + kF;

    output = (abs(output) >= kL) ? output : ((output * kL) / abs(output));
}