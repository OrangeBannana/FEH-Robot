#include "PIDF.h"

PIDF::PIDF(float p, float i, float d, float f) {
    kP = p;
    kI = i;
    kD = d;
    kF = f;
}

void PIDF::setPIDF(float p, float i, float d, float f) {
    kP = p;
    kI = i;
    kD = d;
    kF = f;
}

void PIDF::setTarget(float target) {
    targetValue = target;
}

void PIDF::update(float value, float time) {

    float targetValueDistance = targetValue - value;
    float deltaValue = value - previousValue;
    float deltaTime = time - previousTime;

    float P = kP * targetValueDistance;
    float D = 0;

    if (deltaTime != 0) {
        D = kD * (deltaValue / deltaTime);
    }

    output = P + D;

}