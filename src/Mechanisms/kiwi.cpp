#include "kiwi.h"

kiwi::kiwi(FEHMotor::FEHMotorPort FLport, FEHMotor::FEHMotorPort FRport, FEHMotor::FEHMotorPort BMport, double maxVoltage) {
    this->maxVoltage = maxVoltage;

    FEHMotor FL(FLport, maxVoltage);
    FEHMotor FR(FRport, maxVoltage);
    FEHMotor BM(BMport, maxVoltage);
}

void kiwi::drive() {

    // Calculate PDFL output
    float targetX = targetPose.x;
    float targetY = targetPose.y;
    float targetH = targetPose.h;

    // For both robot and world coordinates
    // Y+ is forward
    // X+ is right
    // Theta+ is CCW

    float tError = sqrt(pow(targetX - pose.x, 2) + pow(targetY - pose.y, 2));
    float hError = targetH - pose.h;
    hError = fmod(fmod(hError + 180, 360) + 360, 360) - 180;

    tController.setTarget(0);
    hController.setTarget(0);

    tController.update(tError);
    hController.update(-hError);

    float hRads = (pose.h * PI) / 180;

    float vX1 = ((targetX - pose.x) / tError) * -tController.output;
    float vY1 = ((targetY - pose.y) / tError) * -tController.output;

    // Drive commands based on PDFL
    float vX = vX1 * cos(hRads)  + vY1 * sin(hRads);
    float vY = -vX1 * sin(hRads) + vY1 * cos(hRads);
    float vTheta = -hController.output;

    // Do not apply PDFL control if disabled
    if (!doXPDFL) {vX = driveVector.x;};
    if (!doYPDFL) {vY = driveVector.y;};
    if (!doHPDFL) {vTheta = driveVector.h;};

    double BMpower = -vX + vTheta;
    double FRpower = (1.0/2.0) * vX - (sqrt(3) / 2) * vY + vTheta;
    double FLpower = (1.0/2.0) * vX + (sqrt(3) / 2) * vY + vTheta;

    double maxPower = max(abs(BMpower), max(abs(FRpower), max(abs(FLpower), 1.0)));

    BMpower = (BMpower / maxPower) * 100 * BMsign;
    FRpower = (FRpower / maxPower) * 100 * FRsign;
    FLpower = (FLpower / maxPower) * 100 * FLsign;

    FEHLog::printf("FL:%.1f FR:%.1f BM:%.1f\n", FLpower, FRpower, BMpower);

    BM.SetPercent(BMpower * powerScalar);
    FR.SetPercent(FRpower * powerScalar);
    FL.SetPercent(FLpower * powerScalar);
}