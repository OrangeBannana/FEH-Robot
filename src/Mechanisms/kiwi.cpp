#include "kiwi.h"

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

    float tOutput =  tController.output;
    float hOutput = hController.output;

    if (atPose()){
        tOutput = tController.PDFoutput;
        hOutput = hController.PDFoutput;
    }

    float vX1 = ((targetX - pose.x) / tError) * -tOutput;
    float vY1 = ((targetY - pose.y) / tError) * -tOutput;

    // Drive commands based on PDFL
    float vX2 = vX1 * cos(hRads)  + vY1 * sin(hRads);
    float vY2 = -vX1 * sin(hRads) + vY1 * cos(hRads);
    float vTheta = -hOutput;

    // Clamp vTheta to avoid current limit
    vTheta = (abs(vTheta) > (7.0f/12.0f)) ? (7.0f/12.0f) * PDFL::signum(vTheta) :  vTheta;

    // Asymmetric Translational Slew Rate Limiting
    float v = sqrt(pow(vX, 2) + pow(vY, 2));
    float v2 = sqrt(pow(vX2, 2) + pow(vY2, 2));

    // Anti Windup
    if (v2 > 1.0) {
        vX2 = (vX2 / v2);
        vY2 = (vY2 / v2);
    }

    float dVX = vX2 - vX;
    float dVY = vY2 - vY;
    float dV = sqrt(pow(dVX, 2) + pow(dVY, 2));

    if (v2 > v) {
        if (dV > upSlewLimit) {
            vX = vX + ((dVX / dV) * upSlewLimit);
            vY = vY + ((dVY / dV) * upSlewLimit);
        } else {
            vX = vX2;
            vY = vY2;
        }
    } else {
        if (dV > downSlewLimit) {
            vX = vX + ((dVX / dV) * downSlewLimit);
            vY = vY + ((dVY / dV) * downSlewLimit);
        } else {
            vX = vX2;
            vY = vY2;
        }
    }

    // Do not apply PDFL control if disabled
    if (!doXPDFL) {vX = driveVector.x;};
    if (!doYPDFL) {vY = driveVector.y;};
    if (!doHPDFL) {vTheta = driveVector.h;};

    float BMpower = -vX + vTheta;
    float FRpower = (1.0/2.0) * vX - (sqrt(3) / 2) * vY + vTheta;
    float FLpower = (1.0/2.0) * vX + (sqrt(3) / 2) * vY + vTheta;

    //FEHLog::printf("FL:%.1f FR:%.1f BM:%.1f\n", FLpower, FRpower, BMpower);

    double maxPower = (max(abs(BMpower), max(abs(FRpower), max(abs(FLpower), (maxVoltage / voltageLimit)))));

    BMpower = (BMpower / maxPower) * 100 * BMsign;
    FRpower = (FRpower / maxPower) * 100 * FRsign;
    FLpower = (FLpower / maxPower) * 100 * FLsign;

    //FEHLog::printf("FL:%.1f FR:%.1f BM:%.1f\n", FLpower, FRpower, BMpower);
    //FEHLog::printf("tOut: %0.1f hOut: %0.1f\n", tController.output, hController.output);
    //FEHLog::printf("PDFLX: %d PDFLY: %d PDFLH: %d\n", doXPDFL, doYPDFL, doHPDFL);

    BM.SetPercent(BMpower);
    FR.SetPercent(FRpower);
    FL.SetPercent(FLpower);
    FEHLog::printf("tError: %0.1f hError: %0.1f\n", tError, hError);
}

bool kiwi::atPoseXY() {
    float distX = targetPose.x - pose.x;
    float distY = targetPose.y - pose.y;

    float Dist = sqrt(pow(distX, 2.0) + pow(distY, 2.0));

    return  0.1 >= Dist;
}

bool kiwi::atPoseH() {
    return 0.5 >= abs(targetPose.h - pose.h);
}

bool kiwi::atPose() {
    return atPoseH() && atPoseXY();
}

bool kiwi::nearPose() {
    float distX = targetPose.x - pose.x;
    float distY = targetPose.y - pose.y;

    float Dist = sqrt(pow(distX, 2.0) + pow(distY, 2.0));

    bool nearT = 0.25 >= Dist;

    bool nearH = 1.5 >= abs(targetPose.h - pose.h);

    return nearT && nearH;
}

bool kiwi::nearPoseR(float radius) {
    float distX = targetPose.x - pose.x;
    float distY = targetPose.y - pose.y;

    float Dist = sqrt(pow(distX, 2.0) + pow(distY, 2.0));

    bool nearT = radius >= Dist;

    bool nearH = 1.5 >= abs(targetPose.h - pose.h);

    return nearT && nearH;
}