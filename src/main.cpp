#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLog.h>

#include "FEHOTOS.h"
#include "PIDF.h"

#define M_PI 3.14159265358979323846

FEHMotor FL(FEHMotor::Motor1,6.0);
FEHMotor FR(FEHMotor::Motor2,6.0);
FEHMotor BM(FEHMotor::Motor0,6.0);

// Weight Biases &  motor direction normalization
double FLsign = -1;
double FRsign = -0.95;
double BMsign = -0.90;

PIDF xController(1.0, 0, 0, 0);
PIDF yController(1.0, 0, 0, 0);
PIDF hController(0.085, 0, 0, 0);

float targetX = 10;
float targetY = 10;
float targetH = 0;

void SetMotorPowers(double vX, double vY, double vTheta) {
    // Y+ is forward
    // X+ is right

    double BMpower = -vX + vTheta;
    double FRpower = (1.0/2.0) * vX - (sqrt(3) / 2) * vY + vTheta;
    double FLpower = (1.0/2.0) * vX + (sqrt(3) / 2) * vY + vTheta;

    double maxPower = max(abs(BMpower), max(abs(FRpower), max(abs(FLpower), 1.0)));

    BMpower = (BMpower / maxPower) * 100 * BMsign;
    FRpower = (FRpower / maxPower) * 100 * FRsign;
    FLpower = (FLpower / maxPower) * 100 * FLsign;

    FEHLog::printf("FL:%.1f FR:%.1f BM:%.1f\n", FLpower, FRpower, BMpower);

    BM.SetPercent(BMpower);
    FR.SetPercent(FRpower);
    FL.SetPercent(FLpower);

}

boolean AtPose(OTOSPose pose) {
    bool atHeading = 0.5 >= abs(targetH - pose.h);

    float distX = targetX - pose.x;
    float distY = targetY - pose.y;

    float Dist = sqrt(pow(distX, 2.0) + pow(distY, 2.0));

    bool atLocation = 0.15 >= Dist;

    return atHeading && atLocation;
}

void ERCMain()
{
    FEHLog::enableBLE(152);

    int step = 1;

    OTOS.begin();
    OTOS.resetTracking();
    OTOS.calibrateImu();

    Sleep(10.0);

    OTOSPose pos;

    xController.setTarget(0);
    yController.setTarget(0);
    hController.setTarget(0);

    while (true) {
        OTOS.getPosition(pos);

        switch (step) {
            case 1:
                targetX = 10;
                targetY = 10;
                targetH = 0;

                if (AtPose(pos)) {
                    step = 2;
                }
                break;
            case 2:
            targetX = 20;
            targetY = 5;
            targetH = 90;

            if (AtPose(pos)) {
                step = 3;
            }
            break;
            case 3:
                targetX = 5;
                targetY = 20;
                targetH = -90;

                if (AtPose(pos)) {
                step = 4;
                }
                break;
            case 4:
                targetX = 0;
                targetY = 0;
                targetH = 0;
                break;

        }

        xController.setTarget(targetX);
        yController.setTarget(targetY);
        hController.setTarget(targetH);

        xController.update(pos.x,TimeNow() * 1000.0f);
        yController.update(pos.y,TimeNow() * 1000.0f);
        hController.update(pos.h,TimeNow() * 1000.0f);

        float hRads = (pos.h * M_PI) / 180;

        float X = xController.output * cos(hRads) + yController.output * sin(hRads);
        float Y = -xController.output * sin(hRads) + yController.output * cos(hRads);

        SetMotorPowers(X, Y, -hController.output);

        FEHLog::printf("Xo:%.2f Yo:%.2f Ho:%.2f\n", xController.output * 1.0f, yController.output * 1.0f, hController.output * -1.0f);
        FEHLog::printf("X:%.2f Y:%.2f H:%.2f\n", X, Y, pos.h);
  
        Sleep(10);
    }

}