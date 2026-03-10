// FEH Library Imports
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLog.h>
#include <FEHRCS.h>

// Local Class Imports
#include "FEHOTOS.h"
#include "PDFL.h"

// Create Motor Objects
FEHMotor FL(FEHMotor::Motor1,6.0);
FEHMotor FR(FEHMotor::Motor2,6.0);
FEHMotor BM(FEHMotor::Motor0,6.0);

// Signs to adjust motor directions
// Should be configured such that X+ is right and Y+ is forward
// when passing drive vectors to robot
double FLsign = -1;
double FRsign = -1;
double BMsign = -1;

// Weights to account for weight distribution and geometric imperfections
double FLweight = 1;
double FRweight = 0.95;
double BMweight = 0.90;

// Boolean to enable or disable using biases when driving
bool useBiases = true;

// PID controllers
PDFL tController(1.0, 0, 0, 0);
PDFL hController(0.085, 0, 0, 0);

// Initial Target Positions
float targetX = 0;
float targetY = 0;
float targetH = 0;

// Robot position
OTOSPose pos;
OTOSPose preRampPose;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

RCSPose* rcsPoseLast;
RCSPose* rcsPoseFirst;

// Drive to a global coordinate using PID
void DriveTo(double X, double Y, double Theta) {

    targetX = X;
    targetY = Y;
    targetH = Theta;
    // For both robot and world coordinates
    // Y+ is forward
    // X+ is right

    float tError = sqrt(pow(targetX - pos.x, 2) + pow(targetY - pos.y, 2));

    tController.setTarget(0);
    hController.setTarget(Theta);

    tController.update(tError);
    hController.update(pos.h);

    float hRads = (pos.h * PI) / 180;

    float vX1 = ((targetX - pos.x) / tError) * -tController.output;
    float vY1 = ((targetY - pos.y) / tError) * -tController.output;

    float vX = vX1 * cos(hRads)  + vY1 * sin(hRads);
    float vY = -vX1 * sin(hRads) + vY1 * cos(hRads);
    float vTheta = -hController.output;

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

// Returns true if the robot is at its position within a tolerance
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

    RCS.InitializeTouchMenu("0800A1KDN");
    LCD.WriteLine("RCS Started, press to continue");
    LCD.WaitForTouchToStart();
    LCD.WaitForTouchToEnd();


    FEHLog::enableBLE(152);

    OTOS.begin();
    OTOS.resetTracking();
    OTOS.calibrateImu();
    OTOS.setOffset(posOffset);

    LCD.WriteLine("OTOS initialized, press to continue");
    LCD.WaitForTouchToStart();
    LCD.WaitForTouchToEnd();

    int step = 1;

    float newX, newY, newH;
    OTOSPose postRampPose;

    while (true) {
        OTOS.getPosition(pos);

        switch (step) {
            case 1:
                DriveTo(-1, -12, 0);
                step = (AtPose(pos)) ? 2 : step;
            break;
            case 2:
                DriveTo(10, -12, 0);
                step = (AtPose(pos)) ? 3 : step;
            break;
            case 3:
                FL.SetPercent(0);
                FR.SetPercent(0);
                BM.SetPercent(0);
                Sleep(1.0);
                rcsPoseFirst = RCS.RequestPosition();
                while (rcsPoseFirst->x < 0) {
                    rcsPoseFirst = RCS.RequestPosition();
                }
                Sleep(1.0);
                OTOS.getPosition(preRampPose);
                Sleep(1.0);
                step = 4;
            break;
            case 4:
                DriveTo(10, 10, 0);
                step = (AtPose(pos)) ? 5 : step;
            break;
            case 5:
                FL.SetPercent(0);
                FR.SetPercent(0);
                BM.SetPercent(0);
                Sleep(1.0);
                rcsPoseLast = RCS.RequestPosition();
                while (rcsPoseLast->x < 0) {
                    rcsPoseLast = RCS.RequestPosition();
                }
                Sleep(1.0);
                newX = preRampPose.x + (rcsPoseLast->x - rcsPoseFirst->x);
                newY = preRampPose.y + (rcsPoseLast->y - rcsPoseFirst->y);
                newH = preRampPose.h + (rcsPoseLast->heading - rcsPoseFirst->heading);
                FEHLog::printf("Y: %.2f %.2f %.2f", preRampPose.y, rcsPoseLast->y, rcsPoseFirst->y);
                LCD.WriteLine(preRampPose.y);
                LCD.WriteLine(rcsPoseLast->y);
                LCD.WriteLine(rcsPoseFirst->y);
                FEHLog::printf("X: %.2f Y: %.2f H: %.2f", newX, newY, newH);
                Sleep(10.0);
                newH = preRampPose.h + (rcsPoseLast->heading - rcsPoseFirst->heading);
                postRampPose = {newX, newY, newH};
                OTOS.setPosition(postRampPose);
                Sleep(1.0);
                step = 6;
            break;
            case 6:
                DriveTo(10-15.875000, 10+11.500000 - 8.5, 90);
            break;
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}