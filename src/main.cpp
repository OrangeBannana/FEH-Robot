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

// Boolean to enable or disable using biases when driving NOT IMPLEMENTED
bool useBiases = true;

// PID controllers
PDFL tController(1.0, 0, 0, 0);
PDFL hController(0.085, 0, 0, 0);

// Initial Target Positions
float targetX = 0;
float targetY = 0;
float targetH = 0;

// Robot position tracking
OTOSPose pos;
OTOSPose preRampPose;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

// Robot positions for tasks
OTOSPose clearRampPos = {0, -12, 0};
OTOSPose startPos = {0.3-.25, -5.18+0.5, -129.0};
OTOSPose buttonPos = {6.72, -20.47, 0};
OTOSPose upRampPos = {6.72, -20.47, 0};
OTOSPose readLightPos = {6.72, -20.47, 0};
OTOSPose blueButtonPos = {6.72, -20.47, 0};
OTOSPose redButtonPos = {6.72, -20.47, 0};
OTOSPose downRampPos = {6.72, -20.47, 0};
OTOSPose finishPos = {6.72, -20.47, 0};


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
    TestGUI();
    LCD.WriteLine("Initializing BLE Logging...");
    FEHLog::enableBLE(152);
    LCD.WriteLine("BLE Logging Initialized");

    LCD.WriteLine("Initializing OTOS, do NOT touch.");
    Sleep(1.0);
    OTOS.begin();
    OTOS.resetTracking();
    OTOS.calibrateImu();
    OTOS.setOffset(posOffset);
    Sleep(1.0);
    LCD.WriteLine("OTOS initialized, press to continue");

    LCD.WaitForTouchToStart();
    LCD.WaitForTouchToEnd();

    // Loop for position output for testing and finding target field positions
    // while (true) {
    //     OTOS.getPosition(pos);
    //     FEHLog::printf("X: %.2fY: %.2fH: %.2f", pos.x, pos.y, pos.h);
    //     Sleep(100);
    // }

    // State counter
    int step = 1;

    while (true) {
        OTOS.getPosition(pos);

        switch (step) {
            // Drive back to avoid ramp
            case 0:
                DriveTo(clearRampPos.x, clearRampPos.y, clearRampPos.h);
                step = (AtPose(pos)) ? 2 : step;
            break;
            // Drive to start light
            case 1:
                DriveTo(startPos.x, startPos.y, startPos.h);
            break;
            // Press Start Button
            case 3:

            break;
            // Go Up Ramp
            case 4:

            break;
            // Go to light
            case 5:

            break;
            // Press Button
            case 6:

            break;
            // Go To Ramp
            case 7:

            break;
            // Go To Final Button
            case 8:

            break;
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}