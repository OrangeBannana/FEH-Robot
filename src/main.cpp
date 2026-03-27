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
FEHMotor FL(FEHMotor::Motor1,7.0);
FEHMotor FR(FEHMotor::Motor2,7.0);
FEHMotor BM(FEHMotor::Motor0,7.0);

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
PDFL tController(1.2, 0, 0, 0);
PDFL hController(0.085, 0, 0, 0);

// Initial Target Positions
float targetX = 0;
float targetY = 0;
float targetH = 0;

// Robot position tracking
OTOSPose pos;
OTOSPose vel;
OTOSPose preRampPose;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

// Robot positions for getting up ramp
OTOSPose zeroPos = {0.0, 0.0, 0.0},
         startPos = {0.3-.25, -5.18+0.5, -129.0},
         buttonPos = {-3.2, -4.5, -155.0},
         upRampPos = {-3.5, -40.4, 90},
         calibrationPos = {-2.75, -40.4, 90};

// Robot positions for button milestone
OTOSPose readLightPos = {6.12, -18.54, -177},
         blueButtonPos = {7.79, -23.0, -177},
         redButtonPos = {-0.3, -25.0, -177},
         downRampPos = {9.70, -3.23, 90},
         finishPos = {40.68, -2.51, 90};

// Robot positions for door milestone
OTOSPose preOpenPose = {11, -16.34, -90},
         openPose = {11, -21.55, -90},
         transitionPose = {8.06, -21.85, -90},
         preClosePose = {11, -22.38, -90},
         closedPose = {11, -17.6, -90},
         leavePose = {8.06, -18, -90};

AnalogInputPin CDS(FEHIO::Pin0);
float redVal = 0.415;
float offVal = 3.162;
float blueVal = 0.0;

struct timer {
    public:
        float startTime;

    private:
        float finalTime;
        bool finished = false;

    float getTime() {
        if (finished) { return finalTime; }
        else { return TimeNow() - startTime; }
    }

    void end() {
        finished = true;
        finalTime = TimeNow() - startTime;
    }

};

timer startBTNTimer;
timer relocTimer;
timer BTN2Timer;

timer openDoorTimer;
timer closeDoorTimer;

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

// Drive with given constant vectors relative to robot position
void DriveAt(double vX, double vY, double vTheta) {
    // For both robot and world coordinates
    // Y+ is forward
    // X+ is right
    // H+ is CW

    double BMpower = -vX + vTheta;
    double FRpower = (1.0/2.0) * vX - (sqrt(3) / 2) * vY + vTheta;
    double FLpower = (1.0/2.0) * vX + (sqrt(3) / 2) * vY + vTheta;

    BMpower = BMpower * 100 * BMsign;
    FRpower = FRpower * 100 * FRsign;
    FLpower = FLpower * 100 * FLsign;

    BM.SetPercent(BMpower);
    FR.SetPercent(FRpower);
    FL.SetPercent(FLpower);

}

// Returns true if the robot is at its position within a tolerance
boolean AtPose() {
    bool atHeading = 0.5 >= abs(targetH - pos.h);

    float distX = targetX - pos.x;
    float distY = targetY - pos.y;

    float Dist = sqrt(pow(distX, 2.0) + pow(distY, 2.0));

    bool atLocation = 0.15 >= Dist;

    return atHeading && atLocation;
}

void zeroMotors() {
    BM.SetPercent(0);
    FR.SetPercent(0);
    FL.SetPercent(0);
}

void ERCMain()
{   

    LCD.WriteLine("Initializing BLE Logging...");
    FEHLog::enableBLE(152);
    LCD.WriteLine("BLE Logging Initialized");
    
    LCD.WriteLine("Connecting to RCS");
    RCS.InitializeTouchMenu("0800A2XNH");
    LCD.WriteLine("RCS Connected");
    LCD.WriteLine("Goofy Goober fr");

    LCD.WriteLine("Waiting for press...");
    LCD.WaitForTouchToStart();
    LCD.WaitForTouchToEnd();

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
        OTOS.getVelocity(vel);

        switch (step) {

            case 1:
                DriveTo(startPos.x, startPos.y, startPos.h);
                if (AtPose()) step = 2;
                break;

            case 2:
                DriveTo(startPos.x, startPos.y, startPos.h);
                if (abs(redVal - CDS.Value()) < 0.35) {
                    startBTNTimer.startTime = TimeNow();
                    step = 3;
                }
                break;

            case 3:
                DriveTo(buttonPos.x, buttonPos.y, buttonPos.h);
                if (TimeNow() - startBTNTimer.startTime >= 1.0) {
                    step = 4;
                }
                break;

            case 4:
                DriveTo(upRampPos.x, upRampPos.y, upRampPos.h);
                if (AtPose()) {
                    relocTimer.startTime = TimeNow();
                    zeroMotors();
                    Sleep(1.0);
                    step = 5;
                }
            break;
            
            case 5:
                DriveAt(-0.1, 0.2, -0.00);
                if (TimeNow() - relocTimer.startTime >= 4.0) step = 6;
            break;

            case 6:
                DriveAt(-0.2, 0.25, 0.0);
                if (abs(vel.x) < 0.01 &&  abs(vel.y) < 0.01 && abs(vel.h) < 0.01 && TimeNow() - relocTimer.startTime >= 2.0) {
                    OTOS.setPosition(zeroPos);
                    zeroMotors();
                    step = 12;
                }
            break;

            case 7:
                DriveTo(readLightPos.x, readLightPos.y, readLightPos.h);
                if (AtPose()) {
                    zeroMotors();
                    Sleep(500);
                    BTN2Timer.startTime = TimeNow();

                    if (abs(redVal - CDS.Value()) < 0.35) {
                        step = 8;
                    } else {
                        step = 9;
                    }

                }
            break;
            // Red BTN
            case 8:
                DriveTo(redButtonPos.x, redButtonPos.y, redButtonPos.h);
                if (TimeNow() - BTN2Timer.startTime >= 4) {
                    LCD.WriteLine("RED");
                    step = 10;
                }
            break;
            // Blue BTN
            case 9:
                DriveTo(blueButtonPos.x, blueButtonPos.y, blueButtonPos.h);
                if (TimeNow() - BTN2Timer.startTime >= 4) {
                    LCD.WriteLine("BLUE");
                    step = 10;
                }
            break;
            // Start of window task cases
            case 12:
                DriveTo(preOpenPose.x, preOpenPose.y, preOpenPose.h);
                if (AtPose()) {
                    openDoorTimer.startTime = TimeNow();
                    step = 13;
                }
            break;

            case 13:
                DriveTo(openPose.x, openPose.y, openPose.h);
                if (AtPose() || TimeNow() - openDoorTimer.startTime >= 2) step = 14;
            break;

            case 14:
                DriveTo(transitionPose.x, transitionPose.y, transitionPose.h);
                if (AtPose()) step = 15;
            break;

            case 15:
                DriveTo(preClosePose.x, preClosePose.y, preClosePose.h);
                if (AtPose()) {
                    closeDoorTimer.startTime = TimeNow();
                    step = 16;
                }
            break;

            case 16:
                DriveTo(closedPose.x, closedPose.y, closedPose.h);
                if (AtPose() || TimeNow() - closeDoorTimer.startTime >= 2) step = 17;
            break;

            case 17:
                DriveTo(leavePose.x, leavePose.y, leavePose.h);
                if (AtPose()) step = 10;
            break;

            // Start of down ramp cases
            case 10:
                DriveTo(downRampPos.x, downRampPos.y, downRampPos.h);
                if (AtPose()) step = 11; 
            break;

            case 11:
                DriveTo(finishPos.x, finishPos.y, finishPos.h);
            break;
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}// FEH Library Imports
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
FEHMotor FL(FEHMotor::Motor1,7.0);
FEHMotor FR(FEHMotor::Motor2,7.0);
FEHMotor BM(FEHMotor::Motor0,7.0);

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
PDFL tController(1.2, 0, 0, 0);
PDFL hController(0.085, 0, 0, 0);

// Initial Target Positions
float targetX = 0;
float targetY = 0;
float targetH = 0;

// Robot position tracking
OTOSPose pos;
OTOSPose vel;
OTOSPose preRampPose;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

// Robot positions for getting up ramp
OTOSPose zeroPos = {0.0, 0.0, 0.0},
         startPos = {0.3-.25, -5.18+0.5, -129.0},
         buttonPos = {-3.2, -4.5, -155.0},
         upRampPos = {-3.5, -40.4, 90},
         calibrationPos = {-2.75, -40.4, 90};

// Robot positions for button milestone
OTOSPose readLightPos = {6.12, -18.54, -177},
         blueButtonPos = {7.79, -23.0, -177},
         redButtonPos = {-0.3, -25.0, -177},
         downRampPos = {9.70, -3.23, 90},
         finishPos = {40.68, -2.51, 90};

// Robot positions for door milestone
OTOSPose preOpenPose = {11, -16.34, -90},
         openPose = {11, -21.55, -90},
         transitionPose = {8.06, -21.85, -90},
         preClosePose = {11, -22.38, -90},
         closedPose = {11, -17.6, -90},
         leavePose = {8.06, -18, -90};

AnalogInputPin CDS(FEHIO::Pin0);
float redVal = 0.415;
float offVal = 3.162;
float blueVal = 0.0;

struct timer {
    public:
        float startTime;

    private:
        float finalTime;
        bool finished = false;

    float getTime() {
        if (finished) { return finalTime; }
        else { return TimeNow() - startTime; }
    }

    void end() {
        finished = true;
        finalTime = TimeNow() - startTime;
    }

};

timer startBTNTimer;
timer relocTimer;
timer BTN2Timer;

timer openDoorTimer;
timer closeDoorTimer;

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

// Drive with given constant vectors relative to robot position
void DriveAt(double vX, double vY, double vTheta) {
    // For both robot and world coordinates
    // Y+ is forward
    // X+ is right
    // H+ is CW

    double BMpower = -vX + vTheta;
    double FRpower = (1.0/2.0) * vX - (sqrt(3) / 2) * vY + vTheta;
    double FLpower = (1.0/2.0) * vX + (sqrt(3) / 2) * vY + vTheta;

    BMpower = BMpower * 100 * BMsign;
    FRpower = FRpower * 100 * FRsign;
    FLpower = FLpower * 100 * FLsign;

    BM.SetPercent(BMpower);
    FR.SetPercent(FRpower);
    FL.SetPercent(FLpower);

}

// Returns true if the robot is at its position within a tolerance
boolean AtPose() {
    bool atHeading = 0.5 >= abs(targetH - pos.h);

    float distX = targetX - pos.x;
    float distY = targetY - pos.y;

    float Dist = sqrt(pow(distX, 2.0) + pow(distY, 2.0));

    bool atLocation = 0.15 >= Dist;

    return atHeading && atLocation;
}

void zeroMotors() {
    BM.SetPercent(0);
    FR.SetPercent(0);
    FL.SetPercent(0);
}

void ERCMain()
{   

    LCD.WriteLine("Initializing BLE Logging...");
    FEHLog::enableBLE(152);
    LCD.WriteLine("BLE Logging Initialized");
    
    LCD.WriteLine("Connecting to RCS");
    RCS.InitializeTouchMenu("0800A2XNH");
    LCD.WriteLine("RCS Connected");

    LCD.WriteLine("Waiting for press...");
    LCD.WaitForTouchToStart();
    LCD.WaitForTouchToEnd();

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
        OTOS.getVelocity(vel);

        switch (step) {

            case 1:
                DriveTo(startPos.x, startPos.y, startPos.h);
                if (AtPose()) step = 2;
                break;

            case 2:
                DriveTo(startPos.x, startPos.y, startPos.h);
                if (abs(redVal - CDS.Value()) < 0.35) {
                    startBTNTimer.startTime = TimeNow();
                    step = 3;
                }
                break;

            case 3:
                DriveTo(buttonPos.x, buttonPos.y, buttonPos.h);
                if (TimeNow() - startBTNTimer.startTime >= 1.0) {
                    step = 4;
                }
                break;

            case 4:
                DriveTo(upRampPos.x, upRampPos.y, upRampPos.h);
                if (AtPose()) {
                    relocTimer.startTime = TimeNow();
                    zeroMotors();
                    Sleep(1.0);
                    step = 5;
                }
            break;
            
            case 5:
                DriveAt(-0.1, 0.2, -0.00);
                if (TimeNow() - relocTimer.startTime >= 4.0) step = 6;
            break;

            case 6:
                DriveAt(-0.2, 0.25, 0.0);
                if (abs(vel.x) < 0.01 &&  abs(vel.y) < 0.01 && abs(vel.h) < 0.01 && TimeNow() - relocTimer.startTime >= 2.0) {
                    OTOS.setPosition(zeroPos);
                    zeroMotors();
                    step = 12;
                }
            break;

            case 7:
                DriveTo(readLightPos.x, readLightPos.y, readLightPos.h);
                if (AtPose()) {
                    zeroMotors();
                    Sleep(500);
                    BTN2Timer.startTime = TimeNow();

                    if (abs(redVal - CDS.Value()) < 0.35) {
                        step = 8;
                    } else {
                        step = 9;
                    }

                }
            break;
            // Red BTN
            case 8:
                DriveTo(redButtonPos.x, redButtonPos.y, redButtonPos.h);
                if (TimeNow() - BTN2Timer.startTime >= 4) {
                    LCD.WriteLine("RED");
                    step = 10;
                }
            break;
            // Blue BTN
            case 9:
                DriveTo(blueButtonPos.x, blueButtonPos.y, blueButtonPos.h);
                if (TimeNow() - BTN2Timer.startTime >= 4) {
                    LCD.WriteLine("BLUE");
                    step = 10;
                }
            break;
            // Start of window task cases
            case 12:
                DriveTo(preOpenPose.x, preOpenPose.y, preOpenPose.h);
                if (AtPose()) {
                    openDoorTimer.startTime = TimeNow();
                    step = 13;
                }
            break;

            case 13:
                DriveTo(openPose.x, openPose.y, openPose.h);
                if (AtPose() || TimeNow() - openDoorTimer.startTime >= 2) step = 14;
            break;

            case 14:
                DriveTo(transitionPose.x, transitionPose.y, transitionPose.h);
                if (AtPose()) step = 15;
            break;

            case 15:
                DriveTo(preClosePose.x, preClosePose.y, preClosePose.h);
                if (AtPose()) {
                    closeDoorTimer.startTime = TimeNow();
                    step = 16;
                }
            break;

            case 16:
                DriveTo(closedPose.x, closedPose.y, closedPose.h);
                if (AtPose() || TimeNow() - closeDoorTimer.startTime >= 2) step = 17;
            break;

            case 17:
                DriveTo(leavePose.x, leavePose.y, leavePose.h);
                if (AtPose()) step = 10;
            break;

            // Start of down ramp cases
            case 10:
                DriveTo(downRampPos.x, downRampPos.y, downRampPos.h);
                if (AtPose()) step = 11; 
            break;

            case 11:
                DriveTo(finishPos.x, finishPos.y, finishPos.h);
            break;
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}
