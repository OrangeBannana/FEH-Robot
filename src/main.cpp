// FEH Library Imports
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLog.h>
#include <FEHRCS.h>
#include <FEHServo.h>

// Local Class Imports
#include "FEHOTOS.h"
#include "PDFL.h"

// Create Motor Objects
FEHMotor FL(FEHMotor::Motor2,7.0);
FEHMotor FR(FEHMotor::Motor3,7.0);
FEHMotor BM(FEHMotor::Motor1,7.0);

// Create servo object
FEHServo armServo(FEHServo::Servo0); 

int armUpPos = 180, armDownPos = 110, armDropPos = 155;

// Create CDS Cell Object
AnalogInputPin CDS(FEHIO::Pin0);

// Signs to adjust motor directions
// Should be configured such that X+ is right and Y+ is forward
// when passing drive vectors to robot
double FLsign = -1;
double FRsign = -1;
double BMsign = -1;

// Scale all power values by this amount
double powerScale = 1;

// Boolean to enable or disable using biases when driving NOT IMPLEMENTED
bool useBiases = true;

// Weights to account for weight distribution and geometric imperfections
double FLweight = 1;
double FRweight = 0.95;
double BMweight = 0.90;

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
         finishPos = {43.68, -2.0, 90};

// Robot positions for door milestone
OTOSPose preOpenPose = {12, -16.34, -90},
         openPose = {12, -21.9, -90},
         transitionPose = {9.06, -21.85, -90},
         preClosePose = {12, -22.85, -90},
         closedPose = {12, -17.6, -90},
         leavePose = {9.06, -18, -90};

// Robot positions for apple bucket milestone
OTOSPose prePickupPos = {6.72, -14.06, -107},
         pickupPos = {9.95, -15.1, -107},
         preRampPos = {-4.81, -16.50, -170.0},
         dropPose = {1.77, -1.60, 90};



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

timer pickupTimer;

// Drive to a global coordinate using PID
void DriveTo(double X, double Y, double Theta) {

    targetX = X;
    targetY = Y;
    targetH = Theta;
    // For both robot and world coordinates
    // Y+ is forward
    // X+ is right

    float tError = sqrt(pow(targetX - pos.x, 2) + pow(targetY - pos.y, 2));
    float hError = targetH - pos.h;
    hError = fmod(hError + 180, 360) - 180;

    tController.setTarget(0);
    hController.setTarget(0);

    tController.update(tError);
    hController.update(-hError);

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

    BM.SetPercent(BMpower * powerScale);
    FR.SetPercent(FRpower * powerScale);
    FL.SetPercent(FLpower * powerScale);

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
    // 36 EW
    // int oX = 0, oY = 0, codeBorderWidth = 14, codeElementWidth = 36;
    // int codeSize = codeBorderWidth * 2 + codeElementWidth * 6;

    // // Border Draw
    // LCD.Clear();
    // LCD.SetFontColor(WHITE);
    // LCD.FillRectangle(oX, oY, codeSize, codeSize);

    // int code[6][6] = {
    //     {0, 0, 0, 0, 0, 0},
    //     {0, 0, 0, 1, 1, 0},
    //     {0, 0, 0, 1, 1, 0},
    //     {0, 0, 0, 1, 0, 0},
    //     {0, 1, 1, 0, 1, 0},
    //     {0, 0, 0, 0, 0, 0},
    // };
    
    // // Code Draw
    // int drawX = oX + codeBorderWidth, drawY = oY + codeBorderWidth;

    // for (int i = 0; i < 6; i++) {
    //     drawX = oX + codeBorderWidth;
    //     for (int k = 0; k < 6; k++) {
    //         if (code[i][k] == 0) {
    //             LCD.SetFontColor(BLACK);
    //         } else {
    //             LCD.SetFontColor(WHITE);
    //         }
    //         LCD.FillRectangle(drawX, drawY, codeElementWidth, codeElementWidth);
    //         drawX += codeElementWidth;
    //     }
    //     drawY += codeElementWidth;
    // }

    armServo.SetMin(750);
    armServo.SetMax(2000);
    armServo.SetDegree(armUpPos);

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
                    step = 18;
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
                if ((abs(vel.x) < 0.02 &&  abs(vel.y) < 0.02 && abs(vel.h) < 0.02 && TimeNow() - relocTimer.startTime >= 6.0) || TimeNow() - relocTimer.startTime >= 20.0) {
                    OTOS.setPosition(zeroPos);
                    zeroMotors();
                    step = 22;
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

            case 18:
                armServo.SetDegree(armDownPos);
                DriveTo(prePickupPos.x, prePickupPos.y, prePickupPos.h);
                if (AtPose()) {
                    relocTimer.startTime = TimeNow();
                    powerScale = 0.4;
                    step = 19;
                }
            break;

            case 19:
                DriveAt(0,.25,0);
                if ((abs(vel.x) < 0.02 &&  abs(vel.y) < 0.02 && abs(vel.h) < 0.02 && TimeNow() - relocTimer.startTime >= 2.0) || abs(pos.x) >= abs(pickupPos.x)) {
                    step = 20;
                    pickupTimer.startTime = TimeNow();
                    zeroMotors();
                }
            break;

            case 20:
                armServo.SetDegree(armUpPos);
                if (TimeNow() - pickupTimer.startTime >= 1.0) {
                    powerScale = 1;
                    step = 21;
                }
            break;

            case 21:
                DriveTo(preRampPos.x, preRampPos.y, preRampPos.h);
                if (AtPose()) {
                    step = 4;
                }
            break;

            case 22:
                DriveTo(dropPose.x, dropPose.y, dropPose.h);
                if (AtPose()) {
                    pickupTimer.startTime = TimeNow();
                    step = 23;
                }
            break;

            case 23:
                DriveTo(dropPose.x, dropPose.y, dropPose.h);
                armServo.SetDegree(armDropPos);
                if (TimeNow() - pickupTimer.startTime >= 1) {
                    step = 24;
                }
            break;
            
            case 24:
                DriveTo(dropPose.x, dropPose.y, dropPose.h);
            break;

        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}// FEH Library Imports