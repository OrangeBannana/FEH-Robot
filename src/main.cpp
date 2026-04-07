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

// Enable test mode?
bool testMode = false;

// Create Motor Objects
FEHMotor FL(FEHMotor::Motor2,7.0);
FEHMotor FR(FEHMotor::Motor3,7.0);
FEHMotor BM(FEHMotor::Motor1,7.0);

// Create servo object
FEHServo armServo(FEHServo::Servo0); 

int armUpPos = 180, 
    armDownPos = 117.5,
    armDropPos = 147,
    armDownLeverPos = 90,
    armCompostPos = 116.0;

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
         upRampPos = {-3.5, -40.4, 180},
         calibrationPos = {-2.75, -40.4, 90};

// Robot positions for button milestone OLD - RELATIVE TO OLD RELOCALIZATION
OTOSPose readLightPos = {6.12, -18.54, -177},
         blueButtonPos = {7.79, -23.0, -177},
         redButtonPos = {-0.3, -25.0, -177},
         downRampPos = {9.70, -3.23, 90},
         finishPos = {43.68, -2.0, 90};

// Robot positions for door milestone OLD - RELATIVE TO OLD RELOCALIZATION
OTOSPose preOpenPose = {12, -16.34, -90},
         openPose = {12, -21.9, -90},
         transitionPose = {9.06, -21.85, -90},
         preClosePose = {12, -22.85, -90},
         closedPose = {12, -17.6, -90},
         leavePose = {9.06, -18, -90};

// Robot positions for apple bucket milestone
OTOSPose prePickupPos = {6.72, -14.06, -109},
         pickupPos = {9.95, -15.1, -109},
         preRampPos = {-4.81, -16.50, -170.0},
         dropPose = {-5.32, -49.54, -175},
         postDropPose = {-5.32, -45.54, -175},
         preMidPose = {6.44, -49.57, -131},
         preRightPose = {3.45, -57.66, -132.56},
         preLeftPose = {10.39, -47.63, -130.0};

// Compost Positions
OTOSPose preRotatePos = {16.5, -14.00, 0},
         postRotatePos = {16.5, -9.00, 0};

int leverIndex = 4;

// Relocalization Poses
OTOSPose relocStartPosOTOS;
RCSPose relocStartPosRCS, relocEndPosRCS;



float redVal = 0.415;
float offVal = 3.162;
float blueVal = 0.0;

struct timer {
    public:

        void start(float length) {
            startTime = TimeNow();
            timerLength = length;
        }
    
        bool isOver() {
            return TimeNow() - startTime >= timerLength;
        }
    
        float getTime() {
            return TimeNow() - startTime;
        }

    private:
        float startTime;
        float timerLength;
};

timer freeTimer;
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

void logMode() {
    LCD.Clear();
    zeroMotors();
    OTOS.getPosition(pos);

    LCD.WriteLine("X:");
    LCD.WriteLine(pos.x);

    LCD.WriteLine("Y:");
    LCD.WriteLine(pos.y);

    LCD.WriteLine("H:");
    LCD.WriteLine(pos.h);

    FEHLog::printf("X: %.2fY: %.2fH: %.2f", pos.x, pos.y, pos.h);
    Sleep(300);
}
void ERCMain()
{   

    armServo.SetMin(750);
    armServo.SetMax(2200);
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
    LCD.WriteLine("OTOS initialized");
    LCD.WriteLine("Press to continue");
    LCD.WriteLine("Hold 2 seconds to enter logging mode");

    LCD.WaitForTouchToStart();
    freeTimer.start(2.0);
    LCD.WaitForTouchToEnd();

    // State counter
    int step = 1;

    // Compost Counters
    int forwardSpinCounter = 0,
        backwardsSpinCounter = 0;

    if (true) {
        step = 0;
    }
    
    LCD.Clear();

    double time1;

    // Main objective switch statement
    while (true) {

        // Update position and velocity
        OTOS.getPosition(pos);
        OTOS.getVelocity(vel);

        switch (step) {

            // If log mode is enabled stay in case 0
            case 0:
                logMode();
            break;

            // Go to to start location
            case 1:
                DriveTo(startPos.x, startPos.y, startPos.h);
                if (AtPose()) step = 2;
            break;
            
            // Waiting for light to signal start
            case 2:
                DriveTo(startPos.x, startPos.y, startPos.h);
                if (abs(redVal - CDS.Value()) < 0.35) {
                    startBTNTimer.start(0.5);
                    step = 3;
                }
            break;
            
            // Pressing Start Button
            case 3:
                DriveTo(buttonPos.x, buttonPos.y, buttonPos.h);
                if (startBTNTimer.isOver()) {
                    step = 16;
                }
            break;

            // Go to position to pickup bucket and move arm down to be ready
            case 4:
            armServo.SetDegree(armDownPos);
            DriveTo(prePickupPos.x, prePickupPos.y, prePickupPos.h);
                if (AtPose()) {
                    relocTimer.start(2.0);
                    powerScale = 0.4;
                    step = 5;
                }
            break;
            
            // Slowly drive forward into bucket, if robot is stopped or x coordinate goes past bucket position then go to next state
            case 5:
                DriveAt(0,.25,0);
                if ((abs(vel.x) < 0.02 &&  abs(vel.y) < 0.02 && abs(vel.h) < 0.02 && relocTimer.isOver()) || abs(pos.x) >= abs(pickupPos.x)) {
                    step = 6;
                    pickupTimer.start(0.5);
                    zeroMotors();
                }
            break;

            // Pickup up (wait time to wait for servo to move)
            case 6:
                armServo.SetDegree(armUpPos);
                if (pickupTimer.isOver()) {
                    powerScale = 1;
                    step = 7;
                }
            break;
            
            // Position before going up the ramp
            case 7:
                DriveTo(preRampPos.x, preRampPos.y, preRampPos.h);
                if (AtPose()) {
                    zeroMotors();
                    time1 = TimeNow();
                    relocTimer.start(3.0);
                    Sleep(3.0);
                    step = 8;
                }
            break;
            
            // Wait and read RCS position
            case 8:
                if (TimeNow() - time1 >= 3) {
                    OTOS.getPosition(relocStartPosOTOS);
                    relocStartPosRCS = * RCS.RequestPosition();

                    step = 9;
                }
            
            // Drive up ramp
            case 9:
                DriveTo(upRampPos.x, upRampPos.y, upRampPos.h);
                if (AtPose()) {
                    zeroMotors();
                    relocTimer.start(1);
                    Sleep(1.0);
                    step = 10;
                }
            break;
            
            // Relocalize with RCS
            case 10:
                if (relocTimer.isOver()) {

                    // NOTE: We might not want to be relocalizing heading with RCS, imu may be more accurate ngl, this code is also not accurate since the rotation axis for the robot(OTOS) & RCS are different but for small heading changes it should not be significant.
                    relocEndPosRCS = * RCS.RequestPosition();

                    // Find Pos Difference in RCS
                    float diffX = relocEndPosRCS.x - relocStartPosRCS.x;
                    float diffY = relocEndPosRCS.y - relocStartPosRCS.y;
                    float diffH = relocEndPosRCS.heading - relocStartPosRCS.heading;

                    // Add pos diff to initial OTOS pos; RCS X+ & Y+ -> OTOS X- Y-
                    float newX = relocStartPosOTOS.x - diffX;
                    float newY = relocStartPosOTOS.y - diffY;
                    float newH = relocStartPosOTOS.h;

                    // Assign new pos to OTOS
                    OTOSPose newPos = {newX, newY, newH};
                    OTOS.setPosition(newPos);

                    // LCD.Clear();
                    // LCD.WriteLine(relocStartPosRCS->x);
                    // LCD.WriteLine(relocStartPosRCS->y);
                    // LCD.WriteLine(relocStartPosRCS->heading);
                    // LCD.WriteLine(relocEndPosRCS->x);
                    // LCD.WriteLine(relocEndPosRCS->y);
                    // LCD.WriteLine(relocEndPosRCS->heading);

                    step = 11;
                }
            break;
            
            case 11:
                DriveTo(dropPose.x, dropPose.y, dropPose.h);
                if (AtPose()) {
                    pickupTimer.start(1.0);
                    step = 12;
                }
            break;
            
            case 12:
                DriveTo(dropPose.x, dropPose.y, dropPose.h);
                armServo.SetDegree(armDropPos);
                if (pickupTimer.isOver()) {
                    step = 13;
                }
            break;
            
            case 13:
                DriveTo(postDropPose.x, postDropPose.y, postDropPose.h);
                if (AtPose()) {
                    step = 14;
                }
            break;

            case 14:
                switch(leverIndex) {
                    case 4:
                        leverIndex = RCS.GetLever();
                    break;

                    case 0:
                        DriveTo(preLeftPose.x, preLeftPose.y, preLeftPose.h);
                        if (AtPose()) {
                            step = 15;
                        }
                    break;

                    case 1:
                    DriveTo(preMidPose.x, preMidPose.y, preMidPose.h);
                    if (AtPose()) {
                        step = 15;
                    }
                    break;

                    case 2:
                    DriveTo(preRightPose.x, preRightPose.y, preRightPose.h);
                    if (AtPose()) {
                        step = 15;
                    }
                break;

                }
            break;

            case 15:
                armServo.SetDegree(armDownLeverPos);
                zeroMotors();
            break;
            
            // Lineup for first bucket Rotate
            case 16:
                DriveTo(postRotatePos.x, postRotatePos.y, postRotatePos.h);
                if (AtPose()) {
                    step = 17;
                    freeTimer.start(1.0);
                    zeroMotors();
                    forwardSpinCounter++;
                }
            break;

            case 17:
                armServo.SetDegree(armCompostPos);
                if (freeTimer.isOver()) {
                    DriveTo(preRotatePos.x, preRotatePos.y, preRotatePos.h);
                    if (AtPose()) {
                        armServo.SetDegree(armUpPos);
                        if (forwardSpinCounter >= 5) {
                            step = 18;
                        } else {
                            step = 16;
                        }
                    }
                }
            break;

            case 18:
                DriveTo(preRotatePos.x, preRotatePos.y, preRotatePos.h);
                if (AtPose()) {
                    step = 19;
                    freeTimer.start(1.0);
                    zeroMotors();
                    backwardsSpinCounter++;
                }
            break;

            case 19:
            armServo.SetDegree(armCompostPos);
            if (freeTimer.isOver()) {
                DriveTo(postRotatePos.x, postRotatePos.y, postRotatePos.h);
                if (AtPose()) {
                    armServo.SetDegree(armUpPos);
                    if (backwardsSpinCounter >= 5) {
                        step = 4;
                    } else {
                        step = 18;
                    }
                }
            }
            break;

            case 20:

            break;
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}