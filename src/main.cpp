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
    armDownPos = 128.5,
    armDropPos = 147,
    armDownLeverPos = 90,
    armCompostPos = 132, // UPDATED 4/15
    armDoorPos = 160, //UPDATED 4/15
    armTestPos = 132;

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
PDFL tController(1.3, 0, 0, 0);
PDFL hController(0.085, 0, 0, 0);

// Initial Target Positions
float targetX = 0;
float targetY = 0;
float targetH = 0;

// Robot position tracking
OTOSPose pos;
OTOSPose vel;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

// General Robot Positions
OTOSPose zeroPos = {0.0, 0.0, 0.0},
         startPos = {0.3-.25-.15, -5.18+0.5 + .25, -129.0},
         buttonPos = {-3.2, -4.5, -155.0},
         rampTransitionPos = {5.0, -16.5, 151.00},
         preRampPos = {-4.81, -16.50, -170.0},
         upRampPos = {-5, -42.0, 90},
         calibrationPos = {-2.75, -40.4, 90};
         
// Robot positions for button milestone OLD - RELATIVE TO OLD RELOCALIZATION
OTOSPose readLightPos = {6.12, -18.54, -177},
         blueButtonPos = {7.79, -23.0, -177},
         redButtonPos = {-0.3, -25.0, -177},
         downRampPos = {9.70, -3.23, 90},
         finishPos = {43.68, -2.0, 90};

// Robot positions for door milestone
OTOSPose preOpenPose = {10.25, -20.85, 180},
         openPose = {4.41, -20.85, 180};

// Robot positions for apple bucket milestone - Some Old
OTOSPose prePickupPos = {6.72, -14.06, -109},
         pickupPos = {9.95, -15.1, -109},
         dropPose = {-5.32, -49.54, -175},
         postDropPose = {-5.32, -45.54, -175},
         preMidPose = {6.44, -49.57, -131},
         preRightPose = {3.45, -57.66, -132.56},
         preLeftPose = {10.39, -47.63, -130.0};

// Compost Positions - UP TO DATE
OTOSPose forwardRotatePos1 = {10.8, -7.5, 0},
         forwardRotatePos2 = {10.8, -6.2, 0},
         forwardRotatePos3 = {9.5, -9.5, 0},
         backRotatePos1 = {17, -13.00, 0},
         backRotatePos2 = {17, -9.00, 0};


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
    hError = fmod(fmod(hError + 180, 360) + 360, 360) - 180;

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
    //RCS.InitializeTouchMenu("0800A2XNH");
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

    // State  counter
    int step = 1;

    // Compost Counters
    int forwardSpinCounter = 0;

    if (testMode) {
        step = 0;
    }
    
    LCD.Clear();

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
                if (abs(redVal - CDS.Value()) < 0.35 || true) {
                    startBTNTimer.start(0.5);
                    step = 3;
                }
            break;
            
            // Pressing Start Button
            case 3:
                DriveTo(buttonPos.x, buttonPos.y, buttonPos.h);
                if (startBTNTimer.isOver()) {
                    step = 16;
                    freeTimer.start(0.0);
                }
            break;

            // Lineup with claw up, over bucket
            case 16:
                if (freeTimer.isOver() || forwardSpinCounter == 0) {
                DriveTo(forwardRotatePos1.x, forwardRotatePos1.y, forwardRotatePos1.h);
                if (AtPose()) {
                    step = 17;
                    freeTimer.start(0.5);
                    zeroMotors();
                    forwardSpinCounter++;
                }
            }
            break;



            case 17:
                armServo.SetDegree(armCompostPos);
                if (freeTimer.isOver()) {
                    DriveTo(forwardRotatePos2.x, forwardRotatePos2.y, forwardRotatePos2.h);
                    if (AtPose() || freeTimer.getTime() >= 3) {
                            step = 18;
                            freeTimer.start(0.5);
                    }


                }
            break;

            case 18:
                DriveTo(forwardRotatePos3.x, forwardRotatePos3.y, forwardRotatePos3.h);
                if (AtPose()) {
                    if (forwardSpinCounter >= 3) {
                        step = 7;
                        freeTimer.start(0.25);
                    } else {
                        step = 16;
                        freeTimer.start(0.25);
                    }
                    armServo.SetDegree(armUpPos);
                    
                    zeroMotors();


                }
            break;

            // Position before opening door
            case 7:
                if (freeTimer.isOver()) {
                DriveTo(preOpenPose.x, preOpenPose.y, preOpenPose.h);
                if (AtPose()) {
                    step = 8;
                    freeTimer.start(1.5);
                }
            }
            break;
            
            // Door Opened
            case 8:
                armServo.SetDegree(armDoorPos);
                if (freeTimer.getTime() >= 0.25) {
                DriveTo(openPose.x, openPose.y, openPose.h);
                if ((AtPose() && freeTimer.getTime() >= 1) || freeTimer.isOver()) {
                    step = 4;
                    armServo.SetDegree(armUpPos);
                    freeTimer.start(.65);
                }
            }
            break;

            // Go to position to pickup bucket and move arm down to be ready
            case 4:
            if (freeTimer.isOver()){
            armServo.SetDegree(armDownPos);
            }
            DriveTo(prePickupPos.x, prePickupPos.y, prePickupPos.h);
                if (AtPose()) {
                    relocTimer.start(2.0);
                    powerScale = 0.4;
                    step = 5;
                    freeTimer.start(4.0);
                }
            break;
            
            // Slowly drive forward into bucket, if robot is stopped or x coordinate goes past bucket position then go to next state
            case 5:
                DriveAt(0,.25,0);
                if ((abs(vel.x) < 0.02 &&  abs(vel.y) < 0.02 && abs(vel.h) < 0.02 && relocTimer.isOver()) || abs(pos.x) >= abs(pickupPos.x) || freeTimer.isOver()) {
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
                    step = 12;
                }
            break;
            
            case 12:
                DriveTo(preRampPos.x, preRampPos.y, preRampPos.h);
                if (AtPose()) {
                    step = 13;
                }
            break;

            case 13:
                DriveTo(upRampPos.x, upRampPos.y, upRampPos.h);
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

            case 21:

                zeroMotors();

            break;
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}