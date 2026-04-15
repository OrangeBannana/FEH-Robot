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
    armDownPos = 128.0,
    armDropPos = 147,
    armDownLeverPos = 90,
    armCompostPos = 110,
    armCompostPos2 = 110,
    armDoorPos = 165;

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

// General Robot Positions
OTOSPose zeroPos = {0.0, 0.0, 0.0},
         startPos = {0.3-.25, -5.18+0.5, -129.0},
         buttonPos = {-3.2, -4.5, -155.0},
         preCompostPos = {9, -5, -38.87},
         compostPos = {8.94, -0.67, -29};

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

    // State  counter
    int step = 1;

    // Compost Counters
    int forwardSpinCounter = 0,
        backwardsSpinCounter = 0;

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
                if (abs(redVal - CDS.Value()) < 0.35) {
                    startBTNTimer.start(0.5);
                    step = 3;
                }
            break;
            
            // Pressing Start Button
            case 3:
                DriveTo(buttonPos.x, buttonPos.y, buttonPos.h);
                if (startBTNTimer.isOver()) {
                    step = 4;
                }
            break;

           case 4:
                DriveTo(preCompostPos.x, preCompostPos.y, preCompostPos.h);
                if (AtPose()) {
                    step = 5;
                    freeTimer.start(3.0);
                }
                break;

            case 5:
            DriveTo(compostPos.x, compostPos.y, compostPos.h);
            if (AtPose() || freeTimer.isOver()) {
                step = 6;
            }
            break;

            case 6:
            DriveTo(buttonPos.x, buttonPos.y, buttonPos.h);
            break;
            
        }
        // 10ms sleep to slow looptime a little
        Sleep(10);
    }

}