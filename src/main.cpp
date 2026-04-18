#include "main.h"

// Enable test mode?
bool testMode = false;

kiwi drivetrain(FEHMotor::Motor2, FEHMotor::Motor3, FEHMotor::Motor1, 12);
cds CDS(FEHIO::Pin0);
FEHServo armServo(FEHServo::Servo0);

// Robot position tracking & pose offset for OTOS
OTOSPose pose;
OTOSPose velPose;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

// Arm Positions
int armUpPos = 180, 
    armDownPos = 128.5,
    armDropPos = 147,
    armIntermediateDropPos = (armDropPos + armUpPos) / 2,
    armCompostPos = 130,
    armDoorPos = 163,
    armTestPos = 132;

timer freeTimer;
timer startBTNTimer;
timer relocTimer;
timer pickupTimer;

void logMode() {
    LCD.Clear();
    drivetrain.zero();
    OTOS.getPosition(pose);

    LCD.WriteLine("X:");
    LCD.WriteLine(pose.x);

    LCD.WriteLine("Y:");
    LCD.WriteLine(pose.y);

    LCD.WriteLine("H:");
    LCD.WriteLine(pose.h);

    FEHLog::printf("X: %.2fY: %.2fH: %.2f", pose.x, pose.y, pose.h);
    Sleep(300);
}

void initialize() {
    drivetrain.setMotorDirections(-1, -1, -1);
    drivetrain.setPowerScalar(7/12);
    drivetrain.setTranslationPDFL(1.3, 0, 0, 0);
    drivetrain.setHeadingPDFL(0.085, 0, 0, 0);

    armServo.SetMin(750);
    armServo.SetMax(2200);
    armServo.SetDegree(armUpPos);
}

void connectSystems() {
    RCS.InitializeTouchMenu("0800A2XNH");
    LCD.WriteLine("RCS Connected");

    LCD.WriteLine("Initializing BLE Logging...");
    FEHLog::enableBLE(152);
    LCD.WriteLine("BLE Logging Initialized");

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
    LCD.WriteLine("Tap thrice to enter logging mode");

    LCD.WaitForTouchToStart();
    freeTimer.start(2.0);
    LCD.WaitForTouchToEnd();

}

void ERCMain()
{   
    initialize();
    connectSystems();

    // State  counter
    int step = 1;

    // Compost Counters
    int forwardSpinCounter = 0;

    if (testMode) {
        step = 0;
    }
    
    LCD.Clear();

    WaitForFinalAction();

    // Main objective switch statement
    while (true) {
        OTOS.getPosition(pose);
        OTOS.getVelocity(velPose);

        drivetrain.setPose(pose);

        switch (step) {

            case 0:
                logMode();
            break;

            case 1:
                drivetrain.setTargetPose(startPos);
                if (drivetrain.atPose()) step = 2;
            break;
            
            case 2:
                drivetrain.setTargetPose(startPos);
                CDS.update();
                if (CDS.Color() == CDSColor::Red) {
                    startBTNTimer.start(0.5);
                    step = 3;
                }
            break;
            
            case 3:
                drivetrain.setTargetPose(buttonPos);
                if (startBTNTimer.isOver()) {
                    step = 16;
                }
            break;

            case 16:
                if (freeTimer.isOver() || forwardSpinCounter == 0) {
                    drivetrain.doPDFL(true, true, true);
                    drivetrain.setTargetPose(forwardRotatePos1);
                    if (drivetrain.atPose()) {
                        step = 17;
                        freeTimer.start(0.5);
                        drivetrain.zero();
                        forwardSpinCounter++;
                    }
                }
            break;

            case 17:
                armServo.SetDegree(armCompostPos);
                if (freeTimer.isOver()) {
                    drivetrain.doPDFL(true, true, true);
                    drivetrain.setTargetPose(forwardRotatePos2);
                    if (drivetrain.atPose() || freeTimer.getTime() >= 3) {
                            step = 18;
                            freeTimer.start(0.5);
                    }
                }
            break;

            case 18:
                drivetrain.setTargetPose(forwardRotatePos3);
                if (drivetrain.atPose()) {
                    if (forwardSpinCounter >= 3) {
                        step = 7;
                        freeTimer.start(0.25);
                    } else {
                        step = 16;
                        freeTimer.start(0.25);
                    }
                    armServo.SetDegree(armUpPos);
                    drivetrain.zero();
                }
            break;

            case 7:
                if (freeTimer.isOver()) {
                    drivetrain.doPDFL(true, true, true);
                    drivetrain.setTargetPose(preOpenPose);
                    if (drivetrain.atPose()) {
                        step = 8;
                        freeTimer.start(1.5);
                    }
                }
            break;
            
            case 8:
                armServo.SetDegree(armDoorPos);
                if (freeTimer.getTime() >= 0.25) {
                    drivetrain.setTargetPose(openPose);
                    if ((drivetrain.atPose() && freeTimer.getTime() >= 1) || freeTimer.isOver()) {
                        step = 4;
                        armServo.SetDegree(armUpPos);
                        freeTimer.start(.65);
                    }
                }
            break;

            case 4:
                if (freeTimer.isOver()){
                    armServo.SetDegree(armDownPos);
                }
                drivetrain.setTargetPose(prePickupPos);
                if (drivetrain.atPose()) {
                    relocTimer.start(2.0);
                    drivetrain.setPowerScalar((7/12) * 0.4);
                    step = 5;
                    freeTimer.start(4.0);
                }
            break;
            
            case 5:
                drivetrain.setDriveVector({0, 0.25, 0});
                drivetrain.doPDFL(false, false, true);
                if ((abs(velPose.x) < 0.02 &&  abs(velPose.y) < 0.02 && abs(velPose.h) < 0.02 && relocTimer.isOver()) || abs(pose.x) >= abs(pickupPos.x) || freeTimer.isOver()) {
                    step = 6;
                    pickupTimer.start(0.5);
                    drivetrain.zero();
                }
            break;

            case 6:
                armServo.SetDegree(armUpPos);
                if (pickupTimer.isOver()) {
                    drivetrain.setPowerScalar(7/12);
                    step = 12;
                }
            break;
            
            case 12:
                drivetrain.setTargetPose(preRampPos);
                drivetrain.doPDFL(true, true, true);
                if (drivetrain.atPose()) {
                    step = 13;
                }
            break;

            case 13:
                drivetrain.setTargetPose(upRampPos);
                if (drivetrain.atPose()) {
                    step = 19;
                    freeTimer.start(0.5);
                    drivetrain.doPDFL(false, false, false);
                }
            break;

            case 19:
                drivetrain.setDriveVector({0, 0.45, 0});
                if ((abs(velPose.x) <= 0.2 && freeTimer.isOver()) || freeTimer.getTime() >= 5) {
                    step = 20;
                    freeTimer.start(0.5);
                }
                break;

            case 20:
                drivetrain.setDriveVector({-0.35, 0.1, -0.05});
                if ((abs(velPose.y) <= 0.1 && freeTimer.isOver()) || freeTimer.getTime() >= 8) {
                    step = 21;
                    freeTimer.start(0.5);
                }
            break;

            case 21:
                drivetrain.setDriveVector({-0.15, 0.15, 0});
                if ((abs(velPose.y) <= 0.1 && abs(velPose.x) <= 0.1 &&  freeTimer.isOver()) || freeTimer.getTime() >= 7) {
                    step = 22;
                    freeTimer.start(0.5);
                }
            break;

            case 22:
                drivetrain.zero();
                Sleep(0.5);
                OTOS.resetTracking();
                Sleep(0.5);
                step = 23;
            break;

            case 23:
                drivetrain.doPDFL(true, true, true);
                drivetrain.setTargetPose(dropPose);
                if (drivetrain.atPose()) {
                    freeTimer.start(0.5);
                    armServo.SetDegree(armIntermediateDropPos);
                    step = 24;
                }
            break;

            case 24:
                if(freeTimer.isOver()) {
                    armServo.SetDegree(armDropPos);
                    if (freeTimer.getTime() >= 1.0){
                    drivetrain.setTargetPose(postDropPose);
                    if (drivetrain.atPose()) {
                        step = 25;
                        armServo.SetDegree(armUpPos);
                    }
                }
                } else {
                    drivetrain.setTargetPose(dropPose);
                }
            break;

            case 25:
                drivetrain.setTargetPose(readLightPos);
                if (drivetrain.atPose()) {
                    step = 26;
                }
            break;

            case 26:
                drivetrain.zero();
                Sleep(0.5);
                CDS.update();

                if (CDS.Color() == CDSColor::Blue) {
                    LCD.WriteLine("blue");
                } else {
                    LCD.WriteLine("red");
                }

                step = 27;
                freeTimer.start(2.0);

            break;

            case 27:
                drivetrain.doPDFL(true, true, true);
                if (CDS.Color() == CDSColor::Blue) {
                    drivetrain.setTargetPose(blueButtonPos);
                } else {
                    drivetrain.setTargetPose(redButtonPos);
                }

                if (drivetrain.atPose() || freeTimer.isOver()) {
                    step  = 28;
                }
            break;

            case 28:
                drivetrain.setTargetPose(preDownRampPos);
                if (drivetrain.atPose()) {
                    step = 29;
                }
            break;

            case 29:
                drivetrain.setTargetPose(finishPos);
            break;

        }

        drivetrain.drive();
        Sleep(10);
    }

}