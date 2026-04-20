#include "main.h"

// Enable test modes
bool testMode = false;
bool controllerTestMode = false;

kiwi drivetrain(FEHMotor::Motor2, FEHMotor::Motor3, FEHMotor::Motor1, 12.0);
cds CDS(FEHIO::Pin0);
FEHServo armServo(FEHServo::Servo0);

// Robot position tracking & pose offset for OTOS
OTOSPose pose;
OTOSPose velPose;
OTOSPose posOffset = {1.915, -1.76847, 0.0};

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
    drivetrain.setMotorDirections(-1.0f, -1.0f, -1.0f);
    drivetrain.setTranslationPDFL(0.45, 20.0, 0, 0.06);
    drivetrain.setHeadingPDFL(0.025, 0.0, 0, 0.0);
    drivetrain.setTargetPose({0, 0, 0});

    armServo.SetMin(750);
    armServo.SetMax(2200);
    armServo.SetDegree(armUpPos);
}

void connectSystems() {
    // RCS.InitializeTouchMenu("0800A2XNH");
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

    if (controllerTestMode) {
        step = -1;
    }
    
    LCD.Clear();

    WaitForFinalAction();
    freeTimer.start(3.0);

    // Main objective switch statement
    while (true) {
        OTOS.getPosition(pose);
        OTOS.getVelocity(velPose);

        drivetrain.setPose(pose);

        switch (step) {

            case -1:
                if (drivetrain.atPose() && freeTimer.isOver()) {
                    drivetrain.setTargetPose(controllerTestPose2);
                    step = -2;
                    freeTimer.start(3.0);
                }
            break;

            case -2:
                if (drivetrain.atPose() && freeTimer.isOver()) {
                    drivetrain.setTargetPose(controllerTestPose3);
                    step = -3;
                    freeTimer.start(3.0);
                }
            break;

            case -3:
                if (drivetrain.atPose() && freeTimer.isOver()) {
                    drivetrain.setTargetPose(controllerTestPose4);
                    step = -4;
                    freeTimer.start(3.0);
                }
            break;

            case -4:
                if (drivetrain.atPose() && freeTimer.isOver()) {
                    drivetrain.setTargetPose(controllerTestPose1);
                    step = -1;
                    freeTimer.start(3.0);
                }
            break;

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
                if (CDS.Color() == CDSColor::Red || true) {
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
                    drivetrain.setTargetPose(forwardRotatePos1);
                    if (drivetrain.atPose()) {
                        step = 17;
                        freeTimer.start(0.5);
                        forwardSpinCounter++;
                    }
                }
            break;

            case 17:
                armServo.SetDegree(armCompostPos);
                if (freeTimer.isOver()) {
                    drivetrain.setTargetPose(forwardRotatePos2);
                    drivetrain.doPDFL(true, false, true);
                    drivetrain.setDriveVector({0, 0.5, 0});
                    if (drivetrain.atPose() || freeTimer.getTime() >= 3 || (velPose.magnitude() < 0.4 && drivetrain.moveTime() >= 0.1)) {
                            step = 18;
                            freeTimer.start(0.5);
                            drivetrain.doPDFL(true, true, true);
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
                    if ((drivetrain.atPose() && freeTimer.getTime() >= 1) || freeTimer.isOver() || (velPose.h < 0.1 && velPose.magnitude() < 0.35 && drivetrain.moveTime() >= 0.1)) {
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
                    step = 5;
                    freeTimer.start(4.0);
                }
            break;
            
            case 5:
                drivetrain.setDriveVector({0, 0.2, 0});
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
                    step = 12;
                    drivetrain.setHeadingPDFL(0.015, 0.0, 0, 0.0);
                }
            break;
            
            case 12:
                drivetrain.setTargetPose(preRampPos);
                drivetrain.doPDFL(true, true, true);
                if (drivetrain.atPose()) {
                    drivetrain.doPDFL(false, false, true);
                    drivetrain.setDriveVector({0, 1.0, 0});
                    step = 13;
                }
            break;

            case 13:
                if (drivetrain.moveTime() >= 1.5) {
                    drivetrain.setTargetPose(upRampPos);
                    drivetrain.doPDFL(true, true, true);
                }

                if (drivetrain.atPose() && drivetrain.moveTime() >= 1.6) {
                    step = 19;
                    freeTimer.start(0.5);
                    drivetrain.doPDFL(false, false, false);
                    drivetrain.setHeadingPDFL(0.03, 0.0, 0, 0.0);
                }
            break;

            case 19:
                drivetrain.setDriveVector({0, 0.25, 0});
                if ((abs(velPose.x) <= 0.4 && freeTimer.isOver()) || freeTimer.getTime() >= 5) {
                    step = 20;
                    freeTimer.start(0.5);
                }
                break;

            case 20:
                drivetrain.setDriveVector({-0.25, 0.1, -0.05});
                if ((abs(velPose.y) <= 0.4 && freeTimer.isOver()) || freeTimer.getTime() >= 8) {
                    step = 21;
                    freeTimer.start(0.5);
                }
            break;

            case 21:
                drivetrain.setDriveVector({-0.25, 0.25, 0});
                if ((abs(velPose.y) <= 0.15 && abs(velPose.x) <= 0.15 &&  freeTimer.isOver()) || freeTimer.getTime() >= 7) {
                    step = 22;
                }
            break;

            case 22:
                drivetrain.zero();
                drivetrain.drive();
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
                drivetrain.drive();
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