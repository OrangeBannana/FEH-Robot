#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLog.h>
#include <FEHRCS.h>
#include <FEHServo.h>

#include "Mechanisms/kiwi.h"
#include "Mechanisms/cds.h"

// General Robot Positions
OTOSPose zeroPos = {0.0, 0.0, 0.0},
         startPos = {0.3-.25-.15 - .5, -5.18+0.5 + .25, -129.0},
         buttonPos = {-6.5, -4.0, 0},
         rampTransitionPos = {5.0, -16.5, 151.00},
         preRampPos = {-4.81, -16.50, -180.0},
         upRampPos = {-1, -42.0, 90},
         preDownRampPos = {8.51, -2.4, -90},
         finishPos = {45, -3, -90},
         finishPos2 = {45, -0, -90};
         
// Robot positions pressing humidifier button
OTOSPose readLightPos = {6.13+.1, -19.5, -180},
         blueButtonPos = {5.9, -23.5, -120},
         redButtonPos = {3.0, -23.5, -115};


// Robot positions for opening door
OTOSPose preOpenPose = {3.4, -19, -109.0},
         openPose = {3.4, -15.45, -82},
         doorTransitionPose = {-0.5, -18.85, -109.0},
         preClosePose = {3.8, -12.45-2.5, -75},
         closePose = {3.7, -12.45-3.25, -120.0};

// Robot positions for picking up and depositing apple bucket
OTOSPose prePickupPos = {6.72, -14.06, -109}, // UP TO DATE
         pickupPos = {10.225, -15., -109}, // UP TO DATE
         dropPose = {3.41, -1.72, 90}, 
         postDropPose = {6.5, -1.72, 90};

// Robot positions for levers
OTOSPose currentLeverPose, currentLeverPose2,
         midLeverPose = {-2.73 - 0.5, -14.81 - 0.5, 135},
         rightLeverPose = {-5.84 - 0.5, -11.73 - 0.5, 135},
         leftLeverPose = {0.44 - 0.5, -17.89 - 0.5, 135},
         midLeverPose2 = {midLeverPose.x + 1.5, midLeverPose.y + 1.5, midLeverPose.h},
         rightLeverPose2 = {rightLeverPose.x + 1.5, rightLeverPose.y + 1.5, rightLeverPose.h},
         leftLeverPose2 = {leftLeverPose.x + 1.5, leftLeverPose.y + 1.5, leftLeverPose.h};

// Compost Positions
OTOSPose compostEngagedPose = {11.21, -6.0, 0},
         compostTransitionPose = {6, -7.5, 0};

OTOSPose controllerTestPose1 = {0, 0, 0},
         controllerTestPose2 = {0, 10, -90},
         controllerTestPose3 = {10, 10, 0},
         controllerTestPose4 = {10, 0, 180};

// Arm Positions
double armUpPos = 180, 
    armDownPos = 130,
    armDropPos = 147,
    armIntermediateDropPos = (armDropPos + armUpPos) / 2,
    armCompostPos = 140,
    armDoorPos = 160.0,
    armDoor2Pos = 140.0,
    armLeverPos = 100.,
    armLeverUpPos = 150.0,
    armTestPos = 132;

// Compost Servo Positions
double CSforward = 180,
       CSbackwards = 0;