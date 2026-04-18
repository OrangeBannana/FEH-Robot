#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHLog.h>
#include <FEHRCS.h>
#include <FEHServo.h>

#include "Drivers/FEHOTOS.h"
#include "Controllers/PDFL.h"
#include "Controllers/timer.h"
#include "Mechanisms/kiwi.h"
#include "Mechanisms/cds.h"

// General Robot Positions
OTOSPose zeroPos = {0.0, 0.0, 0.0},
         startPos = {0.3-.25-.15 - .5, -5.18+0.5 + .25, -129.0},
         buttonPos = {-6.5, -4.0, 0},
         rampTransitionPos = {5.0, -16.5, 151.00},
         preRampPos = {-4.81, -16.50, -170.0},
         upRampPos = {-4.5, -42.0, 90},
         preDownRampPos = {8.51, -2.4, -90},
         finishPos = {43, -3, -90};
         
// Robot positions for button milestone OLD - RELATIVE TO OLD RELOCALIZATION
OTOSPose readLightPos = {6.13, -19.5, -180}, // NEW
         blueButtonPos = {5.9, -23.5, -120}, // NEW
         redButtonPos = {3.0, -23.5, -115}; // NEW


// Robot positions for door milestone
OTOSPose preOpenPose = {11.0, -20.75, 180},
         openPose = {4.41, -20.85, 160};

// Robot positions for apple bucket - Some Old
OTOSPose prePickupPos = {6.72, -14.06, -109}, // UP TO DATE
         pickupPos = {9.95, -15.1, -109}, // UP TO DATE
         dropPose = {3.41, -1.72, 90}, 
         postDropPose = {6.5, -1.72, 90};

OTOSPose preMidPose = {6.44, -49.57, -131},
         preRightPose = {3.45, -57.66, -132.56},
         preLeftPose = {10.39, -47.63, -130.0};

// Compost Positions - UP TO DATE
OTOSPose forwardRotatePos1 = {10.8, -6.8, 0},
         forwardRotatePos2 = {10.8, -6.2, 0},
         forwardRotatePos3 = {9.5, -9.5, 0},
         backRotatePos1 = {17, -13.00, 0},
         backRotatePos2 = {17, -9.00, 0};