// Implementation of a Kiwi drivetrain

// - Global coordinate system based driving with P2P controller
// - Drivebase relative control with direct drive vector control
// - Mixed control by enabling/disable heading or translational PID's

#include <FEHMotor.h>
#include <FEHLog.h>
#include <FEHUtility.h>

#include "../Drivers/FEHOTOS.h"
#include "../Controllers/PDFL.h"
#include "../Controllers/timer.h"

class kiwi {
    public:

    kiwi(FEHMotor::FEHMotorPort FLport, FEHMotor::FEHMotorPort FRport, FEHMotor::FEHMotorPort BMport, double maxVoltage) : FL(FLport, maxVoltage), FR(FRport, maxVoltage), BM(BMport, maxVoltage) {this->maxVoltage = maxVoltage; voltageLimit = maxVoltage;};
    void setTranslationPDFL(float p, float d, float f, float l) {tController.setPIDF(p, d, f, l);};
    void setHeadingPDFL(float p, float d, float f, float l) {hController.setPIDF(p, d, f, l);};
    void setMotorDirections(float FLsign, float FRsign, float BMsign) {this->FLsign = FLsign; this->FRsign = FRsign; this->BMsign = BMsign;};

    void setPose(OTOSPose drivetrainPose) {pose = drivetrainPose;};
    void setTargetPose(OTOSPose targetPose) {if (targetPose != this->targetPose) {moveTimer.start(moveTimeout);}; this->targetPose = targetPose;};
    void setDriveVector(OTOSPose driveVector) {this->driveVector = driveVector;};
    void drive();
    void zero() {doPDFL(false, false, false); setDriveVector({0, 0, 0});};

    bool atPose();
    bool atPoseXY();
    bool atPoseH();
    bool nearPose();
    bool timedOut() {return moveTimer.isOver();};
    float moveTime() {return moveTimer.getTime();};
    

    void doPDFL(bool doX, bool doY, bool doH) {doXPDFL = doX; doYPDFL = doY; doHPDFL = doH;};
    void setVoltageLimit(float voltageLimit) {this->voltageLimit = voltageLimit;};

    private:
        FEHMotor FL;
        FEHMotor FR;
        FEHMotor BM;

        double maxVoltage;

        bool doXPDFL = true,
             doYPDFL = true,
             doHPDFL = true;

        PDFL tController;
        PDFL hController;

        float FLsign = 1.0f;
        float FRsign = 1.0f;
        float BMsign = 1.0f;

        float voltageLimit;

        float vX,
              vY,
              vTheta;

        OTOSPose pose,
                 targetPose,
                 driveVector;

        timer moveTimer;

        float moveTimeout = 1.0;

        float upSlewLimit = 0.1;
        float downSlewLimit = 0.07;
};