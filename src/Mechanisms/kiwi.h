// Implementation of a Kiwi drivetrain

// - Global coordinate system based driving with P2P controller
// - Drivebase relative control with direct drive vector control
// - Mixed control by enabling/disable heading or translational PID's

#include <FEHMotor.h>
#include <FEHLog.h>
#include <FEHUtility.h>

#include "../Drivers/FEHOTOS.h"
#include "../Controllers/PDFL.h"

class kiwi {
    public:

    kiwi(FEHMotor::FEHMotorPort FLport, FEHMotor::FEHMotorPort FRport, FEHMotor::FEHMotorPort BMport, double maxVoltage);
    void setTranslationPDFL(float p, float d, float f, float l) {PDFL tController(p, d, f, l);};
    void setHeadingPDFL(float p, float d, float f, float l) {PDFL hController(p, d, f, l);};
    void setMotorDirections(int FLsign, int FRsign, int BMsign) {this->FLsign = FLsign; this->FRsign = FRsign; this->BMsign = BMsign;};

    void setPose(OTOSPose drivetrainPose) {pose = drivetrainPose;};
    void setTargetPose(OTOSPose targetPose) {this->targetPose = targetPose;};
    void setDriveVector(OTOSPose driveVector) {this->driveVector = driveVector;};
    void drive();

    void doPID(bool doX, bool doY, bool doH) {doXPDFL = doX; doYPDFL = doY; doHPDFL = doH;};
    void setPowerScalar(float powerScalar) {this->powerScalar = powerScalar;};

    private:
        FEHMotor FL;
        FEHMotor FR;
        FEHMotor BM;

        double maxVoltage = 0;

        bool doXPDFL, doYPDFL, doHPDFL;

        PDFL tController;
        PDFL hController;

        int FLsign = 1;
        int FRsign = 1;
        int BMsign = 1;

        float powerScalar;

        OTOSPose pose,
                 targetPose,
                 driveVector;
};