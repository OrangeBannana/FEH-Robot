// Ported from Sparkfun implementation
#include <Wire.h>
#include <stdint.h>

struct OTOSPose {
    float x, y, h;
};

enum OTOSLinearUnit {
    kLinearUnitMeters = 0,
    kLinearUnitInches = 1
};

enum OTOSAngularUnit {
    kAngularUnitRadians = 0,
    kAngularUnitDegrees = 1
};

class FEHOTOS {
public:
    bool begin();

    // Unit configuration (mirrors sfDevOTOS behavior)
    void          setLinearUnit(OTOSLinearUnit unit);
    OTOSLinearUnit getLinearUnit();
    void           setAngularUnit(OTOSAngularUnit unit);
    OTOSAngularUnit getAngularUnit();

    // Scalar calibration
    bool getLinearScalar(float &scalar);
    bool setLinearScalar(float scalar);
    bool getAngularScalar(float &scalar);
    bool setAngularScalar(float scalar);

    // Core pose operations
    bool getPosition(OTOSPose &pose);
    bool setPosition(OTOSPose &pose);
    bool getVelocity(OTOSPose &pose);
    bool getAcceleration(OTOSPose &pose);

    // Bulk read (most efficient for getting all data at once)
    bool getPosVelAcc(OTOSPose &pos, OTOSPose &vel, OTOSPose &acc);

    // Offset (for off-center mounting)
    bool getOffset(OTOSPose &pose);
    bool setOffset(OTOSPose &pose);

    bool resetTracking();
    bool calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);

private:
    static const uint8_t I2C_ADDR    = 0x17;
    static const uint8_t PRODUCT_ID  = 0x5F;

    // Register addresses (directly from sfDevOTOS)
    static const uint8_t REG_PRODUCT_ID    = 0x00;
    static const uint8_t REG_SCALAR_LINEAR = 0x04;
    static const uint8_t REG_SCALAR_ANG    = 0x05;
    static const uint8_t REG_IMU_CALIB     = 0x06;
    static const uint8_t REG_RESET         = 0x07;
    static const uint8_t REG_OFF_XL        = 0x10;
    static const uint8_t REG_POS_XL        = 0x20;
    static const uint8_t REG_VEL_XL        = 0x26;
    static const uint8_t REG_ACC_XL        = 0x2C;

    // Scaling constants (directly from sfDevOTOS)
    static constexpr float kMeterToInch   = 39.37f;
    static constexpr float kRadianToDeg   = 180.0f / 3.14159265f;
    static constexpr float kMeterToInt16  = 32768.0f / 10.0f;
    static constexpr float kInt16ToMeter  = 1.0f / kMeterToInt16;
    static constexpr float kRadToInt16    = 32768.0f / 3.14159265f;
    static constexpr float kInt16ToRad    = 1.0f / kRadToInt16;
    static constexpr float kMpsToInt16    = 32768.0f / 5.0f;
    static constexpr float kInt16ToMps    = 1.0f / kMpsToInt16;
    static constexpr float kMpssToInt16   = 32768.0f / (16.0f * 9.80665f);
    static constexpr float kInt16ToMpss   = 1.0f / kMpssToInt16;
    static constexpr float kRpsToInt16    = 32768.0f / (2000.0f * (3.14159265f / 180.0f));
    static constexpr float kInt16ToRps    = 1.0f / kRpsToInt16;
    static constexpr float kRpssToInt16   = 32768.0f / (3.14159265f * 1000.0f);
    static constexpr float kInt16ToRpss   = 1.0f / kRpssToInt16;
    static constexpr float kMinScalar     = 0.872f;
    static constexpr float kMaxScalar     = 1.127f;

    // Current unit conversion factors (updated by setLinearUnit/setAngularUnit)
    OTOSLinearUnit  _linearUnit  = kLinearUnitInches;
    OTOSAngularUnit _angularUnit = kAngularUnitDegrees;
    float _meterToUnit = kMeterToInch;
    float _radToUnit   = kRadianToDeg;

    // I2C helpers
    bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
    bool writeReg(uint8_t reg, uint8_t val);
    bool writeRegs(uint8_t reg, uint8_t *buf, uint8_t len);

    // Pose conversion helpers (mirrors sfDevOTOS exactly)
    void regsToPose(uint8_t *rawData, OTOSPose &pose, float rawToXY, float rawToH);
    void poseToRegs(uint8_t *rawData, OTOSPose &pose, float xyToRaw, float hToRaw);
    bool readPoseRegs(uint8_t reg, OTOSPose &pose, float rawToXY, float rawToH);
    bool writePoseRegs(uint8_t reg, OTOSPose &pose, float xyToRaw, float hToRaw);
};

extern FEHOTOS OTOS;