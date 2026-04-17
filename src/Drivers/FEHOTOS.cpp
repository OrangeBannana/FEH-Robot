#include "FEHOTOS.h"
#include <Arduino.h>

FEHOTOS OTOS;

// ─── Init ────────────────────────────────────────────────────────────────────

// TODO: Make wire.begin optional
bool FEHOTOS::begin() {
    Wire.begin();
    uint8_t id;
    if (!readRegs(REG_PRODUCT_ID, &id, 1)) return false;
    return id == PRODUCT_ID;
}

// ─── Unit configuration ───────────────────────────────────────────────────────

void FEHOTOS::setLinearUnit(OTOSLinearUnit unit) {
    if (unit == _linearUnit) return;
    _linearUnit  = unit;
    _meterToUnit = (unit == kLinearUnitMeters) ? 1.0f : kMeterToInch;
}

OTOSLinearUnit FEHOTOS::getLinearUnit() {
    return _linearUnit;
}

void FEHOTOS::setAngularUnit(OTOSAngularUnit unit) {
    if (unit == _angularUnit) return;
    _angularUnit = unit;
    _radToUnit   = (unit == kAngularUnitRadians) ? 1.0f : kRadianToDeg;
}

OTOSAngularUnit FEHOTOS::getAngularUnit() {
    return _angularUnit;
}

// ─── Scalar calibration ───────────────────────────────────────────────────────

bool FEHOTOS::getLinearScalar(float &scalar) {
    uint8_t raw;
    if (!readRegs(REG_SCALAR_LINEAR, &raw, 1)) return false;
    scalar = ((int8_t)raw * 0.001f) + 1.0f;
    return true;
}

bool FEHOTOS::setLinearScalar(float scalar) {
    if (scalar < kMinScalar || scalar > kMaxScalar) return false;
    uint8_t raw = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);
    return writeReg(REG_SCALAR_LINEAR, raw);
}

bool FEHOTOS::getAngularScalar(float &scalar) {
    uint8_t raw;
    if (!readRegs(REG_SCALAR_ANG, &raw, 1)) return false;
    scalar = ((int8_t)raw * 0.001f) + 1.0f;
    return true;
}

bool FEHOTOS::setAngularScalar(float scalar) {
    if (scalar < kMinScalar || scalar > kMaxScalar) return false;
    uint8_t raw = (int8_t)((scalar - 1.0f) * 1000 + 0.5f);
    return writeReg(REG_SCALAR_ANG, raw);
}

// ─── Tracking control ────────────────────────────────────────────────────────

bool FEHOTOS::resetTracking() {
    return writeReg(REG_RESET, 0x01);
}

bool FEHOTOS::calibrateImu(uint8_t numSamples, bool waitUntilDone) {
    if (!writeReg(REG_IMU_CALIB, numSamples)) return false;
    delay(3); // Wait 1 sample period per sfDevOTOS
    if (!waitUntilDone) return true;

    for (uint8_t attempts = numSamples; attempts > 0; attempts--) {
        uint8_t val;
        if (!readRegs(REG_IMU_CALIB, &val, 1)) return false;
        if (val == 0) return true;
        delay(3); // ~2.4ms per sample per sfDevOTOS
    }
    return false; // Timed out
}

// ─── Pose operations ─────────────────────────────────────────────────────────

bool FEHOTOS::getOffset(OTOSPose &pose) {
    return readPoseRegs(REG_OFF_XL, pose, kInt16ToMeter, kInt16ToRad);
}
bool FEHOTOS::setOffset(OTOSPose &pose) {
    return writePoseRegs(REG_OFF_XL, pose, kMeterToInt16, kRadToInt16);
}
bool FEHOTOS::getPosition(OTOSPose &pose) {
    return readPoseRegs(REG_POS_XL, pose, kInt16ToMeter, kInt16ToRad);
}
bool FEHOTOS::setPosition(OTOSPose &pose) {
    return writePoseRegs(REG_POS_XL, pose, kMeterToInt16, kRadToInt16);
}
bool FEHOTOS::getVelocity(OTOSPose &pose) {
    return readPoseRegs(REG_VEL_XL, pose, kInt16ToMps, kInt16ToRps);
}
bool FEHOTOS::getAcceleration(OTOSPose &pose) {
    return readPoseRegs(REG_ACC_XL, pose, kInt16ToMpss, kInt16ToRpss);
}

bool FEHOTOS::getPosVelAcc(OTOSPose &pos, OTOSPose &vel, OTOSPose &acc) {
    uint8_t raw[18];
    if (!readRegs(REG_POS_XL, raw, 18)) return false;
    regsToPose(raw,      pos, kInt16ToMeter, kInt16ToRad);
    regsToPose(raw + 6,  vel, kInt16ToMps,   kInt16ToRps);
    regsToPose(raw + 12, acc, kInt16ToMpss,  kInt16ToRpss);
    return true;
}

// ─── Conversion helpers (mirrors sfDevOTOS exactly) ──────────────────────────

void FEHOTOS::regsToPose(uint8_t *raw, OTOSPose &pose, float rawToXY, float rawToH) {
    int16_t rawX = (raw[1] << 8) | raw[0];
    int16_t rawY = (raw[3] << 8) | raw[2];
    int16_t rawH = (raw[5] << 8) | raw[4];
    pose.x = rawX * rawToXY * _meterToUnit;
    pose.y = rawY * rawToXY * _meterToUnit;
    pose.h = rawH * rawToH  * _radToUnit;
}

void FEHOTOS::poseToRegs(uint8_t *raw, OTOSPose &pose, float xyToRaw, float hToRaw) {
    int16_t rawX = (int16_t)(pose.x * xyToRaw / _meterToUnit);
    int16_t rawY = (int16_t)(pose.y * xyToRaw / _meterToUnit);
    int16_t rawH = (int16_t)(pose.h * hToRaw  / _radToUnit);
    raw[0] = rawX & 0xFF;        raw[1] = (rawX >> 8) & 0xFF;
    raw[2] = rawY & 0xFF;        raw[3] = (rawY >> 8) & 0xFF;
    raw[4] = rawH & 0xFF;        raw[5] = (rawH >> 8) & 0xFF;
}

bool FEHOTOS::readPoseRegs(uint8_t reg, OTOSPose &pose, float rawToXY, float rawToH) {
    uint8_t raw[6];
    if (!readRegs(reg, raw, 6)) return false;
    regsToPose(raw, pose, rawToXY, rawToH);
    return true;
}

bool FEHOTOS::writePoseRegs(uint8_t reg, OTOSPose &pose, float xyToRaw, float hToRaw) {
    uint8_t raw[6];
    poseToRegs(raw, pose, xyToRaw, hToRaw);
    return writeRegs(reg, raw, 6);
}

// ─── I2C helpers ─────────────────────────────────────────────────────────────

bool FEHOTOS::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)I2C_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}

bool FEHOTOS::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool FEHOTOS::writeRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(buf, len);
    return Wire.endTransmission() == 0;
}