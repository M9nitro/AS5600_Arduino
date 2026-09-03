#include "./AS5600.h"

bool AS5600::_connect() {
    Wire.begin();
    Wire.setClock(100000);
    return true;
}

bool AS5600::_magnetCheck() {
    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(REG_STATUS);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)AS5600_ADDR, 1);
    while (Wire.available() == 0){
        delay(1);
    
    AS5600::_magnetStatus = Wire.read();
    bool magnetDetected = (AS5600::_magnetStatus & (1 << STATUS_MD_BIT)) != 0;
    bool magnetTooLow = (AS5600::_magnetStatus & (1 << STATUS_ML_BIT)) != 0;
    bool magnetTooHigh = (AS5600::_magnetStatus & (1 << STATUS_MH_BIT)) != 0;

    if (magnetDetected && !magnetTooLow && !magnetTooHigh) {
        AS5600::_magnetPresent = true;
        return true;
    } else {
        AS5600::_magnetPresent = false;
        Serial.print("Magnet Status: ");
        if (!magnetDetected) Serial.print("Not Detected ");
        if (magnetTooLow) Serial.print("Magnet Too Low ");
        if (magnetTooHigh) Serial.print("Magnet Too High ");
        return false;

    }
    }
    
}

void AS5600::EncoderBegin(bool overide = false) {
    AS5600::_connect();
   while (!AS5600::_magnetCheck() | !overide) {
    delay(100);
    if (!overide) {
        Serial.println("Starting without Encoder");
        AS5600::_magnetPresent = false;
    }
   }


}

bool AS5600::readRaw(uint16_t &out) {

    //Check if the magnet is correctly Placed
    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(REG_STATUS);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, 1) != 1) return false;
    uint8_t status = Wire.read();
        if (!(status & (1 << STATUS_MD_BIT))) return false;
        if (status & (1 << STATUS_ML_BIT)) return false;
        if (status & (1 << STATUS_MH_BIT)) return false;

    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(REG_ANGLE_H);
    if (Wire.endTransmission(false) != 0 ) return false;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, 2) != 1) return false;

    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();

    out = (hi << 8) | lo & 0x0FFF;

    return true;

}

void AS5600::update() {
    uint16_t raw; // Raw angle value from the encoder
    if (!readRaw(raw)) return; // Read the Raw Angle from the AS5600 sensor
    AS5600::_angle = ((float)raw/RAW_MAX)*TWO_PI_F; // Absolute Angle in Radians
    uint16_t now = micros(); // Current time in microseconds
    float dt = (now - AS5600::_lastTime) * 1e-6f; // Time difference in seconds
    if (dt < 0.0005f) return; // Ignore updates that are too close together (less than 0.5 ms)
    int32_t delta = (int32_t)raw - (int32_t)AS5600::_lastRaw; // Change in raw angle counts
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    if (abs(delta) > MAX_DELTA_COUNTS) return; // Ignore updates that are too large (greater than 400 counts)
    if (abs(delta) < MIN_DELTA_COUNT) return; // Ignore updates that are too small (less than 0.1024 counts)
    AS5600::_velocity = ((delta/RAW_MAX)*TWO_PI_F)/dt; // Angular velocity in radians per second
    AS5600::_cumulativeAngle += (delta/RAW_MAX)*TWO_PI_F; // Cumulative angle in radians
    AS5600::_accerleration = (AS5600::_velocity - AS5600::_lastVelocity)/dt; // Angular acceleration in radians per second squared
    // Update the last time, last raw angle, and last velocity for the next update
    AS5600::_lastTime = now;
    AS5600::_lastRaw = raw;
    AS5600::_lastVelocity = AS5600::_velocity;
    }

float AS5600::getCenterOffset() const { 
    AS5600::_centerOffset = AS5600::_angle;
    return AS5600::_centerOffset; }
float AS5600::getAngularAngle() const { return AS5600::_angle; }
float  AS5600::getAngularVelocity() const { return AS5600::_velocity; }
float  AS5600::getAngularAcceleration() const { return AS5600::_accerleration; }

