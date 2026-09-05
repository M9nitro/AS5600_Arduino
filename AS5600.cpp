#include "./AS5600.h"

/*
Debug to check if magnet is in potimal position.
*/
bool AS5600::magnetOk() {
    uint8_t s;
    if (!_readReg(REG_STATUS, s)) return false;          // no ACK = no sensor
    return (s & (1 << STATUS_MD_BIT)) && !(s & ((1 << STATUS_ML_BIT) | (1 << STATUS_MH_BIT)));
}

/*
Shows magnet status
*/
uint8_t AS5600::magnetStatus() {
    uint8_t s;
    return _readReg(REG_STATUS, s) ? s : 0;
}

/*
Connect Function Utility function.
*/
bool AS5600::_connect() {
    Wire.begin();
    Wire.setClock(100000);
    return true;
}

/*
Check at the Begining Magnet Position.
*/
bool AS5600::_magnetCheck() {
    uint8_t s;
    if (!_readReg(REG_STATUS, s)) { Serial.println("Encoder not responding"); return false; }
    AS5600::_magnetStatus = s;
    bool magnetDetected = s & (1 << STATUS_MD_BIT);
    bool magnetTooLow   = s & (1 << STATUS_ML_BIT);
    bool magnetTooHigh  = s & (1 << STATUS_MH_BIT);
    AS5600::_magnetPresent = magnetDetected && !magnetTooLow && !magnetTooHigh;
    if (!AS5600::_magnetPresent) {
        Serial.print("Magnet Status: ");
        if (!magnetDetected) Serial.print("Not Detected ");
        if (magnetTooLow)    Serial.print("Magnet Too Low ");
        if (magnetTooHigh)   Serial.print("Magnet Too High ");
        Serial.println();
    }
    return AS5600::_magnetPresent;
}

void AS5600::EncoderBegin(bool overide, float center) {
    AS5600::_connect();
   while (!AS5600::_magnetCheck() && !overide) {
            delay(100);
            Serial.println("Encoder Missing");
        }
    AS5600::setFilter(SF_16X, FTH_OFF, HYST_3); //Agressive Conf
    AS5600::_readAngle(AS5600::_lastRaw);       //Read for Syncing up Update in the first GO.
    AS5600::_center = center;                   // Using the Center Variable
    AS5600::_cumulativeAngle = ((float)AS5600::_lastRaw/RAW_MAX)*TWO_PI_F - center; //Compensate for the center angle
    AS5600::_lastTime = micros();               // Remember time for velocity.
   


}

bool AS5600::_readReg(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

bool AS5600::_writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Volatile: survives until power-off. Never burns.
bool AS5600::setFilter(SlowFilter sf, FastFilter fth, Hysteresis hyst) {
    uint8_t hi, lo;
    if (!_readReg(REG_CONF_H, hi) || !_readReg(REG_HYST, lo)) return false;
    hi = (hi & ~0x1F) | ((fth & 0x07) << 2) | (sf & 0x03);   // keep WD (bit 5)
    lo = (lo & ~0x0C) | ((hyst & 0x03) << 2);                // keep PM/OUTS/PWMF
    return _writeReg(REG_CONF_H, hi) && _writeReg(REG_HYST, lo);
}

/*
Read the Processed Angle. With Hysteresis
*/
bool AS5600::_readAngle(uint16_t &out) {

    if (!AS5600::magnetOk()) return false;

    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(REG_ANGLE_H);
    if (Wire.endTransmission(false) != 0 ) return false;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;

    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();

    out = ((hi << 8) | lo )& 0x0FFF;

    return true;
}

/*
Raw Angle for angle/velocity/Accerlaration without hysteresis.
*/
bool AS5600::_readRawAngle(uint16_t &out) {

    if (!AS5600::magnetOk()) return false; 

    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(REG_RAW_ANGLE_H);
    if (Wire.endTransmission(false) != 0 ) return false;
    if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;

    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();

    out = ((hi << 8) | lo )& 0x0FFF;

    return true;

}

void AS5600::update() {
    uint16_t raw;
    if (!AS5600::_readAngle(raw)) return;                  // ANGLE reg, not RAW: hysteresis only filters ANGLE, kills the 1 LSB dither

    AS5600::_angle = ((float)raw/RAW_MAX)*TWO_PI_F;         // absolute angle, radians

    uint32_t now = micros();
    float dt = (now - AS5600::_lastTime) * 1e-6f;
    if (dt < MIN_DT_S) return;                              // angle already refreshed above; derivatives wait for a full window

    float delta = (float)raw - (float)AS5600::_lastRaw;
    if (delta > 2048) delta -= 4096;                        // unwrap 4095 -> 0 rollover
    if (delta < -2048) delta += 4096;

    // ponytail: steering can't move 400 counts (35 deg) in one loop, so treat it as a glitch:
    // skip the derivative math but still resync so the next sample is compared against real data.
    if (abs(delta) <= MAX_DELTA_COUNTS) {
        float dAngle = (delta/RAW_MAX)*TWO_PI_F;
        AS5600::_velocity = dAngle/dt;                       // 0 when stationary
        AS5600::_cumulativeAngle += dAngle;
        AS5600::_acceleration = (AS5600::_velocity - AS5600::_lastVelocity)/dt;
        AS5600::_lastVelocity = AS5600::_velocity;
    } else {
        AS5600::_velocity = AS5600::_lastVelocity = AS5600::_acceleration = 0.0f; // stall/glitch reads as stopped, not stale
    }

    AS5600::_lastTime = now;                                // always advance, even on a rejected sample
    AS5600::_lastRaw = raw;
}

/*
    Update Here necessary. 
*/
void AS5600::setCenter()  { AS5600::_center = AS5600::_angle;}

// ---- steering: all relative to _center, scaled to the steering shaft ----
float AS5600::getCenter() const { return AS5600::_center; }

float AS5600::getAngle() const { return AS5600::_angle; }                       //Absolute angle. 
float AS5600::getCumalativeAngle() const {return AS5600::_cumulativeAngle;}     //CumlativeAngle

float AS5600::getSteeringAngle() const { return DIRECTION* AS5600::_cumulativeAngle / GEAR_RATIO;}  //Adjusted According to the Reduction ratio
float AS5600::getSteeringVelocity() const { return DIRECTION * AS5600::_velocity / GEAR_RATIO; }    //Adjusted According to the Reduction ratio

float AS5600::getAngularVelocity() const { return AS5600::_velocity; }          //Sensor shaft, rad/s
float  AS5600::getAngularAcceleration() const { return AS5600::_acceleration; } //Acceleration, rad/s/s

