#include "./AS5600.h"
#include <Wire.h>

bool AS5600::_connect() {
    Wire.begin();
    Wire.setClock(100000);
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
    uint16_t raw;
    if (!readRaw(raw)) return;
    uint16_t now = micros();

    float dt = (now - AS5600::_lastTime) * 1e-6f;
    if (dt < 0.0005f) return;
    int32_t delta = (int32_t)raw - (int32_t)AS5600::_lastRaw;
    if (abs(delta) > MAX_DELTA_COUNTS) return;
    if (abs(delta) < MIN_DELTA_COUNT) return; 
    AS5600::_velocity = ((delta/RAW_MAX)*TWO_PI_F)/dt;
    _lastTime = now;
    _lastRaw = raw;
    }

float  AS5600::getAngularVelocity() const { return AS5600::_velocity; }

