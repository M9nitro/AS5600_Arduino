#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>
// ================= AS5600 velocity encoder =================
#define AS5600_ADDR      0x36
#define REG_STATUS       0x0B
#define REG_ANGLE_H      0x0E
#define RAW_MAX          4096.0f
#define TWO_PI_F         6.283185307f
#define STATUS_MD_BIT    5
#define STATUS_ML_BIT    4
#define STATUS_MH_BIT    3
#define MAX_DELTA_COUNTS 400
#define MIN_DELTA_COUNT  0.1024f


class AS5600 {
public:
    void EncoderBegin(bool overide = false);
    void update();
    float getCenterOffset() const;
    float setCenterOffset(float offset);
    float getAngle() const;
    float getCumalativeAngle() const;
    float getAngularVelocity() const;
    float getAngularAcceleration() const;

private:
    bool     _magnetPresent     = false;
    int      _magnetStatus      = 0;
    uint16_t _lastRaw           = 0;
    uint32_t _lastTime          = 0;
    float    _centerOffset      = 0.0f;
    float    _cumulativeAngle   = 0.0f;
    float    _angle             = 0.0f;
    float    _velocity          = 0.0f;
    float    _lastVelocity      = 0.0f;
    float    _acceleration      = 0.0f;
    

    bool _connect();
    bool _magnetCheck();
    bool readRaw(uint16_t &out);
};

#endif // AS5600_H  