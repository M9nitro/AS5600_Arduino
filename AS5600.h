#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>
// ================= AS5600 velocity encoder =================
#define AS5600_ADDR      0x36
#define REG_STATUS       0x0B
#define REG_CONF_H       0x07
#define REG_HYST         0x08
#define REG_ANGLE_H      0x0E
#define REG_RAW_ANGLE_H  0x0C
#define RAW_MAX          4096.0f
#define TWO_PI_F         6.283185307f
#define STATUS_MD_BIT    5
#define STATUS_ML_BIT    4
#define STATUS_MH_BIT    3
#define MAX_DELTA_COUNTS 400
#define MIN_DT_S         0.010f  // velocity window. 1 count / 10 ms = 3 deg/s steering resolution; raise for less noise, lower for faster response
#define GEAR_RATIO       3.00f
#define DIRECTION        -1.0f   // flip to +1.0f if positive steering reads negative


enum SlowFilter : uint8_t { SF_16X = 0, SF_8X = 1, SF_4X = 2, SF_2x = 3};
enum FastFilter : uint8_t { FTH_OFF = 0, FTH_6 = 1, FTH_7 = 2, FTH_9 = 3, FTH_18 = 4, FTH_21 = 5, FTH_24 = 6, FTH_10 = 7 };
enum Hysteresis : uint8_t { HYST_OFF = 0, HYST_1 = 1, HYST_2 = 2, HYST_3 = 3 };


class AS5600 {
public:
    void EncoderBegin(bool overide = false);
    void update();

    void setCenter();
    void setCenter(float sensorRad);

    float getCenter() const ;
    float getSteeringAngle() const;
    float getSteeringVelocity() const; 

    float getAngle() const;
    float getCumalativeAngle() const;
    float getAngularVelocity() const;
    float getAngularAcceleration() const;
    bool setFilter(SlowFilter sf, FastFilter fth, Hysteresis hyst);
    bool magnetOk();
    uint8_t magnetStatus();                 // raw STATUS reg: bit5 MD detected, bit4 ML too weak, bit3 MH too strong. 0 = no reply


private:
    bool     _magnetPresent     = false;
    int      _magnetStatus      = 0;
        
    uint16_t _lastRaw           = 0;
    uint32_t _lastTime          = 0;


    float    _center            = 0.0f;
    float    _cumulativeAngle   = 0.0f;
    float    _angle             = 0.0f;
    float    _velocity          = 0.0f;
    float    _lastVelocity      = 0.0f;
    float    _acceleration      = 0.0f;
 
    // float    _scaledAngle       = 0.0f;
    // float    _lastScaledAngle   = 0.0f;
    // float    _centerOffset      = 0.0f;
    // int      _count             = 0;

    bool _connect();
    bool _magnetCheck();
    bool _readAngle(uint16_t &out);
    bool _readRawAngle(uint16_t &out); 
    bool _readReg(uint8_t reg, uint8_t &val);
    bool _writeReg(uint8_t reg, uint8_t val);

};

#endif // AS5600_H  