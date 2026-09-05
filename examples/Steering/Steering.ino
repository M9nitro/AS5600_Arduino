// Steering-wheel angle through a geared encoder, relative to a calibrated center.
// GEAR_RATIO and DIRECTION live in AS5600.h; change them there to match your linkage.
//
// Calibration: hold the wheel straight, send 'c' over serial, then paste the printed
// value into CENTER below (or store it in EEPROM) so it survives a reboot.
#include <AS5600.h>

const float CENTER = 0.0f;       // radians at the sensor shaft, from a previous 'c'

AS5600 enc;

void setup() {
  Serial.begin(115200);
  enc.EncoderBegin(false, CENTER);
  Serial.println("Send 'c' with the wheel straight to read the center offset.");
}

void loop() {
  enc.update();

  if (Serial.available() && Serial.read() == 'c') {
    Serial.print("center = ");
    Serial.println(enc.getAngle(), 4);      // pass this to EncoderBegin() next boot
  }

  Serial.print("steer ");
  Serial.print(enc.getSteeringAngle() * 57.2958f, 1);   // degrees at the steering shaft
  Serial.print(" deg  ");
  Serial.print(enc.getSteeringVelocity(), 2);
  Serial.println(" rad/s");
  delay(50);
}
