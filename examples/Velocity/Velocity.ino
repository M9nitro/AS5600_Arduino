// Cumulative angle, angular velocity and acceleration of the sensor shaft.
// Cumulative angle keeps counting past one turn, so it's the one to use for multi-turn tracking.
#include <AS5600.h>

AS5600 enc;

void setup() {
  Serial.begin(115200);
  enc.EncoderBegin();
  Serial.println("turns\trad/s\trad/s2");
}

void loop() {
  enc.update();                                 // call as often as possible; derivatives update every MIN_DT_S
  Serial.print(enc.getCumalativeAngle() / TWO_PI_F, 3);
  Serial.print('\t');
  Serial.print(enc.getAngularVelocity(), 3);
  Serial.print('\t');
  Serial.println(enc.getAngularAcceleration(), 1);
  delay(20);
}
