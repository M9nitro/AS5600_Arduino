// Read the absolute shaft angle and print it in radians and degrees.
// Wiring: SDA/SCL to the board's I2C pins, VCC 3.3V or 5V, DIR pin to GND.
#include <AS5600.h>

AS5600 enc;

void setup() {
  Serial.begin(115200);
  enc.EncoderBegin();            // blocks until a magnet is detected
}

void loop() {
  enc.update();
  float rad = enc.getAngle();    // 0 .. 2*pi, absolute
  Serial.print(rad, 4);
  Serial.print(" rad  ");
  Serial.print(rad * 57.2958f, 1);
  Serial.println(" deg");
  delay(50);
}
