// Diagnose magnet placement. Use this first if EncoderBegin() keeps printing "Encoder Missing".
// Ideal air gap is 0.5 - 3 mm with the magnet centered over the chip.
#include <AS5600.h>

AS5600 enc;

void setup() {
  Serial.begin(115200);
  enc.EncoderBegin(true);        // override: don't block on a missing magnet
}

void loop() {
  uint8_t s = enc.magnetStatus();
  if (s == 0)                      Serial.println("no I2C reply, check wiring");
  else if (!(s & (1 << STATUS_MD_BIT))) Serial.println("no magnet detected");
  else if (s & (1 << STATUS_ML_BIT))    Serial.println("magnet too weak, move closer");
  else if (s & (1 << STATUS_MH_BIT))    Serial.println("magnet too strong, move away");
  else                             Serial.println("magnet OK");
  delay(500);
}
