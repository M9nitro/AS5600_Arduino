#include <Arduino.h>
#include "AS5600.h"

AS5600 encoder;

void setup() {
    Serial.begin(115200);

    // Initialize encoder and check magnet
    encoder.EncoderBegin();
}

void loop() {
    // Update encoder measurement
    encoder.update();

    // Get angular velocity in rad/s
    float velocity = encoder.getAngularVelocity();

    Serial.print("Angular Velocity: ");
    Serial.print(velocity);
    Serial.println(" rad/s");

    delay(10);
}
