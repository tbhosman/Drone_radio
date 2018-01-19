#include <Arduino.h>

const int pinThrottle = A3;
const int pinRoll = A0;
const int pinPitch = A1;
const int pinYaw = A2;

void setup() {
    Serial.begin(9600);
}

void loop() {
    Serial.print("\t");
    Serial.print("Throttle: ");
    Serial.print(analogRead(pinThrottle));
    Serial.print("\t");
    Serial.print("Roll: ");
    Serial.print(analogRead(pinRoll));    
    Serial.print("\t");
    Serial.print("Pitch: ");
    Serial.print(analogRead(pinPitch));
    Serial.print("\t");
    Serial.print("Yaw: ");
    Serial.print(analogRead(pinYaw));
    Serial.print("\n");

}