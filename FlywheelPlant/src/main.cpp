#include <Arduino.h>

// Raw Encoder Stuff
volatile unsigned int pulseCount = 0;
volatile unsigned long lastTime = 0;

#define InteruptPin 2
#define Encoder_to_RPM 5000 // Gain = RPM/pulse

// rpm reading
int duino_dt = 10; // In miliseconds Should be smaller than matlab`
float rpm = 0;
int timediff = 0;

void addPulse() { pulseCount++; }

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ; // Wait until USB communication is ready
  attachInterrupt(digitalPinToInterrupt(InteruptPin), addPulse, RISING);
  lastTime = millis();
}

void loop() {
  // Read data sent from python
  if (Serial.available() > 0) {
    byte value = Serial.read(); // Grabs the raw byte (0-255)
    analogWrite(9, value);
  }

  // Update RPM every dt miliseconds
  timediff = millis() - lastTime;
  if (timediff >= duino_dt) {
    detachInterrupt(
        digitalPinToInterrupt(InteruptPin)); // Stop counting while calculating
    rpm = (float)pulseCount * Encoder_to_RPM / timediff;

    byte *b = (byte *)&rpm; // Convert float to byte array

    Serial.write(b, 4);

    pulseCount = 0;      // Reset count
    lastTime = millis(); // Reset timer
    attachInterrupt(digitalPinToInterrupt(InteruptPin), addPulse,
                    RISING); // Resume counting
  }
}
