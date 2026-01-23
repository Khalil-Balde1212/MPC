#include <Arduino.h>


// Raw Encoder Stuff
volatile unsigned int pulseCount = 0;
volatile unsigned long lastTime = 0;
#define InteruptPin 2


int duino_dt= 10; //In miliseconds Should be smaller than matlab`

float rpm = 0;
#define MotorGain 12500 // Gain = RPM/Voltage (5000rpm max, 5v max) ) in RPM/Volt

int u =0;

// Moving Average Filter
#define FILTER_SIZE 10
float rpmBuffer[FILTER_SIZE] = {0};
int bufferIndex = 0;
float rpmFiltered = 0;




void addPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  while(!Serial); // Wait until USB communication is ready
  attachInterrupt(digitalPinToInterrupt(InteruptPin), addPulse, RISING);
  lastTime = millis();
}

void loop() {
    // Read data sent from MATLAB
    if (Serial.available() > 0) {
      byte value = Serial.read(); // Grabs the raw byte (0-255)
      analogWrite(9, value);
  }
 
  // Update RPM every dt miliseconds
  if (millis() - lastTime >= duino_dt) {
    detachInterrupt(digitalPinToInterrupt(InteruptPin)); // Stop counting while calculating
    rpm = (float)pulseCount*MotorGain;
    
    // Apply moving average filter
    rpmBuffer[bufferIndex] = rpm;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    
    // Calculate average
    rpmFiltered = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      rpmFiltered += rpmBuffer[i];
    }
    rpmFiltered /= FILTER_SIZE;
    rpmFiltered /= duino_dt; // Convert to RPM
    Serial.println(rpmFiltered); // Send filtered RPM value to MATLAB
    
    pulseCount = 0; // Reset count
    lastTime = millis(); // Reset timer
    attachInterrupt(digitalPinToInterrupt(InteruptPin), addPulse, RISING); // Resume counting
  }


}
