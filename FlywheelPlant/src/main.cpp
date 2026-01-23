#include <Arduino.h>
#include <queue>

volatile unsigned int pulseCount = 0;
volatile unsigned long lastTime = 0;

float rpm = 0;
int counts_per_revolution = 13;

int u =0;
int duino_dt= 10; //In miliseconds Should be smaller than matlab

std::queue<unsigned int> pulseQueue;
const int MAX_QUEUE_SIZE = 10;

#define MotorGain 1000 // Gain = RPM/Voltage (5000rpm max, 5v max) ) in RPM/V
#define InteruptPin 2


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
    if (Serial.available() > true) {
      byte value = Serial.read(); // Grabs the raw byte (0-255)
      analogWrite(9, value);
  }
 
  // Update RPM every dt miliseconds
  if (millis() - lastTime >= duino_dt) {
    detachInterrupt(digitalPinToInterrupt(InteruptPin)); // Stop counting while calculating
    // Add current pulse count to queue
    pulseQueue.push(pulseCount);
    if (pulseQueue.size() > MAX_QUEUE_SIZE) {
      pulseQueue.pop();
    }
    
    // Calculate moving average of pulse counts
    unsigned int sum = 0;
    std::queue<unsigned int> tempQueue = pulseQueue;
    while (!tempQueue.empty()) {
      sum += tempQueue.front();
      tempQueue.pop();
    }
    float avgPulseCount = (float)sum / pulseQueue.size();
    
    rpm = avgPulseCount/counts_per_revolution; // counts / CPR = Revolutions
    rpm /= (60000/duino_dt); // revolutions / (1 minute / 60 0000ms) = RPM

    Serial.println(rpm); // Send RPM value to MATLAB
    
    lastTime = millis(); // Reset timer
    attachInterrupt(digitalPinToInterrupt(InteruptPin), addPulse, RISING); // Resume counting
  }

}
