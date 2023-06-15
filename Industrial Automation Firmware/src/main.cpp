#include <Arduino.h>

int calibrationTime = 30;
long unsigned int lowIn;
unsigned long motionPause = 5000;

boolean lockLow = true;
boolean takeLowTime;

int pirPin = 34;    // the digital pin connected to the PIR sensor's output
int ledPin = 13;

void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(pirPin, LOW);

  // give the sensor some time to calibrate
  Serial.print("Calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" Done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
}

void loop() {
  if (digitalRead(pirPin) == HIGH) {
    digitalWrite(ledPin, HIGH);   // the LED visualizes the sensor's output pin state
    if (lockLow) {
      lockLow = false;
      Serial.println("---");
      Serial.print("Motion detected at ");
      Serial.print(millis() / 1000);
      Serial.println(" sec");
      delay(50);
    }
    takeLowTime = true;
  }

  if (digitalRead(pirPin) == LOW) {
    digitalWrite(ledPin, LOW);  // the LED visualizes the sensor's output pin state

    if (takeLowTime) {
      lowIn = millis();          // save the time of the transition from high to LOW
      takeLowTime = false;       // make sure this is only done at the start of a LOW phase
    }
    if (!lockLow && millis() - lowIn > motionPause) {
      lockLow = true;
      Serial.print("Motion ended at ");      // output
      Serial.print((millis() - motionPause) / 1000);
      Serial.println(" sec");
      delay(50);
    }
  }
}
