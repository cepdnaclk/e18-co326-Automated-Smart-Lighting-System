#include <Arduino.h>

extern String getDateTime();

// the time when the sensor outputs a low impulse
long unsigned int lowIn;
// the amount of milliseconds the sensor has to be low
// before we assume all motion has stopped
long unsigned int motionPause = 5000;

boolean lockLow = true;
boolean takeLowTime;

String UoP_CO_326_E18_Gr18_PIR_timeStamp = "";
boolean UoP_CO_326_E18_Gr18_PIR_data = false;           // true: movement detected, false: no movement

// Function for get PIR sensor data to check the occupancy
void pirSensor(int pirPin, int ledPin)
{
  for (;;)
  {
    if (digitalRead(pirPin) == HIGH)
    {
      digitalWrite(ledPin, HIGH); // the LED visualizes the sensor's output pin state
      if (lockLow)
      {
        UoP_CO_326_E18_Gr18_PIR_data = true;
        UoP_CO_326_E18_Gr18_PIR_timeStamp = getDateTime();
        lockLow = false;
        Serial.println("---");
        Serial.print("Motion detected at ");
        Serial.println(UoP_CO_326_E18_Gr18_PIR_timeStamp);
        delay(50);
      }
      takeLowTime = true;
    }

    if (digitalRead(pirPin) == LOW)
    {
      digitalWrite(ledPin, LOW); // the LED visualizes the sensor's output pin state

      if (takeLowTime)
      {
        lowIn = millis();    // save the time of the transition from high to LOW
        takeLowTime = false; // make sure this is only done at the start of a LOW phase
      }
      if (!lockLow && millis() - lowIn > motionPause)
      {
        lockLow = true;
        UoP_CO_326_E18_Gr18_PIR_data = false;
        UoP_CO_326_E18_Gr18_PIR_timeStamp = getDateTime();
        Serial.print("Motion ended at "); // output
        Serial.println(UoP_CO_326_E18_Gr18_PIR_timeStamp);
        delay(50);
      }
    }
  }
}