#include <Arduino.h>

// Create 2 tasks for each processor
TaskHandle_t Task1;
TaskHandle_t Task2;

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
  // End of calibrating sensor

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500);

}


//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
  } 
}

//Task2code: PIR sensor
void Task2code( void * pvParameters ){
  for(;;) {
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
}

void loop() {
  
}
