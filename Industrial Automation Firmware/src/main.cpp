#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <BH1750.h>

#define WIFI_SSID "Redmi Note 7"
#define WIFI_PASSWORD "Anush123ga"
// #define WIFI_SSID "Eng-Student"
// #define WIFI_PASSWORD "3nG5tuDt"

// Create 2 tasks for each processor
TaskHandle_t Task1;
TaskHandle_t Task2;

// Initiate BH1750 sensor
BH1750 lightMeter;

int calibrationTime = 15;

int pirPin = 34; // the digital pin 34 connected to the PIR sensor's output
int ledPin = 13;
const int led1 = 33;

// Function signatures
void connectWiFi();
extern void pirSensor(int, int);
extern void lightIntensity(BH1750);
extern String getDateTime();

void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

void setup()
{
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(led1, OUTPUT);
  digitalWrite(pirPin, LOW);

  // give the sensor some time to calibrate
  Serial.print("Calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" Done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
  // End of calibrating sensor

  // BH1750 Sensor
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  connectWiFi();

  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      0);        /* pin task to core 0 */
  delay(500);

  // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 1 */
  delay(500);
}

// Task1code: blinks an LED every 1000 ms
void Task1code(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  // Measuring light intensity from BH1750
  lightIntensity(lightMeter);
}

// Task2code: PIR sensor
void Task2code(void *pvParameters)
{
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  // Calling pir sensor to check occupancy
  pirSensor(pirPin, ledPin);
}

void loop()
{
}

void connectWiFi()
{

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
  }

  // Serial.print("Connected with IP: ");
  // Serial.println(WiFi.localIP());
}
