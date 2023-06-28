#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define WIFI_SSID "Redmi Note 7"
#define WIFI_PASSWORD "Anush123ga"
// #define WIFI_SSID "Eng-Student"
// #define WIFI_PASSWORD "3nG5tuDt"

// Create 2 tasks for each processor
TaskHandle_t Task1;
TaskHandle_t Task2;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Initiate BH1750 sensor
BH1750 lightMeter;

int calibrationTime = 15;
boolean lockLow = true;
boolean takeLowTime;
String dateString = "";
String timeString = "";

// Variables to store values that need to send to MQTT
String UoP_CO_326_E18_Gr18_PIR_timeStamp = "";
boolean UoP_CO_326_E18_Gr18_PIR_data = false;           // true: movement detected, false: no movement

String UoP_CO_326_E18_Gr18_BH1750_timeStamp = "";
float UoP_CO_326_E18_Gr18_BH1750_data;


// the time when the sensor outputs a low impulse
long unsigned int lowIn;
// the amount of milliseconds the sensor has to be low
// before we assume all motion has stopped
long unsigned int motionPause = 5000;

int pirPin = 34; // the digital pin 34 connected to the PIR sensor's output
int ledPin = 13;
const int led1 = 33;

// Function signatures
void pirSensor();
void lightIntensity();
void connectWiFi();
String getDateTime();

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
  lightIntensity();
}

// Task2code: PIR sensor
void Task2code(void *pvParameters)
{
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  // Calling pir sensor to check occupancy
  pirSensor();
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

// Function for get PIR sensor data to check the occupancy
void pirSensor()
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

void lightIntensity()
{
  for (;;)
  {
    float lux = lightMeter.readLightLevel();
    // Set new flux
    UoP_CO_326_E18_Gr18_BH1750_data = lux;
    // Set new timestamp
    UoP_CO_326_E18_Gr18_BH1750_timeStamp = getDateTime();

    Serial.print("Light: ");
    Serial.print(lux);
    Serial.print(" lx ");
    Serial.println(UoP_CO_326_E18_Gr18_BH1750_timeStamp);
    delay(2000);
  }
}

String getDateTime()
{
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon + 1;
  int currentYear = ptm->tm_year + 1900;

  String dash = "-";
  dateString = String(monthDay);
  dateString.concat(dash);
  dateString.concat(currentMonth);
  dateString.concat(dash);
  dateString.concat(currentYear);

  timeString = timeClient.getFormattedTime();

  String space = " ";
  dateString.concat(space);
  dateString.concat(timeString);

  return dateString;
}