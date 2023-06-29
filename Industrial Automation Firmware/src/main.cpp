#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <BH1750.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define WIFI_SSID "Redmi Note 7"
#define WIFI_PASSWORD "Anush123ga"

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800);

// Create 2 tasks for each processor
TaskHandle_t Task1;
TaskHandle_t Task2;

// Initiate BH1750 sensor
BH1750 lightMeter;

// Variables to store values that need to send to MQTT
String intensityTimeStamp = "";
float intensityValue;
String pirTimeStamp = "";
boolean pirValue = false; // true: movement detected, false: no movement
String dateString = "";
String timeString = "";
bool occupancyValue = true;
bool lastLedState = true;
bool securityMode = false;

// the time when the sensor outputs a low impulse
long unsigned int lowIn;
// the amount of milliseconds the sensor has to be low
// before we assume all motion has stopped
long unsigned int motionPause = 5000;
boolean lockLow = true;
boolean takeLowTime;
int calibrationTime = 15;
char lightDataStr[200];
char PIRDataStr[100];
char levelDataStr[50];

int pirPin = 34; // the digital pin 34 connected to the PIR sensor's output
int ledPin = 13;

// Bulb pins
int bulbPin1 = 16;
int bulbPin2 = 17;
int bulbPin3 = 18;
int bulbPin4 = 19;
// Red bulb pins
int redLedPin1 = 4;
int redLedPin2 = 2;

// LED brightness range
const int minBrightness = 0;   // Minimum LED brightness (0-255)
const int maxBrightness = 255; // Maximum LED brightness (0-255)

// MQTT broker details
const char *mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char *mqttTopicIntensity = "UoP/CO/326/E18/18/BH1750";
const char *mqttTopicOccupancy = "UoP/CO/326/E18/18/PIR";
const char *mqttTopicLightControl = "UoP/CO/326/E18/18/LED";
const char *mqttTopicSecurityMode = "UoP/CO/326/E18/18/Security";
const char *mqttTopicLightLevel = "UoP/CO/326/E18/18/LightLevel";

// WiFi and MQTT client instances
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Function signatures
void connectWiFi();
void reconnect();
void pirSensor();
void lightIntensity();
String getDateTimePIR();
String getDateTimeBH1750();
void callback(char *, byte *, unsigned int);
void handlePIRMessage(String);
void handleBH1750Message(String);
void setLEDState(bool);
void handleSecurityMode(String);

void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

void setup()
{
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(bulbPin1, OUTPUT);
  pinMode(bulbPin2, OUTPUT);
  pinMode(bulbPin3, OUTPUT);
  pinMode(bulbPin4, OUTPUT);
  pinMode(redLedPin1, OUTPUT);
  pinMode(redLedPin2, OUTPUT);
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

  // Set MQTT server and callback function
  mqttClient.setServer(mqttServer, mqttPort);

  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));

  mqttClient.setCallback(callback);
  reconnect();

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
  if (!mqttClient.connected())
  {
    reconnect();
  }
  mqttClient.loop();
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
        pirValue = true;
        pirTimeStamp = getDateTimePIR();
        lockLow = false;

        // Convert sensor value and timestamp to a string
        snprintf(PIRDataStr, 100, "{\"DateTime\": \"%s\", \"Occupancy\": %s}", pirTimeStamp.c_str(), pirValue ? "true" : "false");

        if (!mqttClient.connected())
        {
          reconnect();
        }
        // Publish sensor data to MQTT topic
        mqttClient.publish(mqttTopicOccupancy, PIRDataStr);

        // Serial.println(PIRDataStr);
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
        pirValue = false;
        pirTimeStamp = getDateTimePIR();

        // Convert sensor value and timestamp to a string
        snprintf(PIRDataStr, 100, "{\"DateTime\": \"%s\", \"Occupancy\": %s}", pirTimeStamp.c_str(), pirValue ? "true" : "false");
        if (!mqttClient.connected())
        {
          reconnect();
        }
        // Publish sensor data to MQTT topic
        mqttClient.publish(mqttTopicOccupancy, PIRDataStr);

        // Serial.println(PIRDataStr);
        delay(50);
      }
    }
  }
}

void lightIntensity()
{
  for (;;)
  {
    // Set new flux
    float intensityValue = lightMeter.readLightLevel();
    if (intensityValue > 1000.0)
    {
      intensityValue = 1000.0;
    }
    int roundedIntensity = (int)(intensityValue + 0.5);
    // Set new timestamp
    intensityTimeStamp = getDateTimeBH1750();

    // Convert sensor value and timestamp to a string
    snprintf(lightDataStr, 200, "{\"DateTime\": \"%s\", \"Intensity\": %d}", intensityTimeStamp.c_str(), roundedIntensity);

    // Serial.println(lightDataStr);

    // Check if connected to MQTT broker
    if (!mqttClient.connected())
    {
      reconnect();
    }
    // Publish sensor data to MQTT topic
    mqttClient.publish(mqttTopicIntensity, lightDataStr);
    delay(2000);
  }
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

void reconnect()
{
  // Loop until connected to MQTT broker
  while (!mqttClient.connected())
  {
    Serial.println("Connecting to MQTT broker...");

    // Connect to MQTT broker with client ID
    if (mqttClient.connect("ArduinoClient"))
    {
      Serial.println("Connected to MQTT broker");
      mqttClient.subscribe(mqttTopicIntensity);
      mqttClient.subscribe(mqttTopicOccupancy);
      mqttClient.subscribe(mqttTopicSecurityMode);
    }
    else
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");

      // Wait before retrying
      delay(5000);
    }
  }
}

String getDateTimePIR()
{
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon + 1;
  int currentYear = ptm->tm_year + 1900;

  unsigned int millisecond = epochTime % 1000;

  String dash = "-";
  dateString = String(currentYear);
  dateString.concat(dash);
  dateString.concat(currentMonth);
  dateString.concat(dash);
  dateString.concat(monthDay);

  timeString = timeClient.getFormattedTime();

  String space = " ";
  dateString.concat(space);
  dateString.concat(timeString);
  dateString.concat(":");
  dateString.concat(millisecond);

  return dateString;
}

String getDateTimeBH1750()
{
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon + 1;
  int currentYear = ptm->tm_year + 1900;

  unsigned int millisecond = epochTime % 1000;

  String dash = "-";
  dateString = String(currentYear);
  dateString.concat(dash);
  dateString.concat(currentMonth);
  dateString.concat(dash);
  dateString.concat(monthDay);

  timeString = timeClient.getFormattedTime();

  String space = " ";
  dateString.concat(space);
  dateString.concat(timeString);
  dateString.concat(":");
  dateString.concat(millisecond);

  return dateString;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  // Convert payload to a string
  String message = "";
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  // Parse JSON message
  if (String(topic) == mqttTopicIntensity)
  {
    // BH1750 topic
    handleBH1750Message(message);
  }
  else if (String(topic) == mqttTopicOccupancy)
  {
    // PIR topic
    handlePIRMessage(message);
  }
  else if (String(topic) == mqttTopicLightControl)
  {
    if (message == "on")
    {
      setLEDState(true);
    }
    else if (message == "off")
    {
      setLEDState(false);
    }
  }
  else if (String(topic) == mqttTopicSecurityMode)
  {
    Serial.println("Security changed");
    handleSecurityMode(message);
  }
}

void handleBH1750Message(String message)
{
  // Parse JSON data
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error)
  {
    Serial.print("JSON parsing error: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract light intensity value from JSON
  int intensityValue = doc["Intensity"];

  int brightness = 0;

  if (lastLedState && !securityMode)
  {
    if (intensityValue == 0)
    {
      // brightness = 4;
      digitalWrite(bulbPin1, HIGH);
      digitalWrite(bulbPin2, HIGH);
      digitalWrite(bulbPin3, HIGH);
      digitalWrite(bulbPin4, HIGH);
      digitalWrite(redLedPin1, LOW);
      digitalWrite(redLedPin2, LOW);

      snprintf(lightDataStr, 50, "{\"Level\": %d}", 4);
      if (!mqttClient.connected())
        reconnect();
      mqttClient.publish(mqttTopicLightLevel, lightDataStr);
    }
    else if (intensityValue > 0 && intensityValue < 250)
    {
      // brightness = 3;
      digitalWrite(bulbPin1, HIGH);
      digitalWrite(bulbPin2, HIGH);
      digitalWrite(bulbPin3, HIGH);
      digitalWrite(bulbPin4, LOW);
      digitalWrite(redLedPin1, LOW);
      digitalWrite(redLedPin2, LOW);

      snprintf(levelDataStr, 50, "{\"Level\": %d}", 3);
      if (!mqttClient.connected())
        reconnect();
      mqttClient.publish(mqttTopicLightLevel, levelDataStr);
    }
    else if (intensityValue > 249 && intensityValue < 500)
    {
      // brightness = 2;
      digitalWrite(bulbPin1, HIGH);
      digitalWrite(bulbPin2, HIGH);
      digitalWrite(bulbPin3, LOW);
      digitalWrite(bulbPin4, LOW);
      digitalWrite(redLedPin1, LOW);
      digitalWrite(redLedPin2, LOW);

      snprintf(levelDataStr, 50, "{\"Level\": %d}", 2);
      if (!mqttClient.connected())
        reconnect();
      mqttClient.publish(mqttTopicLightLevel, levelDataStr);
    }
    else if (intensityValue > 499 && intensityValue < 750)
    {
      // brightness = 1;
      digitalWrite(bulbPin1, HIGH);
      digitalWrite(bulbPin2, LOW);
      digitalWrite(bulbPin3, LOW);
      digitalWrite(bulbPin4, LOW);
      digitalWrite(redLedPin1, LOW);
      digitalWrite(redLedPin2, LOW);

      snprintf(levelDataStr, 50, "{\"Level\": %d}", 1);
      if (!mqttClient.connected())
        reconnect();
      mqttClient.publish(mqttTopicLightLevel, levelDataStr);
    }
    else if (intensityValue > 749)
    {
      // brightness = 0;
      digitalWrite(bulbPin1, LOW);
      digitalWrite(bulbPin2, LOW);
      digitalWrite(bulbPin3, LOW);
      digitalWrite(bulbPin4, LOW);
      digitalWrite(redLedPin1, LOW);
      digitalWrite(redLedPin2, LOW);

      snprintf(levelDataStr, 50, "{\"Level\": %d}", 0);
      if (!mqttClient.connected())
        reconnect();
      mqttClient.publish(mqttTopicLightLevel, levelDataStr);
    }
  }
  else
  {
    // brightness = 0;
    digitalWrite(bulbPin1, LOW);
    digitalWrite(bulbPin2, LOW);
    digitalWrite(bulbPin3, LOW);
    digitalWrite(bulbPin4, LOW);
    Serial.println("LED forced off.");

    snprintf(levelDataStr, 50, "{\"Level\": %d}", 0);
    if (!mqttClient.connected())
      reconnect();
    mqttClient.publish(mqttTopicLightLevel, levelDataStr);
  }
}

void handlePIRMessage(String message)
{
  // Parse JSON data
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error)
  {
    Serial.print("JSON parsing error: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract occupancy value from JSON
  occupancyValue = doc["Occupancy"];

  // Check if occupancy is true and light intensity is below threshold
  if (occupancyValue && lightMeter.readLightLevel() < 500)
  {
    // digitalWrite(bulbPin, HIGH); // Turn on LED
    Serial.println("Turn on LED");
  }
  else if (!occupancyValue)
  {
    // digitalWrite(bulbPin, LOW); // Turn off LED
    Serial.println("Turn off LED");
    digitalWrite(bulbPin1, LOW);
    digitalWrite(bulbPin2, LOW);
    digitalWrite(bulbPin3, LOW);
    digitalWrite(bulbPin4, LOW);
    digitalWrite(redLedPin1, LOW);
    digitalWrite(redLedPin2, LOW);

    snprintf(levelDataStr, 50, "{\"Level\": %d}", 0);
    if (!mqttClient.connected())
      reconnect();
    mqttClient.publish(mqttTopicLightLevel, levelDataStr);
  }
}

void handleSecurityMode(String message)
{
  // Parse JSON data
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error)
  {
    Serial.print("JSON parsing error: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract light intensity value from JSON
  bool securityModeValue = doc["Security"];

  Serial.print("Mode: ");
  Serial.println(securityModeValue);

  if (securityModeValue == true)
  {
    securityMode = true;
    Serial.println("Security mode on.");
    digitalWrite(bulbPin1, LOW);
    digitalWrite(bulbPin2, LOW);
    digitalWrite(bulbPin3, LOW);
    digitalWrite(bulbPin4, LOW);
    digitalWrite(redLedPin1, LOW);
    digitalWrite(redLedPin2, LOW);

    snprintf(levelDataStr, 50, "{\"Level\": %d}", 0);
    if (!mqttClient.connected())
      reconnect();
    mqttClient.publish(mqttTopicLightLevel, levelDataStr);

    if (occupancyValue)
    {
      digitalWrite(redLedPin1, HIGH);
      digitalWrite(redLedPin2, HIGH);
    }
  }
  else if (securityModeValue == false)
  {
    securityMode = false;
    Serial.println("Security mode off.");
  }
  else
  {
    Serial.println("Security mode off.");
    securityMode = false;
  }
}

// Control the light
void setLEDState(bool state)
{
  if (state && !lastLedState)
  {
    digitalWrite(ledPin, HIGH);
    lastLedState = true;
  }
  else if (!state && lastLedState)
  {
    digitalWrite(ledPin, LOW);
    lastLedState = false;
  }
}
