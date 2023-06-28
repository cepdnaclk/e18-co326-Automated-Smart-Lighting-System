#include <Arduino.h>
#include <BH1750.h>

// Variables to store values that need to send to MQTT
String UoP_CO_326_E18_Gr18_BH1750_timeStamp = "";
float UoP_CO_326_E18_Gr18_BH1750_data;

extern String getDateTime();

void lightIntensity(BH1750 lightMeter)
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