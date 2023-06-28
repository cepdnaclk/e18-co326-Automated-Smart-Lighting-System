#include <Arduino.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFi.h>

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800);

String dateString = "";
String timeString = "";

String getDateTime()
{
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon + 1;
  int currentYear = ptm->tm_year + 1900;

  unsigned int millisecond = epochTime % 1000;

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
  dateString.concat(":");
  dateString.concat(millisecond);

  return dateString;
}
