// Author: Mark Olson 2020-11-06
//
// Test Code for the ESP32 WS2812b Clock - https://github.com/Mark-MDO47/ESP32WS2812bClock
// 
// Using this ESP32 board
//   ESP32 ESP-32S CP2102 NodeMCU-32S Unassembled WiFi
//   https://smile.amazon.com/gp/product/B08DQQ8CBP/
//
// This code is derived from readings on the Internet and from my own fevered brain.
//

#include <WiFi.h>
#include <time.h>
#include <string.h>
#include <mdo_config.h>

#define WIFI_NUMTRIES 10  // how many 500 millisec tries to connect to WIFI

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -8*3600; // UTC is 8 hours prior to Los Angeles
const int   daylightOffset_sec = 3600; // DST moves us one hour

static struct tm tm_time;
#define STRF_NUMCHARS 20 // for strftime "%d %H:%M:%S" (actually takes 12 chars)
static char time_connect[STRF_NUMCHARS]; // last time we refreshed NTP time info
static char time_past[STRF_NUMCHARS];    // last time one second changed
static char time_str[STRF_NUMCHARS];     // this time
static int8_t wifi_good = false;         // true if connected to WiFi


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup()
//   initializes hardware serial port for general debug
//   connects to WiFi in STA mode (normal, connect to router)
//   requests time from NTP server and sets local clock
//   disconnects from WiFi
//
void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("Initializing test_ntp\n");

  for (int idx = 1; idx <= 10; idx++) { // make sure it works more than once
    wifi_good = connectGetNtpInfoAndDisconnect(true, true);
    if (false == wifi_good) {
      Serial.println("\n\nERROR - Could not get NTP Time!");
    }
  } // end try out connectGetNtpInfoAndDisconnect

  Serial.println(" ");
} // end setup()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//  loop()
//    if time in seconds has changed, show current time
//    each hour on the hour resync with NTP and 
void loop() { 
  if (getLocalTime(&tm_time)) {
    strftime(time_str, STRF_NUMCHARS, "%d %H:%M:%S", &tm_time);
    if (0 != strncmp(time_past, time_str, STRF_NUMCHARS)) {
      Serial.println(time_str);
      strncpy(time_past, time_str, STRF_NUMCHARS);
      if (NULL != strstr(time_str, ":00")) { // DEBUG: check every minute
      // if (NULL != strstr(time_str, ":00:")) { // sigh... no strnstr
        wifi_good = connectGetNtpInfoAndDisconnect(true, true);
      } // end if refresh NTP info once per hour
    } // end if the seconds field changed
  }  
  delay(50);
} // end loop()



/////////////////////////////////////////////////////////////////////////////////////////////////////////
// connectGetNtpInfoAndDisconnect() - connect to WiFi (if needed), get NTP info, disconnect (if requested)
//    if disconnectWifi then disconnect when done
//    if doPrint then print status as we go
//
int connectGetNtpInfoAndDisconnect(int disconnectWifi, int doPrint) {
  int success = false;
  int tries = 0;

  if (WiFi.status() != WL_CONNECTED) {
    // Connect to the network
    if (false != doPrint) {
      Serial.print("Connect to ");
      Serial.println(SSID);
    } // end if doPrint
    WiFi.mode(WIFI_STA); // Station mode (aka STA mode or WiFi client mode). ESP32 connects to an access point
    WiFi.begin(SSID, WiFiPassword);
    while ((tries < WIFI_NUMTRIES) && (WiFi.status() != WL_CONNECTED)) {
      // Wait for the Wi-Fi to connect
      delay(500);
      if (false != doPrint) {
        Serial.print('.');
      } // end if doPrint
      tries += 1;
    } // end while waiting for WL_CONNECTED
  } // end if need to connect to network

  if (WiFi.status() == WL_CONNECTED) {
    // connected to WiFi network
    if (0 != doPrint) {
      Serial.println("\n");
      Serial.println("Connection established");  
      Serial.print("IP address:\t");
      Serial.println(WiFi.localIP());
    } // end if doPrint

    // Get the NTP time
    // US  +340308−1181434 America/Los_Angeles Pacific Canonical −08:00  −07:00
    // setenv("TZ", "PST8PDT,M3.2.0/02:00:00,M11.1.0/02:00:00", 1); // see https://users.pja.edu.pl/~jms/qnx/help/watcom/clibref/global_data.html
    //       M3.2.0/02:00:00 - enter DST on second (2) Sunday (0) of March (3) at 2 am (02:00:00)
    //       M11.1.0/02:00:00 - exit DST on first (1) Sunday (0) of November (11) at 2 am (02:00:00)
    //
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    if (getLocalTime(&tm_time)) {
      strftime(time_connect, STRF_NUMCHARS, "%d %H:%M:%S", &tm_time);
      success = true;
    } else {
      if (false != doPrint) {
        Serial.print("\n\nERROR - strftime() failed\n\n");
      } // end if doPrint
      success = false;
    }
  } // end if connected to WiFi

  if ((true == disconnectWifi) && (WiFi.status() == WL_CONNECTED)) {
      //disconnect WiFi
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      if (0 != doPrint) {
        Serial.println("\nWifi Disconnected\n");
      } // end if doPrint
  } // end if need to disconnect WiFi

  return(success);
} // end connectGetNtpInfoAndDisconnect()
