// Author: Mark Olson 2020-10-30
//
// ESP32 WS2812b Clock - https://github.com/Mark-MDO47/ESP32WS2812bClock
// 
// Planning to use this ESP32 board
//   ESP32 ESP-32S CP2102 NodeMCU-32S Unassembled WiFi
//   https://smile.amazon.com/gp/product/B08DQQ8CBP/
//
// I am using this amazing 241 LED WS2812B disk with an insanely low price: https://smile.amazon.com/gp/product/B083VWVP3J/
//   Here is a (somewhat) spec on the 2812b LEDs: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
//   Here is a Worldsemi WS2812B document for download: http://www.world-semi.com/solution/list-4-1.html#108
//
// Major kudos to Daniel Garcia and Mark Kriegsman for the FANTASTIC FastLED library and examples!!!
//    A sad note that Daniel Garcia, co-author of FastLED library, was on the dive boat that caught fire some time ago and has passed. 
//    Here is some info on the FastLED Reddit https://www.reddit.com/r/FastLED/
//
// As of now, I am using just my hand-coded LED patterns.
//    History says I will eventually use a few from Mark Kriegsman's classic DemoReel100.ino
//    https://github.com/FastLED/FastLED/tree/master/examples/DemoReel100
//
//
// Note: I am not making any claims that this software is well written or a good example for anyone.
//       I am just having fun with it.
//       Mark


#include <WiFi.h>
#include <time.h>
#include "FastLED.h"                         // to manipulate WS2812b (NeoPixel) 5050 RGB LED Rings

#define USE_PROGMEM true                     // set true to keep big const items in FLASH (PROGMEM keyword)


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);         
  delay(1000);
  Serial.println("Here I am...");

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10*1000);
  Serial.println("Here I am again...");
}
