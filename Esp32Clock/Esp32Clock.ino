// Author: Mark Olson 2020-10-30
//
// Rubber Band Gun - https://github.com/Mark-MDO47/RubberBandGun
// RBG - A high-tech imagining of the rubber band gun
//
// I am using an Arduino Nano with a USB mini-B connector
//   example: http://www.ebay.com/itm/Nano-V3-0-ATmega328P-5V-16M-CH340-Compatible-to-Arduino-Nano-V3-Without-Cable/201804111413?_trksid=p2141725.c100338.m3726&_trkparms=aid%3D222007%26algo%3DSIC.MBE%26ao%3D1%26asc%3D20150313114020%26meid%3Dea29973f227743f78772d7a22512af53%26pid%3D100338%26rk%3D1%26rkt%3D30%26sd%3D191602576205
//            V3.0 ATmega328P 5V 16M CH340 Compatible to Arduino Nano V3
//            32Kbyte Flash (program storage), 2Kbyte SRAM, 1Kbyte EEPROM
//            http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf
//
// I am using this amazing 241 LED WS2812B disk with an insanely low price: https://smile.amazon.com/gp/product/B083VWVP3J/
// Here is a (somewhat) spec on the 2812b LEDs: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
// Here is a Worldsemi WS2812B document for download: http://www.world-semi.com/solution/list-4-1.html#108
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


#include "Arduino.h"
#include <EEPROM.h>                          // to store configuration info
#include "FastLED.h"                         // to manipulate WS2812b (NeoPixel) 5050 RGB LED Rings

#define USE_PROGMEM true                     // set true to keep big const items in FLASH (PROGMEM keyword)


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
