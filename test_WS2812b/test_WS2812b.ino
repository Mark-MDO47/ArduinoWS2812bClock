
#include <WiFi.h>
#include <time.h>
#include <string.h>
#include <mdo_config.h>

#include "FastLED.h"                         // to manipulate WS2812b (NeoPixel) 5050 RGB LED Rings

#define NUMOF(x) (sizeof((x)) / sizeof((*x)))

#define BRIGHTMAX 40 // set to 250 for final version

// I am using the giant 9-ring 241-LED disk
#define NUM_DISKS 1
#define NUM_SHADOWS 1  // number of shadow disks
#define NUM_LEDS_PER_DISK 241
#define NUM_RINGS_PER_DISK 9
#define MAX_LEDS_PER_RING 60
#define MIN_LEDS_PER_RING 1
#define LCM_LEDS_PER_RING 480 // they all divide into 480 evenly (least common multiple)

// LED count - number of LEDs in each ring in order of serial access
const uint16_t  leds_per_ring[NUM_RINGS_PER_DISK]  = { MAX_LEDS_PER_RING, 48, 40, 32, 24, 16, 12, 8, MIN_LEDS_PER_RING }; // MAX_LEDS_PER_RING = 60
const uint16_t   start_per_ring[NUM_RINGS_PER_DISK] = {  0, 60, 108, 148, 180, 204, 220, 232, 240 }; // which LED index is the start of each ring
const uint16_t  leds_per_ringqrtr[NUM_RINGS_PER_DISK]  = { 15, 12, 10, 8, 6, 4, 3, 2, 1 };
static uint32_t g_radar_xray_bitmask[(NUM_LEDS_PER_DISK+31)/32];  // bitmask where X-Ray LEDs are for STEP2_RADAR_XRAY_SHDW1; >= one bit per LED per ring
static uint32_t bitmsk32; // used to pick out the bit in g_radar_xray_bitmask
static uint16_t  idx_bitmsk32; // index to which array member for g_radar_xray_bitmask

#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB

// Creates an array with the length set by NUM_LEDS_PER_DISK above
// This is the array the library will read to determine how each LED in the strand should be set
static uint32_t data_guard_before = 0x55555555;
static CRGB led_display[(1+NUM_SHADOWS)*NUM_LEDS_PER_DISK]; // 1st set is for display, then shadow1 then shadow2 etc.
static uint32_t data_guard_after = 0x55555555;

// static CRGB led_tmpRing[MAX_LEDS_PER_RING]; // for temp storage
static CRGB led_tmp1; // for temporary storage
static CRGB led_BLACK  = CRGB::Black;
static CRGB led_RED    = CRGB::Red;
static CRGB led_GREEN  = CRGB::Green;
static CRGB led_BLUE   = CRGB::Blue;
static CRGB led_YELLOW = CRGB::Yellow;
static CRGB led_WHITE  = CRGB::White;

static uint8_t gHue = 0; // rotating "base color" used by Demo Reel 100
static uint16_t gSeed = ((uint16_t) 42); // my favorite is 47 but the whole world loves 42 and HHG2TG

#define DPIN_FASTLED 23  // will need to experiment for my board: ESP32 ESP-32S CP2102 NodeMCU-32S ESP-WROOM-32 WiFi Unassembled https://smile.amazon.com/gp/product/B08DQQ8CBP/
                         // GPIO23 is marked on this board as D23. With the USB on the bottom, it is pin number 15 from the bottom on the right hand side (top on RH side).
                         //
                         // NOTE: https://github.com/FastLED/FastLED/issues/923
                         // can ignore following PRAGMA message: 
                         //      /Documents/Arduino/libraries/FastLED/fastspi.h:130:23: note: #pragma message: No hardware SPI pins defined. All SPI access will default to bitbanged output
                         // kriegsman commented on Feb 4  
                         //      The FastLED code for ESP32 uses the ESP32's dedicated "RMT" hardware unit to output the data signal, instead of the "SPI" hardware as is used on (e.g.) AVR microcontrollers.
                         //      But FastLED on ESP32 doesn't actually 'manually bitbang' the data out using just software.
                         //      So I think that since we are already using a hardware-assisted data output channel, we should probably just eliminate this message when that's the case.
                         // bbulkow commented on Sep 15
                         //      The warning actually states that SPI led strings - like the APA102 - will default to bitbanging.
                         //      The RMT hardware can't be used for SPI leds, because it requires two wires of synchronization.

static uint8_t color_idx = 0;
char * colorName[4] = { "Black", "Red", "Green", "Blue" };
CRGB colorLED[4] = { CRGB::Black, CRGB::Red, CRGB::Green, CRGB::Blue };

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup()
//   initializes hardware serial port for general debug
//   initializes FastLED
void setup() {
  // put your setup code here, to run once:
  pinMode(DPIN_FASTLED, OUTPUT);
  Serial.begin(115200);         
  delay(1000);

  for (uint16_t idx = 0; idx < NUM_LEDS_PER_DISK; idx++) {
    led_display[idx] = colorLED[color_idx];
  }
  // initialize the FastLED library for our setup
  // according to Amazon comments: Library configuration used was WS2812B GRB (not RGB). Library call: FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);. Everything worked as expected.
  //    this was for the graduation cap: FastLED.addLeds<NEOPIXEL,DPIN_FASTLED>(led_display, NUM_LEDS_PER_DISK);
  FastLED.addLeds<WS2812B,DPIN_FASTLED,COLOR_ORDER>(led_display, NUM_LEDS_PER_DISK);
  FastLED.setBrightness(BRIGHTMAX); // we will do our own power management

}

void loop() {
  uint16_t idx;
  // put your main code here, to run repeatedly:
  Serial.println(colorName[color_idx]);
  FastLED.show();

  color_idx += 1;
  if (color_idx >= NUMOF(colorLED)) {
    color_idx = 0;
  }
  for (idx = 0; idx < NUM_LEDS_PER_DISK; idx++) {
    led_display[idx] = colorLED[color_idx];
  }
  Serial.println(idx);
  delay(1000);

}
