// Author: Mark Olson 2020-10-30
//
// ESP32 WS2812b Clock - https://github.com/Mark-MDO47/ESP32WS2812bClock
// 
// Using this ESP32 board
//   ESP32 ESP-32S CP2102 NodeMCU-32S Unassembled WiFi
//   https://smile.amazon.com/gp/product/B08DQQ8CBP/
//
// I am using the amazing 241 LED WS2812B disk with an insanely low price: https://smile.amazon.com/gp/product/B083VWVP3J/
//   Here is a (somewhat) spec on the 2812b LEDs: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
//   Here is a Worldsemi WS2812B document for download: http://www.world-semi.com/solution/list-4-1.html#108
//
// Major kudos to Daniel Garcia and Mark Kriegsman for the FANTASTIC FastLED library and examples!!!
// Kudos to Daniel Garcia and Mark Kriegsman for the FANTASTIC FastLED library and examples!!! And it works for ESP32 also!
//    https://github.com/FastLED/FastLED
// A sad note that Daniel Garcia, co-author of FastLED library, was on the dive boat that caught fire some time ago and has passed. 
//    Here is some info on the FastLED Reddit https://www.reddit.com/r/FastLED/
//
// As of now, I am using just my hand-coded LED patterns.
//    History says I will eventually use a few from Mark Kriegsman's classic DemoReel100.ino for Fire2012WithPallette.ino
//    https://github.com/FastLED/FastLED/tree/master/examples/DemoReel100
//    https://github.com/FastLED/FastLED/blob/master/examples/Fire2012WithPalette/Fire2012WithPalette.ino
//
// connections:
//    Data Pin DPIN_FASTLED is used for serial communications with the LEDs for the three rings of LEDs using FastLED
//
// Note: I am not making any claims that this software is well written or a good example for anyone.
//       I am just having fun with it.
//       Mark


#include <WiFi.h>
#include <time.h>
#include "FastLED.h"                         // to manipulate WS2812b (NeoPixel) 5050 RGB LED Rings

#define NUMOF(x) (sizeof((x)) / sizeof((*x)))

// define the symbols - general use symbols:
#define mUNDEFINED 254
#define mNONE 255
#define mZERO 0

#define mEFCT_PTRNLED_OFF 254
#define DLYLED_MIN 7
#define DLYLED_ringRotateAndFade 7 // for RBG_ringRotateAndFade
#define DLYLED_diskDownTheDrain 7 // for RBG_DiskDownTheDrain

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
} // returns -1, 0, 1 for (T < 0), (T == 0), (T > 0) respectively. returns warning if used on uintx_t

#define USE_PROGMEM true                     // set true to keep big const items in FLASH (PROGMEM keyword)

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
const uint8_t   start_per_ring[NUM_RINGS_PER_DISK] = {  0, 60, 108, 148, 180, 204, 220, 232, 240 }; // which LED index is the start of each ring
const uint16_t  leds_per_ringqrtr[NUM_RINGS_PER_DISK]  = { 15, 12, 10, 8, 6, 4, 3, 2, 1 }; // note: not too sure if last should be 0 or 1
static int8_t   this_ring = 0; // from ring_9 (value 0, outer ring) to ring_1 (value 8, inner ring (one LED))
static int8_t   this_qrtr = 0; // from qrtr_1 (value 0) to qrtr_4 (value 3), count modulo in either direction
static uint32_t radar_xray_bitmask[(NUM_LEDS_PER_DISK+31)/32];  // bitmask where X-Ray LEDs are for STEP2_RADAR_XRAY_SHDW1; >= one bit per LED per ring
static uint32_t bitmsk32; // used to pick out the bit in radar_xray_bitmask
static uint8_t  idx_bitmsk32; // index to which array member for radar_xray_bitmask

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

typedef struct _brightSpots_t {
  uint8_t posn; // position relative to start of ring; + = counterclockwise. 255 or mNONE terminates list
  CRGB hue;     // color for that position
} brightSpots_t; // end definition

// for RBG_rotateRingAndFade(), only need to do ring 0 and others follow suit
static brightSpots_t windup1BrightSpots[] = {
  { .posn=0, .hue=CRGB::Red },
  { .posn=6, .hue=CRGB::Green },
  { .posn=12, .hue=CRGB::Blue },
  { .posn=18, .hue=CRGB::Yellow },
  { .posn=24, .hue=CRGB::Red },
  { .posn=30, .hue=CRGB::Green },
  { .posn=36, .hue=CRGB::Blue },
  { .posn=42, .hue=CRGB::Yellow },
  { .posn=48, .hue=CRGB::Red },
  { .posn=54, .hue=CRGB::Green },
  { .posn=mNONE, .hue=CRGB::Black },
}; // end windup1BrightSpots[] definition

static uint8_t gHue = 0; // rotating "base color" used by Demo Reel 100
static uint16_t gSeed = ((uint16_t) 42); // my favorite is 47 but the whole world loves 42 and HHG2TG

// probably get rid of this stuff later but first let's compile
#define mEFCT_UNIQ         60  // 061 to 127 - unique effects used to navigate menus or other activities
#define TYPEOF_lookupLEDpatternTbl uint8_t // so we can match syses
#define mEFCT_PTRNLED_OFF 254
static const TYPEOF_lookupLEDpatternTbl lookupLEDpatternTbl[]
#if USE_PROGMEM
  PROGMEM
#endif // end USE_PROGMEM
  = { 
        7,  5,  2,  1,  6, 11,  12, 254, 254, 254,  // 000 to 009 - wind-up effects
        8,  9, 10, 11,  1,  7,   5,  12, 254, 254,  // 010 to 019 - shoot effects
        4, 11,  2,  7,  5, 13, 254, 254, 254, 254,  // 020 to 029 - open barrel effects
        4, 11,  2,  7,  5, 13, 254, 254, 254, 254,  // 030 to 039 - lock and load barrel effects
        5,  6,  3,  1, 11,  2,   7, 254, 254, 254,  // 040 to 049 - initial power-up effects
        6,  3,  5,  1, 11,  2,   7, 254, 254, 254   // 050 to 059 - waiting for trigger
  };

static struct _myState_t {
  uint32_t timerNow = 0;            // timer now
  uint32_t timerPrevLEDstep = 0;    // start timer from previous LED activity
  uint16_t ptrnDelayLEDstep = 7;    // proper delta delay for Mark's patterns
  uint16_t efctLED = 3;             // which effect to perform

} myState;

#define DEBUGALL_GLOBAL 0                    // sets ALL debug flags at once
#define DEBUG_SHOW_MSEC 1                    // use globalLoopCount for millis() display not loopcount
static uint32_t globalLoopCount = 0;  // based on DEBUG_SHOW_MSEC: this is either the milliseconds since startup or a count of times through loop()
#define DPIN_FASTLED 13  // will need to experiment for my board: ESP32 ESP-32S CP2102 NodeMCU-32S ESP-WROOM-32 WiFi Unassembled https://smile.amazon.com/gp/product/B08DQQ8CBP/


// radar_ring_idx describes the index within each ring of a moving second-hand or a leading edge of a radar sweep as it moves in a clockwise TBR direction
// for ring 0 which has 60 LEDs, it is just a count of 0 through 59
// others are computed by calculating MOD(ROUND(ring0num[idx]*NUM_LEDS_THIS_RING/MAX_LEDS_PER_RING,0),NUM_LEDS_THIS_RING)
// we do NOT add the starting LED per ring from start_per_ring[]. The mod calculations for each ring as we go left and right are easier with the data in this form, without that addition
static uint8_t const radar_ring_idx[NUM_RINGS_PER_DISK][MAX_LEDS_PER_RING] = {
 { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 },
 { 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 13, 14, 14, 15, 16, 17, 18, 18, 19, 20, 21, 22, 22, 23, 24, 25, 26, 26, 27, 28, 29, 30, 30, 31, 32, 33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 42, 42, 43, 44, 45, 46, 46, 47 },
 { 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 13, 13, 14, 15, 15, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 37, 38, 39, 39 },
 { 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31 },
 { 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 16, 17, 17, 18, 18, 18, 19, 19, 20, 20, 20, 21, 21, 22, 22, 22, 23, 23, 0 },
 { 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 0 },
 { 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 0, 0 },
 { 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 0, 0, 0 },
 { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
}; // end radar_ring_idx[][]

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);         
  delay(10);

  for (int i = 0; i < NUMOF(radar_xray_bitmask); i++) { radar_xray_bitmask[i] = 0; }

  // initialize the FastLED library for our setup
  // according to Amazon comments: Library configuration used was WS2812B GRB (not RGB). Library call: FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);. Everything worked as expected.
  //    this was for the graduation cap: FastLED.addLeds<NEOPIXEL,DPIN_FASTLED>(led_display, NUM_LEDS_PER_DISK);
  FastLED.addLeds<WS2812B,DPIN_FASTLED,GRB>(led_display, NUM_LEDS_PER_DISK);
  FastLED.setBrightness(BRIGHTMAX); // we will do our own power management
  // initialize led_display
  RBG_diskInitBrightSpots(windup1BrightSpots, &led_BLACK, 3, 196); // FIXME need initialize for this pattern

  Serial.println("ESP32Clock initialized...");
} // end setup()

void loop() {
  // put your main code here, to run repeatedly:

  myState.timerNow = millis();

  if ((myState.timerNow-myState.timerPrevLEDstep) >= myState.ptrnDelayLEDstep) {
    gHue += 3; // rotating "base color" used by Demo Reel 100 patterns
    checkDataGuard();
    doPattern(myState.efctLED, 0); // start
    checkDataGuard();
    FastLED.show();
    myState.timerPrevLEDstep = myState.timerNow;
    globalLoopCount += 1;
  } // end wait for next LED activity
} // end loop()

// ******************************** LED UTILITIES ****************************************

// doPattern(nowEfctLED, tmpInit) - start or step the pattern
//
// If tmpInit is nonzero, do initialization of pattern
//
#define DEBUG_DPtrn 0
//
void doPattern(uint16_t nowEfctLED, uint8_t tmpInit) {
//  static uint16_t prevEfctLED = mNONE;
  static uint16_t numSteps = 0;

  switch (nowEfctLED) {

    case mEFCT_PTRNLED_OFF: // 254 = OFF
    default:
      if (0 != tmpInit) { // initialize
        for (uint8_t idx = 0; idx < NUM_RINGS_PER_DISK; idx++) {
          led_display[idx] = CRGB::Black;
        }
      } // there is no "step"; just leave the LEDs off
      break;

    case 3:
      RBG_bpm_rings();
      break;

    case 4: // RBG_diskDownTheDrainOrRotate(-1) disk, clockwise, rotate through
      if (0 != tmpInit) { // initialize
        RBG_ringRotateAndFade(mNONE, 0, windup1BrightSpots); // FIXME - initialization from other effect
        RBG_diskDownTheDrainOrRotate(0);
      } else { // step
        RBG_diskDownTheDrainOrRotate(-1);
      }
      break;

    case 5: // RBG_diskDownTheDrainOrRotate(2) disk, counterclockwise, drain, repeat
      if ((0 != tmpInit) || (numSteps > (10+NUM_LEDS_PER_DISK))) { // initialize
        RBG_ringRotateAndFade(mNONE, 0, windup1BrightSpots); // FIXME - initialization from other effect
        numSteps = 0;
      } else { // step
        RBG_diskDownTheDrainOrRotate(2);
        numSteps += 1;
      }
      break;

    case 6: // RBG_juggle_numdot_ring(-4) rings, 4 dots
      RBG_juggle_numdot_ring(-4);
      break;

    case 7: // RBG_juggle_numdot_ring(5); RBG_confetti_fadeby(128); disk, 5 dots, sparkle confetti
      RBG_juggle_numdot_ring(5);
      RBG_confetti_fadeby(128);
      break;

    case 8:
      RBG_RailGunEffect(tmpInit, &led_BLUE);
      break;

    case 9:
      RBG_RailGunEffect(tmpInit, &led_RED);
      break;

    case 10:
      RBG_RailGunEffect(tmpInit, &led_GREEN);
      break;

    case 11:
      RBG_confetti_fadeby(128);
      break;

  } // end switch
} // end doPattern()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// lookupLEDpattern(nowEfctLED)
//
// nowEfctLED - "configured" LED pattern number. This already has EEPROM or CFG applied. Is not divisible by 10.
//
// The table used for this purpose is in PROGMEM
//
// returns: actual LED pattern number
//
uint16_t lookupLEDpattern(uint16_t nowEfctLED) {
  TYPEOF_lookupLEDpatternTbl myRetVal;

  nowEfctLED &= 0xFF; // must be 8 bits for actual pattern
  if (nowEfctLED == mEFCT_PTRNLED_OFF) {
    myRetVal = mEFCT_PTRNLED_OFF;
  } else if ((nowEfctLED >= mEFCT_UNIQ) || (0 == nowEfctLED)) {
    if (mNONE != nowEfctLED) {
      Serial.print(F("ERROR - lookupLEDpattern() out of range - ")); Serial.println(nowEfctLED);
    }
    myRetVal = mEFCT_PTRNLED_OFF;
  } else {
#if USE_PROGMEM
    if (sizeof(myRetVal) - sizeof(lookupLEDpatternTbl[0])) { // cannot use sizeof in #if statement
      Serial.print(F("ERROR FIXME SIZE MISMATCH TYPEOF_lookupLEDpatternTbl ")); Serial.print(sizeof(myRetVal)); Serial.println(sizeof(lookupLEDpatternTbl[0]));
      myRetVal = mEFCT_PTRNLED_OFF;
    } else {
      memcpy_P(&myRetVal, &lookupLEDpatternTbl[nowEfctLED-1], sizeof(myRetVal));
    }
#else // not USE_PROGMEM
    myRetVal = lookupLEDpatternTbl[nowEfctLED-1];
#endif // USE_PROGMEM
  }
  return(myRetVal);
} // end lookupLEDpattern(nowEfctLED)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_diskDownTheDrainOrRotate(direction) - 
//
// direction - 0="initialize", +/- 1 = rotate through, +/- 2 = down drain. With these rings, + is counter-clockwise and - is clockwise
// 
// initialize does not overwrite current colors in disk
//
// #define DEBUG_DDtD 1
void RBG_diskDownTheDrainOrRotate(int8_t direction) {
  int8_t idx;

  #ifdef DEBUG_DDtD
  Serial.print(F(" DEBUG_DDtD direction=")); Serial.println(direction);
  #endif // DEBUG_DDtD

  if (direction > (NUM_RINGS_PER_DISK-1)) {
    // initialize
    myState.ptrnDelayLEDstep = DLYLED_diskDownTheDrain;
  } else {
    // do pattern
    RBG_diskRotateOrDrain(direction, &led_BLACK);
  } // end do pattern

} // end RBG_diskDownTheDrainOrRotate()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_ringRotateAndFade(whichRing, rotateLcm, brightSpots) - rotate a single ring, fade to black, add in new bright spots
//
// whichRing - which ring to rotate, 0 through (NUM_RINGS_PER_DISK-1) {2 for RBG}
//                value = mNONE (255) means initialize entire disk (actually, anything > 2)
// rotateLcm - fraction of LCM_LEDS_PER_RING to rotate, +/-. With these rings, + is counter-clockwise and - is clockwise
// brightSpots - list of positions within ring and color for bright spots
// 
// writes over led_tmpRing and led_display
//
// #define DEBUG_RRandF 1
//
// BUG: There is some sort of Arduino compiler bug when using brightSpots_t* as an input param
//
void RBG_ringRotateAndFade(uint8_t whichRing, int8_t rotateLcm, void *tmpPtr) {
  brightSpots_t* brightSpots = (brightSpots_t*) tmpPtr;
  int8_t idx;
  static int16_t startLocPerRing[NUM_RINGS_PER_DISK]; // (95 + 95 = 190) > 127 so int8_t will not work

  if (whichRing > (NUM_RINGS_PER_DISK-1)) {
    // initialize
    #ifdef DEBUG_RRandF
    Serial.print(F(" DEBUG_RRandF initialize whichRing=")); Serial.println(whichRing);
    #endif // DEBUG_RRandF
    myState.ptrnDelayLEDstep = DLYLED_ringRotateAndFade;
    RBG_diskInitBrightSpots(brightSpots, &led_BLACK, -3, 196);
    for (idx=0; idx < NUM_RINGS_PER_DISK; idx++) {
      startLocPerRing[idx] = 0;
    }
    // end initialize
  } else if ((whichRing >= 0) && (whichRing <= (NUM_RINGS_PER_DISK-1)) && (0 != rotateLcm)) {
    // process individual ring
    int8_t numLcmPerLED = LCM_LEDS_PER_RING / leds_per_ring[whichRing];
    int16_t numHere = startLocPerRing[whichRing] / numLcmPerLED;
    int16_t numEnd = (startLocPerRing[whichRing] + rotateLcm) / numLcmPerLED;
    int8_t moveBy;
    
    if (rotateLcm < 0) {
      moveBy = -1;
    } else {
      moveBy = 1;
    }
    startLocPerRing[whichRing] = (startLocPerRing[whichRing] + rotateLcm) % LCM_LEDS_PER_RING;
    #ifdef DEBUG_RRandF
    Serial.print(F(" DEBUG_RRandF ERROR CALL whichRing=")); Serial.println(whichRing);
    #endif // DEBUG_RRandF
    // end process individual ring
  } // end check on whichRing
} // end RBG_ringRotateAndFade()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_ringIdxIncrDecr(idx, idxRing, direction)
//
// idx - index of LED within ring. must be (start_per_ring[idxRing]) <= idx < (start_per_ring[idxRing]+leds_per_ring[idxRing])
// idxRing - which ring we are on, 0 <= idxRing < NUM_RINGS_PER_DISK
// direction - either -1, 0, or 1
// returns value modulus value of idx+direction where (start_per_ring[idxRing]) <= return < (start_per_ring[idxRing]+leds_per_ring[idxRing])
//
uint8_t RBG_ringIdxIncrDecr(uint8_t idx, uint8_t idxRing, int8_t direction) {
  int16_t rtn = idx + direction;

  if (rtn >= start_per_ring[idxRing]+leds_per_ring[idxRing]) {
    rtn = start_per_ring[idxRing];
  } else if (rtn < start_per_ring[idxRing]) {
    rtn = start_per_ring[idxRing]+leds_per_ring[idxRing];
  } // end if special case
  return((uint8_t) rtn);
} // end RBG_ringIdxIncrDecr(...)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_diskInitBrightSpots(brightSpots, color, direction, fade) - 
//
// brightSpots - list of positions within ring and color for bright spots
// pColor      - pointer to color to use as fill (typically &CRGB::Black)
// direction   - if zero, ignore direction and fade
//               if non-zero, take that many steps in that direction, fading to fade/256 value each step
// fade        - ignored if direction is zero, else fade/256 each step
//
// BUG: There is some sort of Arduino compiler bug when using brightSpots_t* as an input param
//
void RBG_diskInitBrightSpots(void* tmpPtr, CRGB* pColor, int8_t direction, uint16_t fade) {
  brightSpots_t* brightSpots = (brightSpots_t*) tmpPtr;
  int16_t idxBrtspt, idxRing, idxFade, idx;
  CRGB myHue;

  for (idx=0; idx < NUM_LEDS_PER_DISK; idx++) {
    led_display[idx] = *pColor;
  } // end set all to pColor
  for (idxRing=0; idxRing<NUM_RINGS_PER_DISK; idxRing++) {
    for (idxBrtspt=0; (brightSpots[idxBrtspt].posn < leds_per_ring[idxRing]) && (idxBrtspt < leds_per_ring[idxRing]); idxBrtspt++) {
      myHue = brightSpots[idxBrtspt].hue;
      led_display[brightSpots[idxBrtspt].posn+start_per_ring[idxRing]] = myHue;
      // note: sgn(direction) is -1, 0, 1 for (direction < 0), (direction == 0), (direction > 0) respectively
      idxFade = brightSpots[idxBrtspt].posn+start_per_ring[idxRing];
      for (idxFade = RBG_ringIdxIncrDecr(idxFade, idxRing, direction); 0 != sgn(direction); idxFade = RBG_ringIdxIncrDecr(idxFade, idxRing, direction)) {
        myHue.fadeToBlackBy(fade);
        led_display[idxFade] = myHue;
        direction -= sgn(direction); // decrement the count
      } // end do fade if requested
    } // end for all bright spots this ring
  } // end for all rings
} // end RBG_diskInitBrightSpots(...)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_diskRotateOrDrain(direction, pColor) - 
//
// direction - +/- 1 = rotate through, +/- 2 = down drain. With these rings, + is counter-clockwise and - is clockwise
// pColor    - pointer to color to use as fill (typically &CRGB::Black)
// 
void RBG_diskRotateOrDrain(int8_t direction, CRGB* pColor) {
  // do rotate/drain
  if (direction > 0) { // counterclockwise
    if (1 == direction) { led_tmp1 = led_display[0]; } else { led_tmp1 = *pColor; }
    for (int idx=1; idx < NUM_LEDS_PER_DISK; idx++) {
      led_display[idx-1] = led_display[idx];
    }
    led_display[NUM_LEDS_PER_DISK-1] = led_tmp1;
  } else  { // clockwise
    if (-1 == direction) { led_tmp1 = led_display[NUM_LEDS_PER_DISK-1]; } else { led_tmp1 = *pColor; }
    for (int idx=NUM_LEDS_PER_DISK-1; idx > 0; idx--) {
      led_display[idx] = led_display[idx-1];
    }
    led_display[0] = led_tmp1;
  }
} // end RBG_diskRotateOrDrain()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_ringRotateOrDrain(direction, pColor, whichRing) - 
//
// direction - +/- 1 = rotate through, +/- 2 = down drain. With these rings, + is counter-clockwise and - is clockwise
// pColor    - pointer to color to use as fill (typically &CRGB::Black)
// whichRing - which ring to rotate, 0 through (NUM_RINGS_PER_DISK-1) {2 for RBG}
// 
void RBG_ringRotateOrDrain(int8_t direction, CRGB* pColor, uint8_t whichRing) {
  // do rotate/drain
  int16_t idx;
  if (direction > 0) { // counterclockwise
    if (1 == direction) { led_tmp1 = led_display[start_per_ring[whichRing]]; } else { led_tmp1 = *pColor; }
    for (idx=start_per_ring[whichRing]+1; idx < start_per_ring[whichRing]+leds_per_ring[whichRing]; idx++) {
      led_display[idx-1] = led_display[idx];
    }
    led_display[led_display[start_per_ring[whichRing]+leds_per_ring[whichRing]-1]] = led_tmp1;
  } else  { // clockwise
    if (-1 == direction) { led_tmp1 = led_display[start_per_ring[whichRing]+leds_per_ring[whichRing]-1]; } else { led_tmp1 = *pColor; }
    for (idx=start_per_ring[whichRing]+leds_per_ring[whichRing]-1; idx > start_per_ring[whichRing]; idx--) {
      led_display[idx] = led_display[idx-1];
    }
    led_display[0] = led_tmp1;
  }
} // end RBG_ringRotateOrDrain()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_RailGunEffect(myInit, pColor)
//
// myInit - nonzero for initialization
//
// do 27 cycles; be long enough for any of the SHOOT sounds
//
#define DEBUG_RBG_RailGunEffect 0
void RBG_RailGunEffect(uint8_t myInit, CRGB* pColor) {
  static uint16_t myStep = 0;
  static int8_t lastState = 99;
  uint8_t idx;

#if DEBUG_RBG_RailGunEffect
  Serial.print(F("DEBUG rail BEFORE myInit=")); Serial.print(myInit); Serial.print(F(" myStep=")); Serial.print(myStep); Serial.print(F(" ptrnDelayLEDstep=")); Serial.print(myState.ptrnDelayLEDstep); Serial.print(F(" millis()=")); Serial.print(millis()); Serial.print(F(" timerNow=")); Serial.print(myState.timerNow); Serial.print(F(" timerPrevLEDstep=")); Serial.print(myState.timerPrevLEDstep); Serial.print(F(" lastState=")); Serial.println(lastState);
#endif // DEBUG_RBG_RailGunEffect
  if (0 != myInit) {
    myStep = 0;
    myState.ptrnDelayLEDstep = 25;
    lastState = -1; // DEBUG
    for (idx = 0; idx < NUM_LEDS_PER_DISK; idx++) { led_display[idx] = led_BLACK; }
  } else {
    if (myStep < 8*4*27) { // 27 cycles
      switch ((myStep / 8) % 4) {
        case 0: // after some black, set ring[2] to the color
          for (idx = start_per_ring[2]; idx < start_per_ring[2]+leds_per_ring[2]; idx++) {
            led_display[idx] = *pColor;
          } // end for smallest ring
          lastState = 0; // DEBUG
          break;
        case 1: // set ring[2] black and set ring[1] to the color
          for (idx = start_per_ring[2]; idx < start_per_ring[2]+leds_per_ring[2]; idx++) {
            led_display[idx] = led_BLACK;
          }
          for (idx = start_per_ring[1]; idx < start_per_ring[1]+leds_per_ring[1]; idx++) {
            led_display[idx] = *pColor;
          } // end for middle ring
          lastState = 1; // DEBUG
          break;
        case 2: // set ring[1] black and set ring[0] to the color
          for (idx = start_per_ring[1]; idx < start_per_ring[1]+leds_per_ring[1]; idx++) {
            led_display[idx] = led_BLACK;
          } // end for smallest ring
          for (idx = start_per_ring[0]; idx < start_per_ring[0]+leds_per_ring[0]; idx++) {
            led_display[idx] = *pColor;
          } // end for largest ring
          lastState = 2; // DEBUG
          break;
        case 3: // do speed-up and set ring[0] black for next cycle
          if (myStep < 8*4*1) {
            myState.ptrnDelayLEDstep = 16; // speed it up
          } else if (myStep < 8*4*2) {
            myState.ptrnDelayLEDstep = 7; // speed it up
          }
          for (idx = start_per_ring[0]; idx < start_per_ring[0]+leds_per_ring[0]; idx++) {
            led_display[idx] = led_BLACK;
          } // all black
          lastState = 3; // DEBUG
          break;
      } // end switch every eight intervals
      myStep += 1;
      // end for 27 cycles
    } else {
      // then just effect
      RBG_confetti_fadeby(128);
      lastState = 4; // DEBUG
      // confetti();
    } // end if lots of steps
  } // end if not initialization
#if DEBUG_RBG_RailGunEffect
  Serial.print(F("DEBUG rail AFTER  myInit=")); Serial.print(myInit); Serial.print(F(" myStep=")); Serial.print(myStep); Serial.print(F(" ptrnDelayLEDstep=")); Serial.print(myState.ptrnDelayLEDstep); Serial.print(F(" millis()=")); Serial.print(millis()); Serial.print(F(" timerNow=")); Serial.print(myState.timerNow); Serial.print(F(" timerPrevLEDstep=")); Serial.print(myState.timerPrevLEDstep); Serial.print(F(" lastState=")); Serial.println(lastState);
#endif // DEBUG_RBG_RailGunEffect
} // end RBG_RailGunEffect()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_bpm_rings() - a variant of one of Mark Kriegsman's classic DemoReel100.ino patterns
//
void RBG_bpm_rings() { // my mod of pattern from Demo Reel 100
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62+10;
  CRGBPalette16 palette = PartyColors_p;
  int16_t beat = beatsin8( BeatsPerMinute, 0, 255);
  for ( uint8_t ring = 0; ring < NUM_RINGS_PER_DISK; ring++ ) {
    beat = -beat+255; // reverse lighting directions
    for ( uint8_t i = start_per_ring[ring]; i < (start_per_ring[ring]+leds_per_ring[ring]); i++ ) {
      led_display[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
    } // end for LEDs
  } // end for rings
} // end RBG_bpm_rings()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_confetti_fadeby(fadeVal) - a variant of one of Mark Kriegsman's classic DemoReel100.ino patterns
//
// fadeVal -  // 8 bit, 1 = slow, 255 = fast
//
void RBG_confetti_fadeby(uint8_t fadeVal) { // pattern from Demo Reel 100
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(led_display, NUM_LEDS_PER_DISK, fadeVal);
  int pos = random16(NUM_LEDS_PER_DISK);
  led_display[pos] += CHSV( gHue + random8(64), 200, 255);
} // end confetti()

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// RBG_juggle_numdot_ring(numDots) - a variant of one of Mark Kriegsman's classic DemoReel100.ino patterns
//
// numDots: >0 - number of dots, travel entire disk
//          <0 - number of dots, travel within rings
//          =0 - same as 8
//
void RBG_juggle_numdot_ring(int8_t numDots) { // pattern from Demo Reel 100
  // colored dots, weaving in and out of sync with each other
  byte dothue = 0;

  fadeToBlackBy( led_display, NUM_LEDS_PER_DISK, 20);
  if (0 == numDots) { numDots = 8; }
  if (0 < numDots) {
    for( int i = 0; i < numDots; i++) {
      led_display[beatsin16(i+7,0,NUM_LEDS_PER_DISK-1)] |= CHSV(dothue, 200, 255);
      dothue += 32;
    }
  } else { // <0 means travel within rings
    uint16_t tmp;
    numDots = -numDots; // make it >0
    if (numDots > MIN_LEDS_PER_RING) { numDots = MIN_LEDS_PER_RING; }
    for ( uint8_t ring = 0; ring < NUM_RINGS_PER_DISK; ring++ ) {
      for( int i = 0; i < numDots; i++) {
        tmp = beatsin16(i+7,start_per_ring[ring],start_per_ring[ring]+leds_per_ring[ring]-1);
        led_display[tmp] |= CHSV(dothue, 200, 255);
        dothue += 32;
      } // end for dots
    } // end for rings
  } // end if rings or whole disk
} // end RBG_juggle_numdot_ring()

// ******************************** DEBUG UTILITIES ********************************

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// checkDataGuard()
void checkDataGuard() {
  static int8_t showOneTime = 1;
  if ((showOneTime >= 1) && ((0x55555555 != data_guard_before) || (0x55555555 != data_guard_after))) {
    Serial.print(F("checkDataGuard should be 0x55555555; before=0x"));
    Serial.print(data_guard_before, HEX);
    Serial.print(F(" after=0x"));
    Serial.println(data_guard_after, HEX);
    delay(2000); // for debugging & show
    showOneTime--;
  }
} // end checkDataGuard()
