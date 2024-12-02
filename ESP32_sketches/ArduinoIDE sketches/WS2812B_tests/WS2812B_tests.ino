/* test sketch for WS2182B addressable LEDs, to program 3 LEDs with 2 colour
states, for lower keypad use with injector
*/

#include <Adafruit_NeoPixel.h>
//#include "WS2812_Definitions.h"

#define PIN 25
#define LED_COUNT 3

#define GREEN			0x008000
#define RED			0xFF0000
#define YELLOW			0xFF8C00  //0xFF4500 orangey yellow
#define YELLOW2     0xFF8C00  // 0xFF8C00 nicest yellow so far
#define RED_rgb   255,0,0
#define GREEN_rgb  0,255,0
#define BLUE_rgb  0,0,255

// Create an instance of the Adafruit_NeoPixel class called "leds".
// That'll be what we refer to from here on...
Adafruit_NeoPixel keypadleds (LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);



void setup() 
{
  keypadleds.begin();  // Call this to start up the LED strip.
//  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.show();   // ...but the LEDs don't actually update until you call this.
  keypadleds.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

}

void clearLEDs()
{
  for (int i=0; i<LED_COUNT; i++)
  {
    keypadleds.setPixelColor(i, 0);
  }
}

void loop() 
{
//  clearLEDs();
  keypadleds.setBrightness(50);
  keypadleds.setPixelColor(0,GREEN);
  keypadleds.setPixelColor(1,YELLOW);
  keypadleds.setPixelColor(2,YELLOW2);
  keypadleds.show();
  delay(2000);
//  clearLEDs();
  keypadleds.setPixelColor(0,RED);
  keypadleds.setPixelColor(1,GREEN);
  keypadleds.setPixelColor(2,GREEN);
  keypadleds.show();
  delay(2000); 
  clearLEDs();
  keypadleds.show();
  delay(2000); 
  keypadleds.setPixelColor(0, RED_rgb);
  keypadleds.setPixelColor(0, GREEN_rgb);
  keypadleds.setPixelColor(0, BLUE_rgb);
  keypadleds.show();



}
