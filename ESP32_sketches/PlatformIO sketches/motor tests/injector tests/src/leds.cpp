#include "leds.h"
#include <Adafruit_NeoPixel.h>
#include "globals.h"

Adafruit_NeoPixel buttonLEDs(3, WS2812B_BUTTON_LEDS, NEO_GRB + NEO_KHZ800);

void initLEDs() {
    buttonLEDs.begin();
    buttonLEDs.show();
}

void setModeLEDs(OperationMode mode) {
    if (mode == CONTINUOUS_MOVE) {
        buttonLEDs.setPixelColor(0, buttonLEDs.Color(0, 255, 0)); // Up button green
        buttonLEDs.setPixelColor(1, buttonLEDs.Color(0, 0, 255)); // Select button blue
        buttonLEDs.setPixelColor(2, buttonLEDs.Color(0, 255, 0)); // Down button green
    } else {
        buttonLEDs.setPixelColor(0, buttonLEDs.Color(255, 0, 0)); // Up button red
        buttonLEDs.setPixelColor(1, buttonLEDs.Color(255, 165, 0)); // Select button orange
        buttonLEDs.setPixelColor(2, buttonLEDs.Color(255, 0, 0)); // Down button red
    }
    buttonLEDs.show();
}
