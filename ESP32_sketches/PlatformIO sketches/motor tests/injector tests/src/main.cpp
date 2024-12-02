#include <Arduino.h>
#include "globals.h"
#include "motor.h"
#include "leds.h"
#include "buttons.h"
#include "encoder.h"
#include "endstops.h"

// State variables
OperationMode currentMode = CONTINUOUS_MOVE;

void setup() {
    Serial.begin(115200);
    initMotor();
    initLEDs();
    initButtons();
    initEncoder();
    initEndstops();

    Serial.println("System initialized");
    setModeLEDs(currentMode);
}

void loop() {
    handleButtons(currentMode);
    checkEndstops(currentMode);

    // Periodically report encoder position
    static unsigned long lastReport = 0;
    if (millis() - lastReport > 500) { // Report every 500ms
        Serial.print("Encoder Position: ");
        Serial.println(getEncoderPosition());
        lastReport = millis();
    }
}
