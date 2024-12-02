#include <Arduino.h>
#include "buttons.h"
#include "globals.h"
#include "leds.h"
#include "motor.h"

unsigned long lastDebounceTime = 0;

void initButtons() {
    pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
    pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
}

void handleButtons(OperationMode& mode) {
    if (millis() - lastDebounceTime > DEBOUNCE_TIME) {
        if (digitalRead(BUTTON_SELECT_PIN) == LOW) {
            mode = (mode == CONTINUOUS_MOVE) ? ENDSTOP_MOVE : CONTINUOUS_MOVE;
            setModeLEDs(mode);
            lastDebounceTime = millis();
        }

        if (digitalRead(BUTTON_UP_PIN) == LOW) {
            moveMotor(true);
        } else if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
            moveMotor(false);
        } else {
            stopMotor();
        }
    }
}
