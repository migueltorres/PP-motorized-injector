#include "endstops.h"
#include "globals.h"
#include "motor.h"
#include "encoder.h"

void initEndstops() {
    pinMode(ENDSTOP_TOP_PLUNGER, INPUT_PULLUP);
    pinMode(ENDSTOP_BOTTOM_PLUNGER, INPUT_PULLUP);
    pinMode(ENDSTOP_BARREL_CLAMP_OK, INPUT_PULLUP);
    pinMode(ENDSTOP_NOZZLE_BLOCK, INPUT_PULLUP);
}

void checkEndstops(OperationMode mode) {
    if (digitalRead(ENDSTOP_TOP_PLUNGER) == LOW) {
        stopMotor();
        resetEncoderPosition();
        Serial.println("Top Endstop reached, Encoder reset.");
    } else if (digitalRead(ENDSTOP_BOTTOM_PLUNGER) == LOW) {
        stopMotor();
        Serial.println("Bottom Endstop reached.");
    } else if (mode == ENDSTOP_MOVE) {
        if (digitalRead(ENDSTOP_BARREL_CLAMP_OK) == LOW) {
            stopMotor();
            Serial.println("Barrel Clamp Endstop triggered.");
        }
        if (digitalRead(ENDSTOP_NOZZLE_BLOCK) == LOW) {
            stopMotor();
            Serial.println("Nozzle Block Endstop triggered.");
        }
    }
}
