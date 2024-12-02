#include "encoder.h"
#include "globals.h"

volatile int32_t encoderPosition = 0;
bool lastEncoderAState = false;
bool lastEncoderBState = false;

void IRAM_ATTR handleEncoderInterruptA() {
    bool encoderAState = digitalRead(ENCODER_A_PIN);
    bool encoderBState = digitalRead(ENCODER_B_PIN);
    if (encoderAState != lastEncoderAState) {
        encoderPosition += (encoderAState == encoderBState) ? 1 : -1;
        lastEncoderAState = encoderAState;
    }
}

void IRAM_ATTR handleEncoderInterruptB() {
    bool encoderAState = digitalRead(ENCODER_A_PIN);
    bool encoderBState = digitalRead(ENCODER_B_PIN);
    if (encoderBState != lastEncoderBState) {
        encoderPosition += (encoderBState != encoderAState) ? 1 : -1;
        lastEncoderBState = encoderBState;
    }
}

void initEncoder() {
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoderInterruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), handleEncoderInterruptB, CHANGE);
}

int32_t getEncoderPosition() {
    return encoderPosition;
}

void resetEncoderPosition() {
    encoderPosition = 0;
}
