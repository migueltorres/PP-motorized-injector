#include "motor.h"
#include "globals.h"
#include <FastAccelStepper.h>

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

void initMotor() {
    engine.init();
    stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN);
    if (stepper) {
        stepper->setDirectionPin(STEPPER_DIR_PIN);
        stepper->setSpeedInHz(MOTOR_MAX_SPEED);
        stepper->setAcceleration(MOTOR_ACCELERATION);
    }
}

void moveMotor(bool forward) {
    if (stepper) {
        stepper->move(forward ? INT32_MAX : -INT32_MAX);
    }
}

void stopMotor() {
    if (stepper) {
        stepper->forceStopAndNewPosition(0);
    }
}
