#include "MotorControl.h"

FastAccelStepperEngine engine;
FastAccelStepper *stepper;

void initMotor()
{
    engine.init(0);     // core assignment
    #if defined(SUPPORT_SELECT_DRIVER_TYPE)
    stepper = engine.stepperConnectToPin(33,1);
    #endif
    if (stepper)
    {
        stepper->setDirectionPin(32);
        stepper->setAcceleration(50000);
    }
}

void ProgrammedMotorMove(int motorSpeed, int motorMoveSteps, int motorAcceleration=50000) // simply moves motor at set speed, accel and distance indicated
// This would allow for simpler calls where acceleration doesn't change: ProgrammedMotorMove(1000, 2000); // Speed and steps only
//                                                                 ProgrammedMotorMove(1000, 2000, 60000); // Full control
{
    if (stepper)
    {
        stepper->setSpeedInHz(motorSpeed);          // speed in Hz = steps/s
        stepper->move(motorMoveSteps);
        stepper->setAcceleration(motorAcceleration);
    }
}

void ContinuousMotorMoveForward(int motorSpeed) // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
{
    if (stepper)
    {
        stepper->setSpeedInHz(motorSpeed);
        stepper->runForward();
    }
}
    
void ContinuousMotorMoveBackward(int motorSpeed) // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
{
    if (stepper)
    {
        stepper->setSpeedInHz(motorSpeed);
        stepper->runBackward();
    }
}


void stopMotor()
{
    if (stepper)
    {
        stepper->stopMove();
    }
}

void setMotorSpeed(int motorSpeed)
{
    if (stepper)
    {
        stepper->setSpeedInHz(motorSpeed);
    }
}
