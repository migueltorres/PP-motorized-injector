#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <FastAccelStepper.h>

void initMotor();
void ProgrammedMotorMove(int motorSpeed, int motorAcceleration, int motorMoveSteps); // simply moves motor at set speed, accel and distance indicated
void ContinuousMotorMoveForward(int motorSpeed);                                     // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
void ContinuousMotorMoveBackward(int motorSpeed); // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
void stopMotor();
void setMotorSpeed(int motorSpeed);

#endif