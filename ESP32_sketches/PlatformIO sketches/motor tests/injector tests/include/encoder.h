#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

void initEncoder();
int32_t getEncoderPosition();
void resetEncoderPosition();

#endif
