#include <Arduino.h>
#include "MotorControl.h"
#include "globals.h"
#include <SPI.h>
#include <Wire.h>
#include <RotaryEncoderPCNT.h>

RotaryEncoderPCNT encoder(ENCODER_A_PIN, ENCODER_B_PIN);

void setup() 
{
  initMotor();
}


void loop() {
  // put your main code here, to run repeatedly:
}

