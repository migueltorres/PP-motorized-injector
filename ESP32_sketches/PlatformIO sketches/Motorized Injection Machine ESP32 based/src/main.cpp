#include <Arduino.h>
#include "MotorControl.h"
#include "globals.h"
#include <SPI.h>
#include <Wire.h>

initMotor();
ESP32Encoder encoder;

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}