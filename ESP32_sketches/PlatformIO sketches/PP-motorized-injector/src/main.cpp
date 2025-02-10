
/* program to control motorized injection machine via ESP32 for direct Nema34 160mm with external DM860H
driver, 3 inductive endstops, 3 lower keypad buttons with Neopixel LEDs behind them to indicate distinct
funtionality, SPI thermocouple, & independant Encoder to Motor comparison, all via a State Machine to 
progress thru different machine states, using either sensor or user input via buttons

sketch should return via Serial data on Encoder and Temp, later this ESP32 will be connected via UART to 
a ESP32 Display, which will hold different mould structs, and therefore be able to load to the motor ESP32
distinct injection parameters as per mould to inject, and also keep track from Encoder data of when the 
injector is Refilled, so each Refill will be graphically represented by a rectangule inside a tube like 
graphic, move down and change color as injection and refills happens, & have a timing tracker to be able 
to see if it has had enough time to melt (avoiding cold injections!) */

#include <Arduino.h>
#include <SPI.h>

#include <FastAccelStepper.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>

#include "RotaryEncoderPCNT.h"

#include "config.h"

void setup() {
    Serial.begin(9600);

    delay(2000); // wait 2 seconds. REQUIERED !!!
    Serial.println("PP_injector_ESP32");

    // encoder setup

// FIXME commented out for testing
    //encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);  // possible have to reverse..? or rename, for Encoder library "CLK, DT ", and also rename EncoderPins..?
    // encoder.setCount ( 0 );  move this line to INIT_HOMED_ENCODER_ZEROED function?

    // stepper setup
    //#define DRIVER_RMT 1  // type of driver for FAS, to separate any motor interference, not needed as directly used "1" below..?
    engine.init();                                           
    
    // FIXME  Core assignment
    // engine.init(1); 
    // stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN, 1);  // "1" defines DRIVER_RMT, for encoder to use PCNT, not Stepper
    // refere to https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md
    // for ways to CPU_CORE & DRIVER_TYPE, if this does not compile as-is, and 
    // https://valarsystems.com/blogs/val-2000/section-9-dual-core-setup for alternative CPU_CORE method

    stepper = engine.stepperConnectToPin(STEPPER_STEP_PIN);
    stepper->setAbsoluteSpeedLimit(8000);  // 16.000.000 tick/s / 2000steps/s maxSpeedLimit = 8000 ticks/ step
                                                           // 16.000.000 tick/s / 5000steps/s maxSpeedLimit = 3200 ticks/ step

    if (stepper) {
      stepper->setDirectionPin(STEPPER_DIR_PIN);  // possible not needed if moveTo commands are also -ve..?
    }

    // thermocouple setup
    thermocouple.begin();

    // keypadLEDs setup
    keypadleds.begin();  // Call this to start up the LED strip.
    //  clearLEDs();   // This function, defined below, turns all LEDs off...
    keypadleds.show();  // ...but the LEDs don't actually update until you call this.

    // Buttons & Endstops Bounce2 setup is very long, so called from the following function
    bounceButtonsSetup();

    currentState = InjectorStates::INIT_HEATING;
    error = InjectorError::NO_ERROR;

    readNozzleTemp = true;
    readEmergencyStop = true;
}

long now; 
long fastTaskTime = 0;
long mediumTaskTime = 0;
long slowTaskTime = 2;


/** 
 * Comms functions, every 100ms send data via serial
 */
void serialRegular100msMessages() {
  Serial.println("===============");
  Serial.printf("loops: %d, avg time: %d\n", numLoops, avgLoopTime);
  Serial.printf("state: %d, ER: %d\n",currentState, error);
  Serial.println("selectLed: " + String(currentSelectLEDcolour, HEX));
  Serial.println("upLed: " + String(currentUpLEDcolour, HEX));
  Serial.println("downLed: " + String(currentDownLEDcolour, HEX));

  Serial.printf("temps: NZ=%d\n", nozzleTemperature);

  Serial.printf("inputs: ES=%d, SEL=%d, UP=%d, DW=%d, TE=%d, BE=%d, BR=%d\n", emergencyStop, selectButtonPressed, upButtonPressed, downButtonPressed, topEndStopActivated, bottomEndStopActivated, barrelEndStopActivated);

// // FIXME commented out for testing
//       //actualENPosition = encoder.getCount() / 2;
//       if (actualENPosition != oldENPosition) /* ... and actualENPosition & oldENPosition are updated by EncoderActualPosition, but if there
//      there has been no change in last 100ms, then do not print new position... possibly this should be divided in loop..? */
//       {
//         Serial.print("Encoder: ");
//         Serial.println(actualENPosition);
//         oldENPosition = actualENPosition;
//       }
//       Serial.print("TempBox: ");
//       Serial.println(thermocouple.readInternal());
//       Serial.print("TempNozzle: ");
//       Serial.println(nozzleTemperature);

}

void loop() {
  // this eventloop runs tasks based on 3 different rates: high speed tasks running every loop interation, fast tasks running every 1ms, medium tasks at 10ms, and slow tasks at 100ms
  
  // high speed tasks
  //bounce.update();

  now = millis();
  if (now - fastTaskTime  >= 1) {
    fastTaskTime = now;
    // fast tasks
  
  }
  if (now - mediumTaskTime >= 10) {
    mediumTaskTime = now;
    // medium tasks
    //stateMachineLoop();

  }
  if (now - slowTaskTime >= 100) {
    slowTaskTime = now;
    // slow tasks
    serialRegular100msMessages();
  }

}

