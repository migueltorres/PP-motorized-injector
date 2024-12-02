// injector_ESP32_state_machine.ino

/* sketch to control motorized injection machine via ESP32 for direct Nema34 160mm with external DM860H
driver, 3 inductive endstops, 3 lower keypad buttons with Neopixel LEDs behind them to indicate distinct
funtionality, SPI thermocouple, & independant Encoder to Motor comparison, all via a State Machine to 
progress thru different machine states, using either sensor or user input via buttons

sketch should return via Serial data on Encoder and Temp, later this ESP32 will be connected via UART to 
a ESP32 Display, which will hold different mould structs, and therefore be able to load to the motor ESP32
distinct injection parameters as per mould to inject, and also keep track from Encoder data of when the 
injector is Refilled, so each Refill will be graphically represented by a rectangule inside a tube like 
graphic, move down and change color as injection and refills happens, & have a timing tracker to be able 
to see if it has had enough time to melt (avoiding cold injections!) */

// files to include
#include "globals.h"
#include "functions.h"
#include "state_machine.h"


// libraries to include

#include <ESP32Encoder.h> // * https://github.com/madhephaestus/ESP32Encoder.git 
ESP32Encoder encoder; // should use PCNT timer on ESP32, so should be callable at any moment with updated encoder value..?

#include <FastAccelStepper.h>  // *
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
setAbsoluteSpeedLimit(16000000/maxSpeedLimit);  // 16.000.000 tick/s / 2000steps/s maxSpeedLimit = 8000 ticks/ step
                                                // 16.000.000 tick/s / 5000steps/s maxSpeedLimit = 3200 ticks/ step

#include <elapsedMillis.h>
elapsedMillis printTime;

//#include <ACS712.h> // current sensor, may not be needed, but added 2 to motor phase B + & - just in case
#include <SPI.h>
#include <Adafruit_MAX31855.h>  // temp board, was enclosed with quotes "" in example, not sure why
Adafruit_MAX31855 thermocouple(TEMPNozzleVSPI_SCK_CLK, TEMPNozzleVSPI_Dpin_MOSI_CS, TEMPNozzleVSPI_MISO_DO);  

#include <Adafruit_NeoPixel.h>  // ws2812B RGB addressable LEDs
Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(buttonLedCount, WS2812B_addressableLEDs, NEO_GRB + NEO_KHZ800);

#include <Bounce2.h>    // bounce library allows to time button presses
Bounce2::Button down_button = Bounce2::Button();
Bounce2::Button up_button = Bounce2::Button();
Bounce2::Button select_button = Bounce2::Button();
Bounce2::Button TOP_ENDSTOP = Bounce2::Button();
Bounce2::Button BOTTOM_ENDSTOP = Bounce2::Button();
Bounce2::Button BARREL_ENDSTOP = Bounce2::Button();
Bounce2::Button EMERGENCY_STOP = Bounce2::Button();


void setup() 
{
  Serial.begin ( 115200 );
  Serial.println("injector_ESP32_state_machine.ino");

  // encoder setup
  encoder.attachHalfQuad ( ENCODERApin, ENCODERBpin );  // possible have to reverse..? or rename, for Encoder library "CLK, DT ", and also rename EncoderPins..?
  // encoder.setCount ( 0 );  move this line to INIT_HOMED_ENCODER_ZEROED function?

  // stepper setup
  //#define DRIVER_RMT 1  // type of driver for FAS, to separate any motor interference, not needed as directly used "1" below..?
  engine.init(1);  //   Core assignment
  stepper = engine.stepperConnectToPin(stepPinStepper, 1);  // "1" defines DRIVER_RMT, for encoder to use PCNT, not Stepper
 /*  refere to https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md
 for ways to CPU_CORE & DRIVER_TYPE, if this does not compile as-is, and 
 https://valarsystems.com/blogs/val-2000/section-9-dual-core-setup for alternative CPU_CORE method
 */
  if (stepper) 
  {
    stepper->setDirectionPin(dirPinStepper); // possible not needed if moveTo commands are also -ve..?
  }

  // thermocouple setup
  thermocouple.begin();

  // keypadLEDs setup
  keypadleds.begin();  // Call this to start up the LED strip.
  //  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.show();   // ...but the LEDs don't actually update until you call this.

  // Buttons & Endstops Bounce2 setup is very long, so called from the following function
  BounceButtonsSetup();


}

void loop() {
  // put your main code here, to run repeatedly:

}
