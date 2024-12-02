/* yet another re-write of Motorizec Injector Sketch, this time for ESP32, as should be able to 
a) use Core 1 ONLY for motor control, avoiding any types of delays/ interferences/ any other types 
of extra MCU core loads that interfiere with correct step pulse generacion at speeds requiered
b) use PCNT for encoder pulse counting, so should also be completely independent of MCU loads,
and callable at any time from the main loop to report encoder position

will use StateMachine to decide all states and movements at any point in time, with decisions based
on machine/user input to move from one State to another

both ProgrammedMotorMove and EncoderActualPosition functions will be basic, so as to re-use in as
many places as needed, only with in each State, before/after calling the function, will the local
variables be changed as needed to the global variables that these functions will use to make motor
move / get motor real position




*/
// files to include
#include "globals.h"
#include "state_machine.h"
#include "functions.h"
#include "constructors.h"


// libraries to include

#include <ESP32Encoder.h> // * https://github.com/madhephaestus/ESP32Encoder.git 
#include <FastAccelStepper.h>  // *
#include <elapsedMillis.h>
//#include <ACS712.h> // current sensor, may not be needed, but added 2 to motor phase B + & - just in case
#include <SPI.h>
#include <Adafruit_MAX31855.h>  // temp board, was enclosed with quotes "" in example, not sure why
#include <Adafruit_NeoPixel.h>  // ws2812B RGB addressable LEDs
#include <Bounce2.h>    // bounce library allows to time button presses




void setup() 
{
  Serial.begin ( 115200 );

// encoder setup
  encoder.attachHalfQuad ( ENCODERApin, ENCODERBpin );  // possible have to reverse..? or rename, for Encoder library "CLK, DT ", and also rename EncoderPins..?
  // encoder.setCount ( 0 );  move this line to INIT_HOMED_ENCODER_ZEROED function?

// stepper setup
  engine.init(1);  // 
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_RMT);  // as have not defined DRIVER_RMT, maybe should use "1"?
 /*  refere to https://github.com/gin66/FastAccelStepper/blob/master/extras/doc/FastAccelStepper_API.md
 for ways to CPU_CORE & DRIVER_TYPE, if this does not compile as-is, and 
 https://valarsystems.com/blogs/val-2000/section-9-dual-core-setup for alternative CPU_CORE method
 */
  if (stepper) 
  {
    stepper->setDirectionPin(dirPinStepper); // possible not needed if moveTo commands are also -ve..?
  }

// keypadLEDs setup
  keypadleds.begin();  // Call this to start up the LED strip.
//  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.show();   // ...but the LEDs don't actually update until you call this.

// Buttons Bounce setup
SelectButtonBounce.attach(BUTTONSelectPin, INPUT_PULLUP);
SelectButtonBounce.interval(5);
UpButtonBouce.attach(BUTTONUpPin, INPUT_PULLUP);
UpButtonBouce.interval(5);
DownButtonBounce.attach(BUTTONDownPin, INPUT_PULLUP)
DownButtonBounce.interval(5);

}

void loop() 
{
  // put your main code here, to run repeatedly:

}
