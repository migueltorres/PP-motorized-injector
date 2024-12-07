/*   
stepper_2but_for_ESP32_fastaccel_encoder.ino
 */
 
#include "globals.h"

#include <RotaryEncoderPCNT.h>
#include <FastAccelStepper.h>
#include <elapsedMillis.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>



RotaryEncoderPCNT encoder(CLK, DT);


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

elapsedMillis printTime;

Adafruit_NeoPixel keypadleds (KEYS_LED_COUNT, KEYS_LED_PIN, NEO_GRB + NEO_KHZ800);

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button TOP_ENDSTOP = Bounce2::Button();
Bounce2::Button BOTTOM_ENDSTOP = Bounce2::Button();
Bounce2::Button BARREL_ENDSTOP = Bounce2::Button();
Bounce2::Button EMERGENCY_STOP = Bounce2::Button();
  /* ADJUST & ADD BUTTON PIN READS TO USE BOUNCE2 LIBRARY, as in bounce_inductive_tests_to_LEDs.ino */



void setup() 
{
  Serial.begin(115200);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);

old_position = encoder.position();
//Serial.println(old_position);


  engine.init();     // core assignment to 0..?  /* this causes panic messages and ESP sends errors
 //#if defined(SUPPORT_SELECT_DRIVER_TYPE)
    stepper = engine.stepperConnectToPin(stepPinStepper);  // convert to DRIVER 1, RMT
  //  #endif   /* this causes not to compile when included 2nd argument for RMT DRIVER */
 
  if (stepper) 
  {
    stepper->setDirectionPin(dirPinStepper);
    //stepper->setSpeedInHz(runSpeed);  // the parameter is Hz, steps/s !!!
    stepper->setAcceleration(motor_def_accel);
    //stepper->move(moveDist);
  }

  keypads_setup();
  bounce2_buttons_setup();

}

void keypads_setup()
{
  keypadleds.begin();            // Call this to start up the LED strip.
                                 //  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
  keypadleds.show();             // ...but the LEDs don't actually update until you call this.
}

void bounce2_buttons_setup() 
{
    /* ADJUST & ADD KEYPAD BUTTONs TO USE BOUNCE2 LIBRARY, as in bounce_inductive_tests_to_LEDs.ino */

  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR BUTTON HAS AN INTERNAL PULL-UP or PULL-DOWN
  TOP_ENDSTOP.attach(TOP_ENDSTOP_PIN, INPUT_PULLUP);
  BOTTOM_ENDSTOP.attach(BOTTOM_ENDSTOP_PIN, INPUT_PULLUP);    //Top & Bottom NO, 5v
  BARREL_ENDSTOP.attach(BARREL_ENDSTOP_PIN, INPUT_PULLDOWN);  //FINDA sensor is NC, 0v
  EMERGENCY_STOP.attach(EMERGENCY_STOP_PIN, INPUT_PULLUP);    // CHECK THAT IS NO!!

  // DEBOUNCE INTERVAL IN MILLISECONDS
  TOP_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  BOTTOM_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  BARREL_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  EMERGENCY_STOP.interval(DEBOUNCE_INTERVAL);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  TOP_ENDSTOP.setPressedState(LOW);  // NO, with pullup, goes LOW when closed
  BOTTOM_ENDSTOP.setPressedState(LOW);
  BARREL_ENDSTOP.setPressedState(HIGH);  //  NC, with pulldown, goes HIGH when active
  EMERGENCY_STOP.setPressedState(LOW);   //  CHECK THAT IS NO!!
}

void executeMotorCommand(int8_t runState)
{
  switch (runState)
  {
    case 0:
//	  stepper1.setAcceleration(200);
      stepper->stopMove();
	    motorSpeed = 0;
	    //stepper->setCurrentPosition(0);
      break;
    case 1:
	    stepper->setSpeedInHz(runSpeed);
	    stepper->runForward();
      break;
    case -1:
	    stepper->setSpeedInHz(runSpeed);
	    stepper->runBackward();
      break;
  }
}

void PrintData()
{
  if ((printTime >= 200))
   {
    Serial.print("Down: ");
    Serial.print(downPinState);
    Serial.print(", Up: ");
    Serial.print(upPinState);
    Serial.print(", rS: ");
    Serial.print(runState);
    Serial.print(", Sp: ");
    Serial.print(runSpeed);
    Serial.print(", En: ");
    Serial.print(position);
    Serial.print(", FAS: ");
    Serial.println(stepper->getCurrentPosition());
   /* Serial.print(" ");
    Serial.print("D");
    Serial.print(" ");
    Serial.println(stepper1.distanceToGo());*/
    printTime=0;
   }
}


void loop() 
{
  position = encoder.position()/10;
    // UPDATE THE BUTTON
  // YOU MUST CALL THIS EVERY LOOP
  BARREL_ENDSTOP.update();
  TOP_ENDSTOP.update();
  BOTTOM_ENDSTOP.update();
  EMERGENCY_STOP.update();
  /* ADJUST PIN READS TO USE BOUNCE2 LIBRARY, as in bounce_inductive_tests_to_LEDs.ino */


  downPinState = digitalRead(downPin);
  upPinState = digitalRead(upPin);

  if ((downPinState == 0 && upPinState == 0) || (downPinState == 1 && upPinState == 1)) 
  {
    runState = 0;
    executeMotorCommand(runState);
  }
  else if (downPinState == 0 && upPinState == 1)
  {
    runState = 1;
    executeMotorCommand(runState);
  }
  else if (downPinState == 1 && upPinState == 0)
  {
    runState = -1;
    executeMotorCommand(runState);
  }

  motorSpeed =  map(potReading, 0, 1023, 5, maxSpeedLimit);
  runSpeed = (motorSpeed); 

  PrintData();
}