// functions.h

#include <Arduino.h>

// functions declarations, to try and organize most commonly re-used functions
int EncoderActualPosition();                                             // reads encoder position and returns REFERENCE variable for use by other functions
void ResetEncoderZero();                                                 // resets Encoder to Zero, ONLY to be called ONCE at power up, toggle initialHomingDone
void ProgrammedMotorMove(motorSpeed, motorAcceleration, motorMove);    // simply moves motor at set speed, accel and distance indicated
void ContinuousMotorMoveForward(motorSpeed);                             // simply moves plunger continously (during button press or other condition is 0/1)
void ContinuousMotorMoveBackward(motorSpeed);                            // simply moves plunger continously (during button press or other condition is 0/1)
void HomeMove(motorSpeed, runDirection);                                 // simply sends plunger to TopEndstop - CAN USE ContinuousMotorMoveBackward
bool compareMotorEncoder();                                              /* function to compare Encoder & FastAccel position, to detect when skipping steps, 
and therefore compression of plastic or filling of mould, and to reduce speed/distance commands until are below min threshold,
when then returns "complete"/"1" to function where has been called from */
void compressionFunction(initialCompressionSpeed, minCompressionSpeed);  // ChatGPT written compression uses compareMotorEncoder funtion
void clearLEDs();                                                        // clears LEDs and turns off, before next colour setting
void ButtonLEDsColors(upLEDcolour, downLEDcolour, selectLEDcolour);      // sets colours of LEDs behind buttons
void SerialRegular100msMessages();                                        // every 100ms, send via serial the Temp, Encoder position, etc
void SerialChangeOfState();                               // serial print when a change of state happens, and new state
void SerialPrintNextInjectionValues();          // serial print the parameters of next injection
void BounceButtonsSetup();                      // setup of buttons list to avoid setup() overload
void BounceButtonsUpdateCall();                 // same for button update call in loop()
void CheckMinTempEndstopsEmergencyStop();                                /* should run in each loop, and if any endstop or emergency stop is triggered, stop machine 
and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or 
other options..? */

//////////////////////////////////

// functions fully described

// Encoder function will get called not every loop, but maybe every 10ms, to give time for max distances to trigger other code, and every 100ms
// wil be called by SerialRegular100msMessages, where if motor is stopped, actual = old, and no need to print value
void EncoderActualPosition() 
{
  actualENPosition = encoder.getCount() / 2;  // revise last int, if motor steps = 400 & encoder PPR = 1000, adjust this int
                                                   // (possible issue of needing to round, this is bad, better maybe to increase driver steps to same as encoder?)
  oldENPosition = actualENPosition;  // if this does NOT change between Serial.prints, then don't print

  return (actualENPosition);
}

void ResetEncoderZero() 
{
  encoder.clearCount();

  return;
}

// motor functions
/* NOTE: When calling this function from different states, it will be called with the arguments particular to that state, ex.
INJECT will pass ProgrammedMotorMove(FillMouldMoveSpeed, FillMouldAccel, FillMouldMoveDistSteps)... will this AUTOMATICALLY write those 
parameters to the functions named paramters, ie motorSpeed, motorAcceleration, motorMove..? Because if I wish to do a later
compareMotorEncoder function, this function needs to have the ProgrammedMotorMove NAMED params, not the PASSED params names..?
 */ 
void ProgrammedMotorMove(motorSpeed, motorAcceleration, motorMove)  // RELATIVE motor move to new position
{
  stepper->setSpeedInHz(motorSpeed);
  stepper->setAcceleration(motorAcceleration);
  stepper->move(motorMove);

  return;
}


void ContinuousMotorMoveForward(motorSpeed) // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
{
  stepper->setSpeedInHz(motorSpeed);
  stepper->runForward();

  return;
}

void ContinuousMotorMoveBackward(motorSpeed) // CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
{
  stepper->setSpeedInHz(motorSpeed);
  stepper->runBackward();

  return;
}

void HomeMove(GeneralFastSpeed, HomingSlowSpeed)  // once called, no user action can stop until complete!
{
  //int HomingDirection;      // maybe not needed, as using ContMoveBackwards
  int HomeOffSetDistSteps=212;   // once first reached HomeEndstop, how much to back off before slower approach, 212 steps ≈ 5mm
  int HomeOffSetAccel=10000;  // once first reached HomeEndstop, how much Accel to back off before slower approach 10000 = 1/5th normal
  int HomeOffsetSpeed=maxSpeedLimit/2;  // once first reached HomeEndstop, how quick to back off before slower approach
  int HomingSlowSpeed=maxSpeedLimit/10;  // how slow to home, 2nd approach, continous move until reaching endstop
  if (digitalRead(EndstopTOPPlunger) == !topPlungerEndstopActive) 
  {
    ContinuousMotorMoveBackward(GeneralFastSpeed);  // OR ProgrammedMotorMove with maxHomingStepsDistSteps, if fails, gives error?
  }
  ProgrammedMotorMove(HomeOffsetSpeed, HomeOffSetAccel, HomeOffSetDistSteps);  // programmed motor move assumed to be positive DistSteps..?
  if (digitalRead(EndstopTOPPlunger) == !topPlungerEndstopActive) 
  {
    ContinuousMotorMoveBackward(HomingSlowSpeed);
  }

  return;
}

// function to compare Motor & Encoder positions
bool compareMotorEncoder( motorMove, actualENPosition, comparisonPercentage) // chatGPT written
{
  // Calculate difference between motor position and encoder position
  int motorError = abs(motorMove - actualENPosition);
    
  // Calculate the acceptable difference threshold
  int threshold = (motorMove * comparisonPercentage) / 100;
    
  // Return true if the difference is greater than the threshold
  return (motorError >= threshold);
}

/* motor compression function takes compareMotorEncoder and motor moves to apply pressure up to a threshold and then
reduce the speed (increases torque of stepper?) until again threshold past, etc, until minCompressionSpeed reached, thenwill stop */
void compressionFunction(initialCompressionSpeed, minCompressionSpeed)  // chatGPT written
{
  ButtonLEDsColors (RED, BLACK, RED);
  select_button.update();

// add ButtonSelect interrupt function on user press, goes to Ready_to_Inject (also EStop..? or EStop better permament in loop
//  and then goes to ERROR-STATE?)
  if (!select_button.isPressed)
    {
    // Set initial speed
    stepper->setSpeedInHz(initialCompressionSpeed);  // Starting speed in steps per second

    while (stepper->getSpeed() > minCompressionSpeed) 
    {
      // Move the plunger down
      stepper->runForward();

      // Call the comparison function to check if motor-encoder difference exceeds threshold
      if (compareMotorEncoder(stepper->getTargetPosition(), encoder.getPosition(), comparisonPercentage)) 
        {
          // Reduce speed when the difference is greater than the allowed threshold
          int newSpeed = stepper->getSpeed() * (speedReductionFactor/10);
          stepper->setSpeedInHz(newSpeed);
          ButtonLEDsColors (RED, BLACK, YELLOW); // to show compression is acting (has reduced speed at least once) and ongoing..

          // Optionally, reduce comparison threshold on each iteration (for finer control) 
          comparisonPercentage *= speedReductionFactor;  /* could lead to infinite smaller steps if minCompressionSpeed is too low...? 
          for debugging this option, would be good to show actual speed - if later wish to add another compressionDuringHeating function
          where the plastic is already quite compressed, and will start (and end) from/to a much lower speeds, have to make sure this 
          does not produce infinite loop as never reaches minCompressionSpeed */
        }
    }
    
    // Stop motor when speed falls below threshold
    stepper->stopMove();
    ButtonLEDsColors (RED, BLACK, GREEN); // to show compression is complete
    delay(500); // ONLY DELAY IN ENTIRE SKETCH, and is only to give time for GREEN LED to be shown before

    return; // once finished function should return to function that called it, be it compression or inject.. should be break..?
    }
  else 
  {
    stepper->stopMove();
    StatesMachine::READY_TO_INJECT;
  }
}


void clearLEDs()
{
  for (int i=0; i<LED_COUNT; i++)
  {
    keypadleds.setPixelColor(i, 0);
  }
}


void ButtonLEDsColors (char upLEDcolour, char downLEDcolour, char selectLEDcolour);   //could use case/switch instead? but are different types of defining behaviours..?
{
  BounceButtonsUpdateCall();
  if (down_button.isPressed() || up_button.isPressed() || select_button.isPressed() ) // set brightness to about 80% on any button press
  { 
    keypadleds.setBrightness(ledHBrightness);   
    keypadleds.setPixelColor(0, upLEDcolour);
    keypadleds.setPixelColor(1, downLEDcolour);
    keypadleds.setPixelColor(2, selectLEDcolour);
    keypadleds.show(); 
  }
  else          // set brightness to about 30% when no button pressed
  {
    keypadleds.setBrightness(ledLBrightness);   
    keypadleds.setPixelColor(0, upLEDcolour);
    keypadleds.setPixelColor(1, downLEDcolour);
    keypadleds.setPixelColor(2, selectLEDcolour);
    keypadleds.show(); 
  }

  return;
}



// Comms functions, every 100ms send data via serial
void SerialRegular100msMessages()
{
  if ( millis()-100 >= printTime)  // only when 100ms has past...
  {
    actualENPosition = encoder.getCount() / 2;
    if (actualENPosition != oldENPosition)  /* ... and actualENPosition & oldENPosition are updated by EncoderActualPosition, but if there
     there has been no change in last 100ms, then do not print new position... possibly this should be divided in loop..? */
    {
      Serial.print("Encoder: ");
      Serial.println(actualENPosition);
      oldENPosition = actualENPosition; 
    }
    Serial.print("TempBox: ");
    Serial.println(thermocouple.readInternal());
    Serial.print("TempNozzle: ");
    Serial.println(thermocouple.readCelsius());
    printTime = millis();  
  }

  return;
}

void SerialChangeOfState()
{
  if (runState !=runState)
  {
    Serial.print("State Machine change to: ");
    Serial.println(runState);
  }

  return;
}

void SerialPrintNextInjectionValues()
{
  Serial.println("Next injection values:");
  // add here struct of variablePerMouldParams

  return;
}


void BounceButtonsSetup()
{
  // IF YOUR BUTTON HAS AN INTERNAL PULL-UP or PULL-DOWN
  down_button.attach( DOWN_BUTTON_PIN, INPUT_PULLUP);
  up_button.attach( UP_BUTTON_PIN ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  select_button.attach( SELECT_BUTTON_PIN, INPUT_PULLUP );
  TOP_ENDSTOP.attach(TOP_ENDSTOP_PIN, INPUT_PULLUP);
  BOTTOM_ENDSTOP.attach(BOTTOM_ENDSTOP_PIN, INPUT_PULLUP);    //Top & Bottom NO, 5v
  BARREL_ENDSTOP.attach(BARREL_ENDSTOP_PIN, INPUT_PULLDOWN);  //FINDA sensor is NC, 0v
  EMERGENCY_STOP.attach(EMERGENCY_STOP_PIN, INPUT_PULLUP);    // CHECK THAT IS NO!!

  // DEBOUNCE INTERVAL IN MILLISECONDS
  down_button.interval(DEBOUNCE_INTERVAL); 
  up_button.interval(DEBOUNCE_INTERVAL); 
  select_button.interval(DEBOUNCE_INTERVAL);   
  TOP_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  BOTTOM_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  BARREL_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  EMERGENCY_STOP.interval(DEBOUNCE_INTERVAL);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  down_button.setPressedState(LOW); 
  up_button.setPressedState(LOW); 
  select_button.setPressedState(LOW);
  TOP_ENDSTOP.setPressedState(LOW);  // NO, with pullup, goes LOW when closed
  BOTTOM_ENDSTOP.setPressedState(LOW);
  BARREL_ENDSTOP.setPressedState(HIGH);  //  NC, with pulldown, goes HIGH when active
  EMERGENCY_STOP.setPressedState(LOW);   //  CHECK THAT IS NO!!
}

void BounceButtonsUpdateCall()
{
  down_button.update();
  up_button.update();
  select_button.update();
  BARREL_ENDSTOP.update();
  TOP_ENDSTOP.update();
  BOTTOM_ENDSTOP.update();
  
  EMERGENCY_STOP.update();
}

char CheckMinTempEndstopsEmergencyStop()  /* should run in each loop, or at least every 10ms, and if any endstop or emergency stop is triggered, stop machine                              
and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or 
other options..? */
{
  tempNozzleDegrees=thermocouple.readCelsius(); // digitalRead, maybe every 10ms, state of temo, compare to minTemp, EmergencyStop, all ENdstops, etc, and if any are active, give error
  if (tempNozzleDegrees <= minTempForAnyMove)     // and stop machine
  {
    runState=StatesMachine::ERROR_STATE;
    return A;
  }
  BounceButtonsUpdateCall();
  else if (BARREL_ENDSTOP.isPresssed());
  {
    runState=StatesMachine::ERROR_STATE;
    return B;
  }
  else if (TOP_ENDSTOP.isPresssed());
  {
    runState=StatesMachine::ERROR_STATE;
    return C;
  }
  else if (BOTTOM_ENDSTOP.isPresssed());
  {
    runState=StatesMachine::ERROR_STATE;
    return D;
  }
  else if (EMERGENCY_STOP.isPresssed());
  {
    runState=StatesMachine::ERROR_STATE;
    return E;
  }
  else 
  {
    break;  // or should be "return"..?
  }
    
}