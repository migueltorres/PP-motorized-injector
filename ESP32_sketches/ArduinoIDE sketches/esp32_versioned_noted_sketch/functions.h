
// functions declarations, to try and organize most commonly re-used functions
void ProgrammedMotorMove (motorSpeed, motorAcceleration, motorMoveTo);  // simply moves motor @ seet speed, accel and distance indicated
int EncoderActualPosition ();                       // reads encoder position and returns REFERENCE variable for use by other functions
void ContinuousMotorMoveForward (motorSpeed);       // simply moves plunger continously (during button press or other condition is 0/1)
void ContinuousMotorMoveBackward (motorSpeed);      // simply moves plunger continously (during button press or other condition is 0/1)
void HomeMove (motorSpeed, runDirection);           // simply sends plunger to TopEndstop - CAN USE ContinuousMotorMoveBackward
void SerialRegularMessages ();                      // every X ms, send via serial the Temp, Encoder position, MachineState, etc
bool compareMotorEncoder ();                    /* function to compare Encoder & FastAccel position, to detect when skipping steps, 
and therefore compression of plastic or filling of mould, and to reduce speed/distance commands until are below min threshold,
when then returns "complete"/"1" to function where has been called from */
void compressionFunction(initialCompressionSpeed, minCompressionSpeed);   // ChatGPT written compression uses compareMotorEncoder funtion
void CheckMinTempEndstopsEmergencyStop ();      /* should run in each loop, and if any endstop or emergency stop is triggered, stop machine 
and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or 
other options..? */
void clearLEDs();                                 // clears LEDs and turns off, before next colour setting
void ButtonLEDsColors (upLEDcolour, downLEDcolour, selectLEDcolour); // sets colours of LEDs behind buttons


//////////////////////////////////

// functions fully described
void EncoderActualPosition()
{
  long actualENPosition = encoder.getCount() / 2;  // revise last int, if motor steps = 400 & encoder PPR = 1000, adjust this int
                                                   // (possible issue of needing to round, this is bad, better maybe to increase driver steps to same as encoder?)

  return;
}

void ProgrammedMotorMove (motorSpeed, motorAcceleration, motorMoveTo)
{
  stepper->setSpeedInHz(motorSpeed);
  stepper->setAcceleration(motorAcceleration);
  stepper->moveTo(motorMoveTo);

  return;
}

void ContinuousMotorMoveForward (motorSpeed)
{
  stepper->setSpeedInHz(motorSpeed);
  stepper->runForward(); 
  
  return; 
}

void ContinuousMotorMoveBackward (motorSpeed)
{
  stepper->setSpeedInHz(motorSpeed);
  stepper->runBackward();  

  return;
}


void HomeMove (HomingFastSpeed, HomingSlowSpeed);
{
  if (digitalRead(EndstopTOPPlunger)==!topPlungerEndstopActive)
  {
    ContinuousMotorMoveBackward (HomingFastSpeed);
  }
  ProgrammedMotorMove (HomeOffsetSpeed, HomeOffSetAccel, HomeOffSetDist);
  if (digitalRead(EndstopTOPPlunger)==!topPlungerEndstopActive)
  {
    ContinuousMotorMoveBackward (HomingSlowSpeed);
  } 
  return;
}


void SerialRegularMessages()
{
  // put millis timer and messages to be sent via Serial here
}


bool compareMotorEncoder(int targetSteps, int actualEncoderPosition, float percentageDifference) // chatGPT written
{
  // Calculate difference between motor position and encoder position
  int motorError = abs(targetSteps - actualEncoderPosition);
    
  // Calculate the acceptable difference threshold
  int threshold = (targetSteps * percentageDifference) / 100;
    
  // Return true if the difference is greater than the threshold
  return (motorError >= threshold);
}


void compressionFunction(initialCompressionSpeed, minCompressionSpeed)  // chatGPT written
{
    int initialSpeed = initialCompressionSpeed;  // Starting speed in steps per second
    int minSpeed = minCompressionSpeed;       // Minimum speed threshold
    float speedReductionFactor = 0.5;  // Percentage to reduce speed when limit reached
    float comparisonPercentage = 50.0; // Initial percentage difference to trigger speed reduction
    ButtonLEDsColors (RED, BLACK, RED);

// add ButtonSelect interrupt function on user press (also EStop..? or EStop better permament in loop
//  and then goes to ERROR-STATE?)

    // Set initial speed
    stepper->setSpeedInHz(initialSpeed);

    while (stepper->getSpeed() > minSpeed) {
        // Move the plunger down
        stepper->runForward();

        // Call the comparison function to check if motor-encoder difference exceeds threshold
        if (compareMotorEncoder(stepper->getTargetPosition(), encoder.getPosition(), comparisonPercentage)) {
            // Reduce speed when the difference is greater than the allowed threshold
            int newSpeed = stepper->getSpeed() * speedReductionFactor;
            stepper->setSpeedInHz(newSpeed);
            //add here a keypadleds.setPixelColor(2, YELLOW) that changes to YELLOW and/or flashes faster the last (down) LED
            // to show compression is acting (has reduced speed at least once) and ongoing..

            // Optionally, reduce comparison threshold on each iteration (for finer control) 
            comparisonPercentage *= speedReductionFactor;  // could lead to infinite smaller steps if minspeed is too low...?
        }
    }
    
    // Stop motor when speed falls below threshold
    stepper->stopMove();
    //add here a keypadleds.setPixelColor(2, GREEN) to show compression is complete
    return; // once finished function should return to function that called it, be it cmopression or inject.. should be break..?
}


void CheckMinTempEndstopsEmergencyStop()
{
  // put checks here to stop machine in case of any triggers
}


void clearLEDs()
{
  for (int i=0; i<ledCount; i++)
  {
    keypadleds.setBrightness(50);
    keypadleds.setPixelColor(i, 0);

  return;
  }
}


void ButtonLEDsColors (upLEDcolour, downLEDcolour, selectLEDcolour);   //could use case/switch instead? but are different types of defining behaviours..?
{    
  keypadleds.setBrightness(50);   // set brightness to about 50% (maybe less, revise)
  keypadleds.setPixelColor(0, upLEDcolour);
  keypadleds.setPixelColor(1, downLEDcolour);
  keypadleds.setPixelColor(2, selectLEDcolour);
  keypadleds.show(); 

  return;
}
