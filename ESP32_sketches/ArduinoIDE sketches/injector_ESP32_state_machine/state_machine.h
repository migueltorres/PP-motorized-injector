// state_machine.h

#include <Arduino.h>

// StatesMachine, states that the machine may bein at any point in time

  enum  StatesMachine 
  {
    ERROR_STATE,                     // SOME TYPE OF CHECK TRIGGERED
    INIT_HEATING,                    // INITIAL POWER ON STATES
    INIT_HOT_NOT_HOMED,              // INITIAL POWER ON STATES
    INIT_HOMED_ENCODER_ZEROED,       // INITIAL POWER ON STATES
    REFILL,                          // DEFAULT WAITING STATES
    COMPRESSION,                     // DEFAULT WAITING STATES
    READY_TO_INJECT,                 // DEFAULT WAITING STATES
    PURGE_ZERO,                      // INJECT PROCESS STATES, normally will start here...
    ANTIDRIP,                        // INJECT PROCESS STATES,
    INJECT,                          // INJECT PROCESS STATES,
    HOLD_INJECTION,                  // INJECT PROCESS STATES,
    RELEASE,                         // INJECT PROCESS STATES, 
    CONFIRM_MOULD_REMOVAL            // INJECT PROCESS STATES, ... and end here before returning to Refill 
                                     // or READY_TO_INJECT, depending on EndOfDayFlag
  };

enum StatesMachine runState   // delaring variable runState can only have valid values of enum

/* notes on MachineState...:
1) "return" in called functions will return back to this function EACH LOOP..? or ONLY when a user input 
finishes the called function..? This would cause a jump to next state..?  
1a) or the CASE statement should provide the user input to pass to the next state 

example, Purge State the user  may press the down/up buton varios times, but until presses Confirm, 
the State is the same and will not change 

to revise, that each State has an exit code line that must be completed (by user or parameter value
of sensor) before being able to change State


2) variables (passed on to called functions) that can be adjusted by user input must be global, but 
variables that will never change (except by re-writing code as are fundamentally incorrect) should 
be only declared within the FUNCTION that is later called from this general function,... example, 
ProgrammedMotorMove is called many times, with distStepsint variable values as per State of machine, so 
will require global variables, but HomeMove will ALWAYS use same (common) variables, so should be
declared within this function locally...
some States will requiere Actual values particular to this cycle, other States maybe Common values that
should never change..? However, it would be good practice to have Common variables also globals as 
SOMETIMES there may be a requirement to change them especially during testing and calibration, 
these local variables may be more convenient to have as globals, and write extra functions to 
be able to change them..? Once running alpha machine, should be able to distStepsiguish more variables
that can be made local to functions, and others that, although Common, may require occasional 
adjustment.. QUESTION: can local varibles of a function be updated via Serial writes..? Useful...
 */




void MachineState (int runState)  // 
{
//static StatesMachine runState;  // useful here, or to make runState a Static variable between loops
                                  // maybe can add "static" to above enum declaration...?

switch (runState)
  {
    case StatesMachine::ERROR_STATE:    // function that includes stopMove, flashes LEDs red (maybe with
      stepper->stopMove;
      switch (errorReason)
        {
          case errorReason=A:
            Serial.println("Temp too low for machine move, check heating system")
            ButtonLEDsColors (RED, RED, RED);
          break;

          case errorReason=B:
            Serial.println("Barrel Endstop indicates Barrel has slipped, please turn off machine, & reposition")
            ButtonLEDsColors (RED, RED, RED);
            delay(500);
            ButtonLEDsColors (0, RED, RED);
            delay(500);
          break;

          case errorReason=C:
            Serial.println("Top Endstop triggered, this should not be possibe, check endstop wiring if plungerreally has not reached max height")
            ButtonLEDsColors (RED, RED, RED);
            delay(500);
            ButtonLEDsColors (RED, 0, RED);
            delay(500);
          break;

          case errorReason=D:
            Serial.println("Bottom Endstop triggered, no more plastic left to inject, please Home or Refill")
            ButtonLEDsColors (RED, RED, RED);
            delay(500);
            ButtonLEDsColors (RED, RED, 0);
            delay(500);
          break;

          case errorReason=E:
            ProgrammedMotorMove(ReleaseMouldMoveSpeed, motorAcceleration, ReleaseMouldMoveDistSteps)
            Serial.println("Emergency Endstop triggered, please revise reason, fix, and de-activate")
            ButtonLEDsColors (RED, RED, RED);
            delay(500);
            ButtonLEDsColors (0, 0, 0);
            delay(500);
          break;
          
        }
      
    break;

    case StatesMachine::INIT_HEATING:
      delay(500);  // delay to give MAX31855 time to initialize and stabilize
      tempNozzleDegrees = thermocouple.readCelsius();
      if (tempNozzleDegrees <= minTempForAnyMove )
        {
          ButtonLEDsColors (RED, RED, RED);
          // send every 500ms the actual temp via serial
        }
      else 
        {
        runState=StatesMachine::INIT_HOT_NOT_HOMED
        }
    break;

    case StatesMachine::INIT_HOT_NOT_HOMED:
      ButtonLEDsColors (YELLOW, YELLOW, YELLOW);// set button LEDs to all YELLOW, user can now press center/up button to start Homing process, for first time
      up_button.update();
      UpPinState = up_button.read();
      if (UpPinState == LOW && up_button.currentDuration() > buttonShortPress)
      {
     
      /*while (digitalRead(BUTTONUp_InjectConfirm)==LOW) // user starts to press Homing button...
        {
          int button_millis = millis;
          if ((digitalRead(BUTTONUp_InjectConfirm)==LOW)  && (button_millis >= millis+buttonShortPress))  // ... continous holding for ShortPress time...
          {  */

            HomeMove(GeneralFastSpeed, HomingSlowSpeed);  // ... but once sees Homing sequence start, should let go of button..? whatif doesn't..?
            runState=StatesMachine::INIT_HOMED_ENCODER_ZEROED;   // should only change state when HomeMove has been finished, && encoder set to 0
          }
        //}   
      
    break;

    case StatesMachine::INIT_HOMED_ENCODER_ZEROED
      ResetEncoderZero();  // should only set Encoder to 0 when HomeMove has done RETURN (finished)
      stepper->setCurrentPosition(0);
      ProgrammedMotorMove(GeneralFastSpeed, motorAcceleration, refillOpeningOffsetDistSteps);  
      runState=StatesMachine::REFILL;
    break;

    case StatesMachine::REFILL:
      BounceButtonsUpdateCall();
      if (endOfDayFlag==0)
      {
        ButtonLEDsColors (GREEN, BLACK, BLACK);
      }
      else (endOfDayFlag==1)
      {
        ButtonLEDsColors (GREEN, BLUE, BLUE);
      }
      while (digitalRead(BUTTONUp_InjectConfirm)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW)
      // while ((up_button.isPressed) && (down_button.isPressed))     // better to use Bounce2 library methods..?
      {
        int button_millis = millis; 
        if ((digitalRead(BUTTONUp_InjectConfirm)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW) && (button_millis + buttonShortPress >= millis))
        // if ((up_button.isPressed) && (down_button.isPressed) && (button_millis + buttonShortPress >= millis))  // can combine Bounce2 methods with millis..?
                                           // would seem shorter than having to .read to a variable, then logic && variable & currentDuration of EACH button..?
        {
          endOfDayFlag=!endOfDayFlag;
        }
      }
      while (digitalRead(BUTTONSelect_InjectCancel==LOW)
      {
        nozzleBlockPresentEndstopActive=digitalRead(EndstopNOZZLEBlockForCompressOrPurge)// CHECK that EndstopNOZZLEBlockForCompressOrPurge is PRESENT before 
        if (nozzleBlockPresentEndstopActive=0)                          // allowing next State compression, otherwise plastic will be extruded from tip!
        {
        int button_millis = millis;
        if (digitalRead(BUTTONSelect_InjectCancel==LOW) && (button_millis + buttonShortPress >= millis))
          {
            // relative move from Refill position
            ProgrammedMotorMove((GeneralFastSpeed/2), (motorAcceleration/2), (heatedZoneOffsetDistSteps-refillOpeningOffsetDistSteps)); 
            runState=StatesMachine::COMPRESSION;
          }
        }
        else
        {
          Serial.print("The Nozzle block is not in place, cannot proceed until nozzle block in place")
        }
    break;

    case StatesMachine::COMPRESSION:
      ButtonLEDsColors (RED, BLACK, RED);
      compressionFunction (initialCompressionSpeed, minCompressionSpeed);
      // compare totalStrokeSteps to amountRecordedDuringCompresion, and if less than ex 80%, go back to Refill as barrel is not full - or ignore this (flag)
      runState=StatesMachine::READY_TO_INJECT;
      
    break;

    case StatesMachine::READY_TO_INJECT:
      ButtonLEDsColors (GREEN, YELLOW, GREEN)
      while (digitalRead(BUTTONUp_InjectConfirm==LOW)
      {
        int button_millis = millis;
        if (digitalRead(BUTTONUp_InjectConfirm==LOW) && (button_millis+buttonLongPress >= millis))
          {
            runState=StatesMachine::REFILL;
          }
      }
      while (digitalRead(BUTTONSelect_InjectCancel)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW)
      {
        int button_millis = millis; 
        if ((digitalRead(BUTTONSelect_InjectCancel)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW) && (button_millis+buttonShortPress >= millis))
          {
            runState=StatesMachine::PURGE_ZERO;
          }
      }

    break;


    case StatesMachine::PURGE_ZERO:
      ButtonLEDsColors (GREEN, YELLOW, YELLOW);
      down_button.update();
      up_button.update();
      select_button.update();
      DownPinState = down_button.read();
      UpPinState = up_button.read();
      SelectPinState = select_button.read();

      if ((DownPinState == 0 && UpPinState == 0) || (DownPinState == 1 && UpPinState == 1))   // whilst NO button press or BOTH 
      {                                                                           // button press, NO MOVE and set stepper position to 0
        stepper->stopMove();
        stepper->setCurrentPosition(0);
      }
      else if (DownPinState == 0 && UpPinState == 1)       // if DOWN button pressed, move continuous down until button released
      {
        ContinuousMotorMoveForward (PurgeSpeed);
      }
      else if (DownPinState == 1 && UpPinState == 0)    // if UP button pressed, move continuos up until button released
      {
        ContinuousMotorMoveBackward (PurgeSpeed);
      }
      else if (SelectPinState == 0)
      {
        int button_millis = millis;
        if ((SelectPinState == 0) && (button_millis+buttonShortPress >= millis))      // if SELECT button pressed 1/2s, go to next state 
          {                                                     // as no other buttons are pressed, stepper position is already set to 0
            stepper->setCurrentPosition(0);
            runState=StatesMachine::ANTIDRIP;
          }
      }

    break;

    case StatesMachine::ANTIDRIP:
      ButtonLEDsColors (RED, GREEN, GREEN);
      Serial.print("You have ");
      Serial.print(AntiDripReverseTime/1000);
      Serial.println("s to place mould, adjust platform, and press both Green buttons to proceed with Injection")
      /*down_button.update();
      up_button.update();
      select_button.update();
      DownPinState = down_button.read();
      UpPinState = up_button.read();
      SelectPinState = select_button.read();
      
      moved inside "while" loop so that the button presses are looped, if they are above, they will only be updated once, before "while"..?
      */

      AntiDripReverseMillis = millis();
      
      while (millis() - AntiDripReverseMillis <= AntiDripReverseTime)
      {
        ContinuousMotorMoveBackward(AntiDripReverseSpeed);
        down_button.update();
        up_button.update();
        select_button.update();
        DownPinState = down_button.read();
        UpPinState = up_button.read();
        SelectPinState = select_button.read();
      
        if ( (DownPinState == LOW && down_button.currentDuration() >= buttonShortPress) && (UpPinState == LOW && up_button.currentDuration() >= buttonShortPress))
        {
          stepper->stopMove;
          AntiDripFinalOffset = stepper->getCurrentPosition;
          runState=StatesMachine::INJECT;  // do I need a "break" or "return" here to escape the "while" and/or "if" loops..?
          break;      // this in theory will break out of highest "while" loop, therefore stopping motor and going to next State, or will
                      // only break out of second while, forcing timer to finish before advancing to next State..?
        }
        else if (SelectPinState == LOW && select_button.currentDuration() >= buttonShortPress)
        {
          stepper->stopMove;
          runState=StatesMachine::READY_TO_INJECT;
          Serial.println("Injection Process CANCELED by user, please start a new Purge/ Injection cycle")
          break;
        }
        
      } 
      runState=StatesMachine::READY_TO_INJECT;
      Serial.println("Timeout on Injection Process exceeded, please start a new Purge/ Injection cycle")
    
      /*  simple continuousMotorMoveBackwards, to avoid drip, but with countdown, after which auto-cancel (lack of user input, or
      simply finishing ContinuousMotorMoveBackward within the millis timeout in while loop), next command is goto ReadyToInject..
      as to avoid too much air / time allows plastic to cool in nozzle tip, so returns to previous State...
      if buttons are pressed within "while" loop, then new State is registered.. should add "break" to immediately stop motor move..? */

    break;

    case StatesMachine::INJECT:
      ButtonLEDsColors (RED, GREEN, 0);
      Serial.print("Injecting ");
      Serial.print(FillMouldMoveDistSteps/stepsToCM3);
      Serial.println("cm3 of plastic");

      ProgrammedMotorMove(FillMouldMoveSpeed, FillMouldAccel, (FillMouldMoveDistSteps += AntiDripFinalOffset));

      runState=StatesMachine::HOLD_INJECTION;
     /* CHECK that MOULD is PRESENT in previous State before allowing injection, otherwise plastic will be extruded from tip!
     Barrel will also move, etc
    // per-mould ProgrammedMotorMove to inject with required pressure/speed during time requiered 
    REFINEMENTS: change to include compareMotorEncoder function, to check that mould is full... actually, if using contMotorMove
    & comparison, similar to Compression function, but maybe with other Adjustment parameteres, could maybe detect some/all
    types of mould when are full..? At least 3D moulds, and the slow down could also be adjusted to act as a Hold function, so all in 1!
    */

    break;

    case StatesMachine::HOLD_INJECTION:
      ButtonLEDsColors (RED, GREEN, GREEN);
      Serial.print("Holding pressure, adding ");
      Serial.print(HoldMouldMoveDistSteps/stepsToCM3);
      Serial.println("cm3 of plastic");

      ProgrammedMotorMove(HoldMouldMoveSpeed, FillMouldAccel, HoldMouldMoveDistSteps);

      runState=StatesMachine::RELEASE;
      // per-mould ProgrammedMotorMove to maintain pressure during time requiered, see Inject function, could be a constMotorMove with Compare
      // to combine both stages... instead of a "linear" dropoff in compression speed and distance, could be first of all a new much lower HoldSpeed,
      // then continue with much lower parameters and possibly for timed amount
    break;

    case StatesMachine::RELEASE:
      ButtonLEDsColors (GREEN, GREEN, GREEN);
      Serial.print("Releasing, please lower platform and remove ");
      Serial.print(mouldNow.MouldName);
      Serial.println(" mould");

      ProgrammedMotorMove(ReleaseMouldMoveSpeed, FillMouldAccel, ReleaseMouldMoveDistSteps);

      runState=StatesMachine::CONFIRM_MOULD_REMOVAL;
      // common ProgrammedMotorMove to release pressure and allow mould removal
      // if EndOfDayFlag==1, goto ReadyToInject, if ==0, goto Refill
    break;

    case StateMachine::CONFIRM_MOULD_REMOVAL:
      ButtonLEDsColors (GREEN, GREEN, GREEN);
      down_button.update();
      up_button.update();
      select_button.update();
      DownPinState = down_button.read();
      UpPinState = up_button.read();
      SelectPinState = select_button.read();

      if (DownPinState==LOW || UpPinState==LOW || SelectPinState==LOW)
      {
        if (EndOfDayFlag=0)
        {
          HomeMove(GeneralFastSpeed/2, HomingSlowSpeed);
          stepper->setCurrentPosition(0);
          ProgrammedMotorMove(GeneralFastSpeed, motorAcceleration, refillOpeningOffsetDistSteps);  
          runState=StatesMachine::REFILL;
        }
        else if (EndOfDayFlag=1)
        {
          Serial.println("Please place nozzle block to avoid plastic drip until next use of the injector, thank you!")
          runState=StatesMachine::READY_TO_INJECT;
        }
      }
      // press any button to confirm mould removal, 
      // if EndOfDayFlag==1, goto ReadyToInject, if ==0, goto Refill

    break;


  } 
}