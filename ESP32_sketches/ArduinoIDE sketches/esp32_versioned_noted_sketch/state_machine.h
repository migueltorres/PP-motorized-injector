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
be only declared within the CASE that then later calls the general function,... example, 
ProgrammedMotorMove is called many times, with distint variable values as per State of machine, 
some States will requiere Actual values particular to this cycle, other States maybe Common values that
should never change..? However, it would be practice to have Common variables also globals as 
SOMETIMES there may be a requirement to change them expecially during testing and calibration, 
these local variables may be more convenient to have as globals, and write extra functions to 
be able to change them..?
 */



void MachineState (int runState)  // 
{
//static StatesMachine runState;  // useful here, or to make runState a Static variable between loops
                                  // maybe can add "static" to above enum declaration...?

switch (runState)
  {
    case StatesMachine::INIT_HEATING:
      delay(500);
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
      ButtonLEDsColors (YELLOW, YELLOW, YELLOW);// set button LEDs to all YELLOW, usr can now press center/up button to start Homing process, for first time
      UpButtonBounce.update();
      UpPinState = UpButtonBounce.read();
      if (UpPinState == LOW && UpButtonBounce.currentDuration() > buttonShortPress)
      {
     
      /*while (digitalRead(BUTTONUp_InjectConfirm)==LOW) // user starts to press Homing button...
        {
          int button_millis = millis;
          if ((digitalRead(BUTTONUp_InjectConfirm)==LOW)  && (button_millis >= millis+buttonShortPress))  // ... continous holding for ShortPress time...
          {  */

            HomeMove (HomingFastSpeed, HomingSlowSpeed);  // ... but once sees Homing sequence start, should let go of button..? whatif doesn't..?
            encoder.setCount(0);  // should only set Encoder to 0 when HomeMove has done RETURN (finished)
            runState=StatesMachine::INIT_HOMED_ENCODER_ZEROED;   // should only change state when HomeMove has been finished, && encoder set to 0
          }
        //}   
      
    break;

    case StatesMachine::INIT_HOMED_ENCODER_ZEROED
      // what doing here? goto Refill only..?
    break;

    case StatesMachine::REFILL:
      if (endOfDayFlag==0)
      {
        ButtonLEDsColors (GREEN, BLACK, BLACK);
      }
      else (endOfDayFlag==1)
      {
        ButtonLEDsColors (GREEN, BLUE, BLUE);
      }
      while (digitalRead(BUTTONUp_InjectConfirm)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW)
      {
        int button_millis = millis; 
        if ((digitalRead(BUTTONUp_InjectConfirm)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW) && (button_millis >= millis+buttonShortPress))
        {
          endOfDayFlag=!endOfDayFlag;
        }
      }
      while (digitalRead(BUTTONSelect_InjectCancel==LOW)
      {
        // CHECK that EndstopNOZZLEBlockForCompressOrPurge is PRESENT before allowing compression, otherwise plastic will be extruded from tip!
        int button_millis = millis;
        if (digitalRead(BUTTONSelect_InjectCancel==LOW) && (button_millis >= millis+buttonShortPress))
          {
            StatesMachine::COMPRESSION;
          }
    break;

    case StatesMachine::COMPRESSION:
      ButtonLEDsColors (RED, BLACK, RED);
      compressionFunction (initialCompressionSpeed, minCompressionSpeed);
      // compare totalStrokeSteps to 
      StatesMachine::READY_TO_INJECT;
      
    break;

    case StatesMachine::READY_TO_INJECT:
      ButtonLEDsColors (GREEN, YELLOW, GREEN)
      while (digitalRead(BUTTONUp_InjectConfirm==LOW)
      {
        int button_millis = millis;
        if (digitalRead(BUTTONUp_InjectConfirm==LOW) && (button_millis+buttonLongPress >= millis))
          {
            StatesMachine::REFILL;
          }
      }
      while (digitalRead(BUTTONSelect_InjectCancel)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW)
      {
        int button_millis = millis; 
        if ((digitalRead(BUTTONSelect_InjectCancel)==LOW) && (digitalRead(BUTTONDown_InjectConfirm)==LOW) && (button_millis+buttonShortPress >= millis))
          {
            StatesMachine::PURGE_ZERO;
          }
      }

    break;


    case StatesMachine::PURGE_ZERO:
      ButtonLEDsColors (GREEN, YELLOW, YELLOW);
      forwardPinState = digitalRead(BUTTONDown_InjectConfirm);
      backwardPinState = digitalRead(BUTTONUp_InjectConfirm);
      selectPinState = digitalRead(BUTTONSelect_InjectCancel);

      if ((forwardPinState == 0 && backwardPinState == 0) || (forwardPinState == 1 && backwardPinState == 1))   // whilst NO button press or BOTH 
      {                                                                           // button press, NO MOVE and set stepper position to 0
        stepper->stopMove();
        stepper->setCurrentPosition(0);
      }
      else if (forwardPinState == 0 && backwardPinState == 1)       // if DOWN button pressed, move continuous down until button released
      {
        ContinuousMotorMoveForward (PurgeSpeed);
      }
      else if (forwardPinState == 1 && backwardPinState == 0)    // if UP button pressed, move continuos up until button released
      {
        ContinuousMotorMoveBackward (PurgeSpeed);
      }
      else if (selectPinState == 0)
      {
        int button_millis = millis;
        if ((selectPinState == 0) && (button_millis+buttonShortPress >= millis))      // if SELECT button pressed 1/2s, go to next state 
          {                                                         // as no other buttons are pressed, stepper position is already set to 0
            StatesMachine::ANTIDRIP;
          }
      }

    break;

    case StatesMachine::ANTIDRIP:
      ButtonLEDsColors (RED, GREEN, GREEN);
      /* antiDrip funtion, or simple programmed slow backward move, to avoid drip, but with countdown, after which auto-cancel 
      possibility of continuing with Injection Process, as too much air / time allows plastic to cool in nozzle tip, so returns
      to ReadyToInject..?*/
    break;

    case StatesMachine::INJECT:
     // CHECK that EndstopNOZZLEBlockForCompressOrPurge is PRESENT before allowing compression, otherwise plastic will be extruded from tip!
    // per-mould ProgrammedMotorMove to inject with required pressure/speed during time requiered

    break;

    case StatesMachine::HOLD_INJECTION:
      // per-mould ProgrammedMotorMove to maintain pressure during time requiered
    break;

    case StatesMachine::RELEASE:
      // common ProgrammedMotorMove to release pressure and allow mould removal
      // if EndOfDayFlag==1, goto ReadyToInject, if ==0, goto Refill
    break;

    case StateMachine::CONFIRM_MOULD_REMOVAL:
      ButtonLEDsColors (GREEN, GREEN, GREEN);
      // press any button to confirm mould removal, 
      // if EndOfDayFlag==1, goto ReadyToInject, if ==0, goto Refill

    break;


  } 
}