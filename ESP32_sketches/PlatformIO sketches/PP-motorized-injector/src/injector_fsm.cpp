#include "injector_fsm.h"
#include "config.h"

void stateMachineLoop() {
    // latch inputs, here you can simulate inputs
  
    // eval states
  
    // update outputs
  }

/**
 * to transition states would also apply button activation / availablity..? or this only in machine states..? 
 * some motor moves user may want to stop before motor move is completed..? INJECT or HOLD, or COMPRESSION
 * how to apply ContinuousMotorMoves..? As in Purge, where State does not change whilst moving motor.. this function must be in Machine State..?
 * 
 */
void transitionToState(InjectorStates toState) {
    if (currentState != toState) { 
      Serial.printf("Changing from %d to %d\n", currentState, toState);
   
      if (toState == ERROR_STATE) {
          stepper->stopMove();
  
      } else if (toState == REFILL) {
          doMoveMotor = true;
          programmedMotorMove(generalFastSpeed, -30000 /*,defaultAcceletationNema*/); 
      
      } else if (toState == COMPRESSION) {
          doMoveMotor = true;
          //compressionFunction();  // FIXME  compression function currently commented out
  
      } else if (toState == INJECT) {
          doMoveMotor = true;
          programmedMotorMove(fillMouldMoveSpeed, fillMouldMoveDistSteps, fillMouldAccel); 
  
      } else if (toState == HOLD_INJECTION) {
          doMoveMotor = true;
          programmedMotorMove(holdMouldMoveSpeed, holdMouldMoveDistSteps /*, fillMouldAccel*/); // leave as default or as same as fillMouldAccel?
  
      } else if (toState == RELEASE) {
          doMoveMotor = true;
          programmedMotorMove(releaseMouldMoveSpeed, releaseMouldMoveDistSteps); // common machine action, uses default accel
  
  
  
      } else {  
        switch(currentState) {
  
          case INIT_HOMED_ENCODER_ZEROED:
            
          break;
        }
      } 
  
      currentState = toState;
    }  
  } 
  
  void machineState()  //
  {
    //static StatesMachine runState;  // useful here, or to make runState a Static variable between loops
    // maybe can add "static" to above enum declaration...?
    
    /** button activation/ availability is activted on entering new state..? if motor move is started during transtion, STOPPING move by user
     * can only be done once in new State..?
     */
  
    switch (currentState) 
    {
      case InjectorStates::ERROR_STATE:  // function that includes stopMove(), flashes LEDs red (maybe with
        //Serial.println("ERROR!! " + error);
        
        buttonLEDsColors(RED_RGB, GREEN_RGB, BLUE_RGB);  // FIXME ERROR_STATE should be all red, with flashing sequence as per Error to Identify
  
        if(selectButtonPressed)
        {
          transitionToState(InjectorStates::INIT_HEATING);
        } 
        break;
  
      case InjectorStates::INIT_HEATING:
        buttonLEDsColors(RED_RGB, RED_RGB, RED_RGB);
  
        if(downButtonPressed)
        {
          transitionToState(InjectorStates::INIT_HOT_NOT_HOMED);
        } 
        break;
  
      case InjectorStates::INIT_HOT_NOT_HOMED:
        buttonLEDsColors(YELLOW_RGB, YELLOW_RGB, YELLOW_RGB);
  
        if(downButtonPressed)
        {
          transitionToState(InjectorStates::INIT_HOMED_ENCODER_ZEROED);
        } 
        if(upButtonPressed)
        {
          transitionToState(InjectorStates::INIT_HEATING);
        } 
        break;
  
      case InjectorStates::INIT_HOMED_ENCODER_ZEROED:
        buttonLEDsColors(YELLOW_RGB, RED_RGB, YELLOW_RGB);
  
        if(downButtonPressed)
        {
          transitionToState(InjectorStates::REFILL);
        } 
        if(upButtonPressed)
        {
          transitionToState(InjectorStates::INIT_HOT_NOT_HOMED);
        } 
        break;
  
      case InjectorStates::REFILL:
        buttonLEDsColors(GREEN_RGB, BLACK_RGB, BLACK_RGB);
  
        if(downButtonPressed)
        {
          transitionToState(InjectorStates::ERROR_STATE);
        } 
        if(upButtonPressed)
        {
          transitionToState(InjectorStates::INIT_HOMED_ENCODER_ZEROED);
        } 
          break;
  
      case InjectorStates::COMPRESSION:
        buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);
        break;
  
      case InjectorStates::READY_TO_INJECT:
        buttonLEDsColors(GREEN_RGB, YELLOW_RGB, GREEN_RGB);
        break;
  
      case InjectorStates::PURGE_ZERO:
        buttonLEDsColors(GREEN_RGB, YELLOW_RGB, YELLOW_RGB);
        break;
  
      case InjectorStates::ANTIDRIP:
        buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);
        break;
      case InjectorStates::INJECT:
        buttonLEDsColors(RED_RGB, GREEN_RGB, BLACK_RGB);  // middle/UP LED GREEN flashing
        break;
      
      case InjectorStates::HOLD_INJECTION:
        buttonLEDsColors(RED_RGB, GREEN_RGB, GREEN_RGB);  // bottom/DOWN LED GREEN flashing
        break;
  
      case InjectorStates::RELEASE:
        buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);
        break;
  
      case InjectorStates::CONFIRM_MOULD_REMOVAL:
        buttonLEDsColors(GREEN_RGB, GREEN_RGB, GREEN_RGB);  
        break;
    }
  }
  
  
////////////////////
 /**
  * should run in each loop, or at least every 10ms, and if any endstop or emergency stop is triggered, stop machine
  * and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or 
  * other options..?
  * 
  * @returns InjectorError
  */
 int sanityCheck()
 {
   int error = InjectorError::NO_ERROR;

   if (nozzleTemperature <= minTempForAnyMove)      // and stop machine
   {
     error |= InjectorError::ERROR_1;
   }
   
   if (barrelEndStopActivated)
   {
     error |= InjectorError::ERROR_2;
   }

   if (topEndStopActivated)
   {
     error |= InjectorError::ERROR_3;
   }
   
   if (bottomEndStopActivated)
   {
     error |= InjectorError::ERROR_4;
   }
   
   if (emergencyStop)
   {
     error |= InjectorError::ERROR_5;
   }
// FIXME add error state when function REQUIRES something under barrel (compression, purge or mould fill)
   return error;
 }
