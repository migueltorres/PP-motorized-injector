

boolean doMoveMotor = false;
// for motor Funtion, to be re-written depenging on State to these variables
int currentMotorSpeed;         // example, for INJECT state, motorSpeed=fillMouldMoveSpeed, then call ProgrammedMotorMove
                        // should use this updated value for this next call of ProgrammedMotorMove
int currentAcceleration;  // same as above, requires each State that contemplates using ProgrammedMotorMove
                        // to have all three values, otherwise risk of using older value from previous call!
int currentMotorDistance;          // same as above, is a RELATIVE move (not moveTo!)
int MotorDir;           // direction of ANY motor move, or can this be omitted as DistSteps variables can be -ve..?
int MotorPosition;      /* reference position for next ProgrammedMotorMove, set to 0 after Purge is finished... possibly 
                        not needed as all moves should be RELATIVE, and resetting motor position does not need a variable */
int stepsMoved = 0;     // Track the number of steps moved, to compare to steps made, and also for encoder comparison


/* global common variables, different types of movement parameter storage, to be sent from ESP32 or stored 
locally as common constants between all injection/homing/compression/ etc processes..
during initial calibration of machine, will refine these values, probably from advanced access
panel on display, that will later be fixed (or advanced panel will be hidden from user until
certain combo of button presses will reveal, but these common values should not need changing
once initial machien calibration has found ideal values)
FIND IDEAL VALUES OF THESE COMMON PARAMETERS AND INITIALIZE HERE!! Could become constants, but should be changable 
under certain (testing & calibration?) purposes
*/

// int readyState=0  // states for non-moving ready states  - N/A
long int purgeSpeed = 80;                   // speed for purge move, continous move until button release  80= 1cm3/s
long int generalFastSpeed = maxSpeedLimit;  // how fast to home, continous move until reaching endstop, also for other general moves..? max /2..?
// homing params moved to Homng Function as not needed elsewhere
// int HomingDirection;      // maybe not needed, as maxHomingStepsDistSteps can be -ve..?
// int HomeOffSetDistSteps;       // once first reached HomeEndstop, how much to back off before slower approach
// int HomeOffSetAccel;      // once first reached HomeEndstop, how much Accel to back off before slower approach
// int HomeOffsetSpeed;      // once first reached HomeEndstop, how quic to back off before slower approach
// int HomingSlowSpeed;      // how slow to home, 2nd approach, continous move until reaching endstop
long int initialCompressionSpeed = generalFastSpeed / 2;                    // could be generalFastSpeed / 2..?
long int minCompressionSpeed = generalFastSpeed / 20;                       // at what speed is compression no longer useful..?
long int comparisonPercentage = 50;                                         /* EXAMPLE VALUE what % difference between sent motor steps and read encoder steps should a change 
                            in speed be commanded?
                            higher % will cuase more skipping before changing speed, lower % will cause more speed changes (o vica versa),
                            but also possibly increase probability of infinite loop, if use line comparisonPercentage *= speedReductionFactor; */
long int speedReductionFactor = 5;                                          // EXAMPLE VALUE Percentage to reduce speed when comparisonPercentage limit reached
long int antiDripReverseTime = 15000;                                       // 15s in case if using ConstMotorMove, instead of ProgrammedMotorMove, how many seconds before cancel..?
long int antiDripReverseSpeed = stepsToCML / (antiDripReverseTime / 1000);  // 80steps/15, per sec.. after purge, to recuperate and avoid drip until mould is placed - TEST
long int antiDripReverseMillis;                                             //  after purge, start millis timer
// using distance: normally the user SHOULD press INJECT button BEFORE this move is completed, but if is ProgrammedMotorMove,
// can I interrupt this move before it completes..?
long int antiDripFinalOffset;                    // once AntiDrip constMoveBack is interrupted with button press for injection, store actual motor position and sum to 0?
long int releaseMouldMoveSpeed = maxSpeedLimit;  // after hold, must raise the barrel a few mm to free mould for removal.. most likely will be a constant
long int releaseMouldMoveDistSteps = -200;       //  same as above, 200 steps is approx 0.47cm, -ve as ProgMotorMove is normally +ve downward
long int constantInjectionParams[] = {
  purgeSpeed,
  generalFastSpeed,
  initialCompressionSpeed,
  minCompressionSpeed,
  comparisonPercentage,
  speedReductionFactor,
  antiDripReverseTime,
  antiDripReverseSpeed,
  releaseMouldMoveSpeed,
  releaseMouldMoveDistSteps
};

/* array to hold message with above constant parameters, to be assigned to and returned to ESP32..?
 ={purgeSpeed, generalFastSpeed, initialCompressionSpeed, antiDripReverseSpeed, AntiDripReverseDistSteps, releaseMouldMoveSpeed, releaseMouldMoveDistSteps}
*/

/* actual injection variables, will be adjusted each time for each different mould, user will have access
via panel on display to change these values whilst calibrating a mould, and will then be stored on Display
ESP32 in Flash as array/struct of values particular to each mould
*/


/// @brief struct to hold ActualMouldParams
char mouldName;              // unique string name for stuct of mould parameters for below variables
int fillMouldMoveSpeed;      /* speed of next move, sent from ESP32, test, is possible different for 2D and 3D moulds, 
higher speeds will lead to earlier over-current errors */
int fillMouldMoveDistSteps;  /* distance to move plunger to fill mould, sent from ESP32 at the start of each injection, as 
per mould in use */
int fillMouldAccel;          // possibly not needed, have to discover whether small 2D/2.5D moulds benefit from slower accel on fill..?

int holdMouldMoveSpeed;      /* ... and how fast (will be a low speed, test.. this will have to translate into a timed move,
for example has to last 15s to allow 25g of plastic to cool enough, then distSteps * speed has to take 15s.
make formula that can take time requiered, distance to move, and divide to get speed OR use ContMove for Time..? */
int holdMouldMoveDistSteps;  // after mould fill, apply a little more move/pressure to get good surface finish
int holdMouldAcccel; // as this will be a very slow speed and small distance motor move, either use fillMouldAccel, or leave as default..?
struct actualMouldParams {
  const char* mouldName;
  int fillMouldMoveSpeed;
  int fillMouldAccel;
  int fillMouldMoveDistSteps;
  int holdMouldMoveSpeed;
  int holdMouldMoveDistSteps;
} ;


// FIXME commented out for testing
//struct actualMouldParams mouldNow = { "Posavasos", maxSpeedLimit, motorAcceleration, 2160, (maxSpeedLimit / 10), 80 };
/* struct to hold received message with above variable parameters... can I include in {variable name1, variable name2, etc}..?
={fillMouldMoveSpeed, fillMouldMoveDistSteps, holdMouldMoveSpeed, holdMouldMoveDistSteps}, better practice would be struct including name
of mould and list of valies ofr the variables above, which would then be written to these variables ... include check of 
message (5 comma separated values with special starting char?) length */



// comms between Arduino & ESP32
char messageToDisplay;    /* message character to be sent to display, some letters mean errors, some to display text, some as first message
to second message that contain parameter arrays for variables or constants (for confirmation on screen) */
int messageArrayToESP32;  // parameter array, can be either variable array or constants array, depending on previous message..?

// encoder & position data from FAS library
long oldFA0Position = 0;         // encoder postion on motor FAS zeroing - maybe not need, revise ComparisonFunction
long actualFAPosition;           // getPosition from zeroed FastAccel library, to compare to Encoder data, and to send to ESP32 for display
unsigned long oldENPosition;     // for referencing encoder positions..
unsigned long actualENPosition;  // same for encoder data.. if cannot zero encoder data (so far only zeros on Arduino reset), then also make
                                 // oldENPosition, and subract to actualOldENPosition on FA zeroing, before substituting old with actual, should
                                 // give same value since last zero of FA



// endstop states, active or not, other checks to relay via Serial. mostly active LOW, with PULLUP (exceptions as per endstop)
bool EMERGENCYStopPinActive = 0;              // is Emergency Stop true, then cannot allow movement - consider wiring also direct to stepper motor wires
                                           // but ONLY if sure that breaking this connection will not damage driver.. otherwise, also wire to driver PSU
bool topPlungerEndstopActive = 0;          // is 1/NC, then can move up, if 0/NC, then triggered
bool bottomPlungerEndstopActive = 0;       // is 1/NC, then can move down, if 0/NC, then triggered
bool barrelClampOKEndstopActive = 0;       // is 0/NO, then can move inject, if 1/NC, then triggered, barrel has moved!!
bool mouldPresentEndstopActive = 0;        // still to see how to implement, to assure that nozzle is blocked by either mould or
bool nozzleBlockPresentEndstopActive = 0;  // purge bar/cap, so can move down w/o displacing barrel
bool endOfDayFlag = 0;                     // 0 = refillAfterInject, 1= skip and, possibly, measure steps since last top endstop - could read from encoder.. as FA library
                                           // is contemplated to be reset after each purge, so if ==0, reset encoder, ==1 do not reset encoder
bool initialHomingDone = 0;                // on power on, machine needs homing, then mark this flag to show has been done at least once
int errorReason;


// lower keypad button states, could be bool?  Or just use directly digitalRead of pin in any arguments/functions..?
bool DownPinState;    // var for purge function, read from BUTTON_DOWN_PIN pin
bool UpPinState;      // var for purge function, read from BUTTON_UP_PIN pin
bool SelectPinState;  // var for purge function, read from BUTTON_SELECT_PIN pin

//  LED stuff
int ledLBrightness = 50;       // initial brightness for LEDs, to possible increase to 100 during an actual press..?
int ledHBrightness = 200;      // pressed key brightness for LEDs, to increase to 200 (of 255 max) during an actual press..?
const int keypadLedCount = 3;  // 3 for just keypad buttons
const int ringLedCount = 35;  // 35 for LED ring on end of barrel
#define GREEN_RGB  ((uint32_t)0x008000)
#define RED_RGB ((uint32_t)0xFF0000)
#define YELLOW_RGB  ((uint32_t)0xFF8C00)
#define BLUE_RGB  ((uint32_t)0x0000FF)
#define BLACK_RGB ((uint32_t)0x000000)
#define WHITE_RGB  ((uint32_t)0xFFFFFF)



// libraries to include

// FIXME commented out for testing
//#include <RotaryEncoderPCNT.h>
//RotaryEncoderPCNT encoder(ENCODER_A_PIN, ENCODER_B_PIN);


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


//#include <ACS712.h> // current sensor, may not be needed, but added 2 to motor phase B + & - just in case
Adafruit_MAX31855 thermocouple(TEMPNozzleVSPI_SCK_CLK, TEMPNozzleVSPI_Dpin_MOSI_CS, TEMPNozzleVSPI_MISO_DO);

Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(keypadLedCount, WS2812B_BUTTON_LEDS_PIN);
Adafruit_NeoPixel ringleds = Adafruit_NeoPixel(ringLedCount, WS2812B_RING_LEDS_PIN);









/*
  // functions declarations, to try and organize most commonly re-used functions
  long EncoderActualPosition();                                             // reads encoder position and returns REFERENCE variable for use by other functions
  void ResetEncoderZero();                                                 // resets Encoder to Zero, ONLY to be called ONCE at power up, toggle initialHomingDone
  void ProgrammedMotorMove(motorSpeed, motorAcceleration, motorMove);      // simply moves motor at set speed, accel and distance indicated
  void ContinuousMotorMoveForward(motorSpeed);                             // simply moves plunger continously (during button press or other condition is 0/1)
  void ContinuousMotorMoveBackward(motorSpeed);                            // simply moves plunger continously (during button press or other condition is 0/1)
  void HomeMove(generalFastSpeed, HomingSlowSpeed);                                 // simply sends plunger to TopEndstop - CAN USE ContinuousMotorMoveBackward
  bool compareMotorEncoder();                                              // function to compare Encoder & FastAccel position, to detect when skipping steps, 
and therefore compression of plastic or filling of mould, and to reduce speed/distance commands until are below min threshold,
when then returns "complete"/"1" to function where has been called from 
  void compressionFunction(initialCompressionSpeed, minCompressionSpeed);  // ChatGPT written compression uses compareMotorEncoder funtion
  void clearLEDs();                                                        // clears LEDs and turns off, before next colour setting
  void ButtonLEDsColors(selectLEDcolour, upLEDcolour, downLEDcolour);      // sets colours of LEDs behind buttons
  void SerialRegular100msMessages();                                       // every 100ms, send via serial the Temp, Encoder position, etc
  void SerialChangeOfState();                                              // serial print when a change of state happens, and new state
  void SerialPrintNextInjectionValues();                                   // serial print the parameters of next injection
  void BounceButtonsSetup();                                               // setup of buttons list to avoid setup() overload
  void BounceButtonsUpdateCall();                                          // same for button update call in loop()
  int CheckMinTempEndstopsEmergencyStopPin();                                // should run in each loop, and if any endstop or emergency stop is triggered, stop machine 
and report which endstop has been triggered... possible thereafter only allow moves in opposite direction, or back to refill, or 
other options..? */

  //////////////////////////////////

  // functions fully described

  /**
   * Encoder PCNT library use should mean that encoder value is callable at any time (will always give back 
   * current position, so really only needed at start of relative motor moves, to check next move positions) 
   * Encoder function will get called not every loop, but maybe every 10ms, to give time for max distances to trigger other code, and every 100ms
   * wil be called by SerialRegular100msMessages, where if motor is stopped, actual = old, and no need to print value
   */
  long EncoderActualPosition() {

// FIXME commented out for testing
    //actualENPosition = encoder.getCount() / 2;  // revise last int, if motor steps = 400 & encoder PPR = 1000, adjust this int
                                                // (possible issue of needing to round, this is bad, better maybe to increase driver steps to same as encoder?)
    oldENPosition = actualENPosition;           // if this does NOT change between Serial.prints, then don't print

    return (actualENPosition);
  }

  /**
   * 
   */
  void ResetEncoderZero() {

// FIXME commented out for testing
 //   encoder.clearCount();

    return;
  }

/////////////////////////////////////
// motor functions

/** 
 * NOTE: When calling this function from different states, it will be called with the arguments particular to that state, ex.
 * INJECT will pass ProgrammedMotorMove(fillMouldMoveSpeed, fillMouldAccel, fillMouldMoveDistSteps)... will this AUTOMATICALLY write those 
 * parameters to the functions named paramters, ie motorSpeed, motorAcceleration, motorMove..? Because if I wish to do a later
 * compareMotorEncoder function, this function needs to have the ProgrammedMotorMove NAMED params, not the PASSED params names..?
 * 
 * @param motorSpeed The speed...
 * @param motorAcceleration Accelerattion...
 * @param motorMove Move the motor forward or backward
 * 
 * NOTE ABOUT using shortened simplier function calls: as expressed below, can call programmedMotorMove with 2 (speed & distance) or 3 (w/ accel as well) parameters...
 * when using 2 parameteres, then the accel parameter will be auto-added with the "default" setting from this function definition, AFAIK! example:
 * programmedMotorMove(1000,2000); speed & steps only, programmedMotorMove (1000,2000,3000); full control including acceleration
 * 
 * REVISE ALL CALLS OF programmedMotorMove IN OTHER FUNCTIONS, THIS HAS CHANGED THE ORDER OF THE PARAMETERS, PREVIOUSLY ACCEL WAS SECOND AND NOW IS THIRD! 
 * DO ***NOT*** WANT TO MIX UP OLD ACCELERATION PARAMTER WITH NEW DISTANCE PARAMETER
*/
  void programmedMotorMove(int motorSpeed, int motorDistance, int motorAcceleration = defaultAcceletationNema)  // RELATIVE motor move to new position
  {
    if (doMoveMotor)
    { 
      doMoveMotor = false;

      currentMotorSpeed = motorSpeed;
      currentAcceleration = motorAcceleration;
      currentMotorDistance = motorDistance;

      stepper->setSpeedInHz(motorSpeed);
      stepper->setAcceleration(motorAcceleration);
      stepper->move(motorDistance);  // <- RELATIVE move, motor position should be zeroed, and then move is simply set from function variable, 
                                    // or could be set from encoder position and ABSOLUTE move (moveTo) used, but ADDING function variable to actual position
    } 
  }

  /**
   * CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
   */
  void continuousMotorMoveForward(int motorSpeed)
  {
    currentMotorSpeed = motorSpeed;

    stepper->setSpeedInHz(motorSpeed);
    stepper->runForward();
  }

  /**
   * CONTINUOUS motor move IF/WHILE a condition is met (button press, !endStop trigger, etc)
   */
  void continuousMotorMoveBackward(int motorSpeed)
  {
    currentMotorSpeed = motorSpeed;

    stepper->setSpeedInHz(motorSpeed);
    stepper->runBackward();
  }

  int HomingSlowSpeed = maxSpeedLimit / 10;  // how slow to home, 2nd approach, continous move until reaching endstop

  /**
   * FIXME
   * once called, no user action can stop until complete!
   */
  // void homeMove(int generalFastSpeed, int homingSlowSpeed)
  // {
  //   //int HomingDirection;      // maybe not needed, as using ContMoveBackwards
  //   int homeOffSetDistSteps = 212;             // once first reached HomeEndstop, how much to back off before slower approach, 212 steps ≈ 5mm
  //   int homeOffSetAccel = 10000;               // once first reached HomeEndstop, how much Accel to back off before slower approach 10000 = 1/5th normal
  //   int homeOffsetSpeed = maxSpeedLimit / 2;   // once first reached HomeEndstop, how quick to back off before slower approach
  //   int homingSlowSpeed = maxSpeedLimit / 10;  // how slow to home, 2nd approach, continous move until reaching endstop
    
  //   if (digitalRead(ENDSTOP_TOP_PLUNGER_PIN) == !topPlungerEndstopActive) {
  //     continuousMotorMoveBackward(generalFastSpeed);  // OR ProgrammedMotorMove with maxHomingStepsDistSteps, if fails, gives error?
  //   }

  //   programmedMotorMove(homeOffsetSpeed, homeOffSetDistSteps, homeOffSetAccel);  // programmed motor move assumed to be positive DistSteps..?
  //    FIXED: ACCEL LAST PARAMETER OK    
  //   if (digitalRead(ENDSTOP_TOP_PLUNGER_PIN) == !topPlungerEndstopActive) {
  //     continuousMotorMoveBackward(homingSlowSpeed);
  //   }
  // }

  // function to compare Motor & Encoder positions
  bool compareMotorEncoder(int motorMove, int actualENPosition, int comparisonPercentage)  // chatGPT written
  {
    // Calculate difference between motor position and encoder position
    int motorError = abs(motorMove - actualENPosition);

    // Calculate the acceptable difference threshold
    int threshold = (motorMove * comparisonPercentage) / 100;

    // Return true if the difference is greater than the threshold
    return (motorError >= threshold);
  }

//   /**
//    * FIXME
//    * motor compression function takes compareMotorEncoder and motor moves to apply pressure up to a threshold and then
//    * reduce the speed (increases torque of stepper?) until again threshold past, etc, until minCompressionSpeed reached,
//    *  thenwill stop
//    */
//   void compressionFunction(int initialCompressionSpeed, int minCompressionSpeed)  // chatGPT written
//   {
//     buttonLEDsColors(RED_RGB, BLACK_RGB, RED_RGB);
//     select_button.update();

//     // add ButtonSelect interrupt function on user press, goes to Ready_to_Inject (also EStop..? or EStop better permament in loop
//     //  and then goes to ERROR-STATE?)
//     if (!select_button.isPressed()) {
//       // Set initial speed
//       stepper->setSpeedInHz(initialCompressionSpeed);  // Starting speed in steps per second

//       while (stepper->getSpeedInMilliHz()/1000 > minCompressionSpeed) {
//         // Move the plunger down
//         stepper->runForward();


// // FIXME commented out for testing
//         // // Call the comparison function to check if motor-encoder difference exceeds threshold
//         // if (compareMotorEncoder(stepper->targetPos(), encoder.getPosition(), comparisonPercentage)) {
//         //   // Reduce speed when the difference is greater than the allowed threshold
//         //   int newSpeed = stepper->getSpeedInMilliHz() / 1000 * speedReductionFactor / 10;
//         //   stepper->setSpeedInHz(newSpeed);
//         //   buttonLEDsColors(RED_RGB, BLACK_RGB, YELLOW_RGB);  // to show compression is acting (has reduced speed at least once) and ongoing..

//         //   // Optionally, reduce comparison threshold on each iteration (for finer control)
//         //   comparisonPercentage *= speedReductionFactor; /* could lead to infinite smaller steps if minCompressionSpeed is too low...? 
//         //   for debugging this option, would be good to show actual speed - if later wish to add another compressionDuringHeating function
//         //   where the plastic is already quite compressed, and will start (and end) from/to a much lower speeds, have to make sure this 
//         //   does not produce infinite loop as never reaches minCompressionSpeed */
//         // }
//       }

//       // Stop motor when speed falls below threshold
//       stepper->stopMove();
//       buttonLEDsColors(RED_RGB, BLACK_RGB, GREEN_RGB);  // to show compression is complete
//       delay(500);                           // ONLY DELAY IN ENTIRE SKETCH, and is only to give time for GREENrgb LED to be shown before

//       return;  // once finished function should return to function that called it, be it compression or inject.. should be break..?
//     } else {
//       stepper->stopMove();
//       InjectorStates::READY_TO_INJECT;
//     }
//   }


  void clearLEDs() {
    for (int i = 0; i < keypadLedCount; i++) {
      keypadleds.setPixelColor(i, 0);
    }
  }


  boolean changeSelectLEDcolour;
  boolean changeUpLEDcolour;
  boolean changeDownLEDcolour;

  uint32_t currentSelectLEDcolour;
  uint32_t currentUpLEDcolour;
  uint32_t currentDownLEDcolour;

  /**
   * 
   */
  void buttonLEDsColors(uint32_t newSelectLEDcolour, uint32_t newUpLEDcolour, uint32_t newDownLEDcolour)  //could use case/switch instead? but are different types of defining behaviours..?
  {
    changeSelectLEDcolour = currentSelectLEDcolour != newSelectLEDcolour;
    if (changeSelectLEDcolour) {
      currentSelectLEDcolour = newSelectLEDcolour;
    }  

    changeUpLEDcolour = currentUpLEDcolour != newUpLEDcolour;
    if (changeUpLEDcolour) {
      currentUpLEDcolour = newUpLEDcolour;
    }  

    changeDownLEDcolour = currentDownLEDcolour != newDownLEDcolour;
    if (changeDownLEDcolour) {
      currentDownLEDcolour = newDownLEDcolour;
    }  
  }

  /**
   * 
   */
  void outputButtonLEDsColors()  //could use case/switch instead? but are different types of defining behaviours..?
  {
    if (selectButtonPressed  || upButtonPressed || downButtonPressed)  // set brightness to about 80% on any button press
    {
      keypadleds.setBrightness(ledHBrightness);
    } else {
      keypadleds.setBrightness(ledLBrightness);
    }  

    if (changeSelectLEDcolour) { 
      keypadleds.setPixelColor(0, currentSelectLEDcolour);
      changeSelectLEDcolour = false;
    }

    if (changeUpLEDcolour) { 
      keypadleds.setPixelColor(1, currentUpLEDcolour);
      changeUpLEDcolour = false;
    }

    if (changeDownLEDcolour) { 
      keypadleds.setPixelColor(2, currentDownLEDcolour);
      changeDownLEDcolour = false;
    }

    keypadleds.show();

    return;
  }



  void serialPrintNextInjectionValues() {
    Serial.println("Next injection values:");
    // add here struct of variablePerMouldParams

    return;
  }



  void loop() {
    long startTime = millis();

    getInputs();


    machineState();

    // output
    outputButtonLEDsColors();

    // debug
    serialRegular100msMessages();

    long endTime = millis();

    long elapsedTime = endTime - startTime;
    // 

  }


