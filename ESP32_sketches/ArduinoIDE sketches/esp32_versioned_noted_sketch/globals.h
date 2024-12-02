//  COPIED FROM NANO SKETCH!! "*" in descripcion HAS BEEN MODIFIED FOR ESP32
//  COPIED FROM NANO SKETCH!! "*" in descripcion HAS BEEN MODIFIED FOR ESP32
//  COPIED FROM NANO SKETCH!! "*" in descripcion HAS BEEN MODIFIED FOR ESP32

// pin definitions

#define UART_tx_ESP32 17   // * TX2 UART comms with ESP32
#define UART_rx_ESP32 16   // *
#define ENCODERApin 34     // * A+ encoder pin, A- not used  "CLK ENCODER "
#define ENCODERBpin 35     // * B+ encoder pin, B- not used  "DT ENCODER "
#define EndstopTOPPlunger  19     // * all buttons/endstops with INPUT_PULLUP to avoid floating values
#define EndstopBOTTOMPlunger  18  // * so all ACTIVE STATES are ==0
#define EndstopBARRELClampOK  5  // *   
//#define EndstopMOULDPresent  21 // NOT IN USE YET
#define EndstopNOZZLEBlockForCompressOrPurge 15  
#define StepperSTEPPin   23  // * any GPIO for ESP32
#define StepperDIRPin    22  // * may not be needed if using -ve moves..?
#define TEMPNozzleVSPI_MISO_DO 12  // *
#define TEMPNozzleVSPI_Dpin_MOSI_CS 13  // * these 3 pins are for daughter PCB thermocouple 
#define TEMPNozzleVSPI_SCK_CLK 14   //  *  (possible change to analog component if run out of pins
#define BUTTONDownPin 27  // * these 3 pins for lower keypad, to facilite Purge and INject Confirm 
#define BUTTONUpPin 26    // *
#define BUTTONSelectPin 25 // *
#define WS2812B_ButtonAddressableLEDs 33  // * 1 pin to address 3 LEDs buttons... 
#define WS2812B_RingAddressableLEDs 32  // 1 pin for 35 LEDs nozzle ring, to change color as per moment of use.. 
#define EMERGENCYStop  0       //      Boot failure if LOW on BOOT, so assure IS/NOT PRESSED on BOOT..?
#define CURRENTSensor1   4      // *
#define CURRENTSensor2   2      // *

//////////////////////////////////

// global common variables
// revise what is now NOT NEEDED, OR what variables should go inside Functions to make LOCAL


// for motor Funtion, to be re-written depenging on State to these variables
int motorSpeed;   // example, for INJECT state, motorSpeed=FillMouldMoveSpeed, then call ProgrammedMotorMove
                  // should use this updated value for this next call of ProgrammedMotorMove
int motorAcceleration;  // same as above, requires each State that contemplates using ProgrammedMotorMove
                        // to have all three values, otherwise risk of using older value from previous call!
int motorMoveTo;      // same as above


const int readMillisTime = 1000;    // time in millis for temp readings, maybe for other stuff too..?

//  LED stuff
int ledBrightness = 50;           // initial brightness for LEDs, to possible increase to 100 during an actual press..?
const int ledCount = 3;          // 3 for just keypad buttons, +36 for LED ring on end of barrel!
const char GREEN =	 0x008000;
const char RED	=		 0xFF0000;
const char YELLOW	=	 0xFF8C00; 
const char BLUE =  0x001F;
const char BLACK = 0x0000;


/* global common variables for machine limits and moves to offsets, these, once identified the max values
should NEVER need changing, even from advanced Common panel, as superior values run the risk of machine
performing outside of the decided security limits of movement/accel, distances, etc
revise what is  NOT NEEDED, OR what variables should go inside Functions to make LOCAL*/
const int minTempForAnyMove = 170;  // min temp considered safe for machine move
const int maxSpeedLimit = 2000; // 2000 corresponds to 5rps NEMA, about 1/4 rps gear, about 18.85/4 = 4.7125cm linear
                                // plunger movement, about 25cm3 injected volume, per second MAX speed limited
const int moveContinuosDist=5000;    // arbitrary large distance for continuous movements, gets reset to 0 each loop anyway
const int defaultAcceletationNema = 50000; // check in reality, maybe large moulds will benefit from slower acceleration together with higher speeds..?
                                           // to avoid blocking motor/losing steps with high speed requests with too high accelerrations..
const int maxHomingStepsDist = 30000;  // CHECK THIS NUMBER OF STEPS COVERS WHOLE MOVEMENT RANGE, HEATED ZONE + REFILL + TOPENDSTOP + extra
const int totalStrokeStepsDist = 21220  //calc'd value, check against real, this is ONLY HEATED ZONE, need another OFFSET from Encoder 0
const int refillOpeningOffsetDist=2000;  // once homed to top endstop, how far to move to position just above filling hole in barrel
                // THIS VALUE HAS TO BE FOUND BY CALIBRATION AND ADJUSTED TO REALITY!!
const int heatedZoneOffsetDist = 8500  // calc'd value, check with reality
const int buttonShortPress = 500  // for short press of button, ms
const int buttonLongPress = 2000  // long press...
int stepsMoved = 0;          // Track the number of steps moved, to compare to steps made, and also for encoder comparison



/* global common variables, different types of movement parameter storage, to be sent from ESP32 or stored 
locally as common constants between all injection/homing/compression/ etc processes..
during initial calibration of machine, will refine these values, probably from advanced access
panel on display, that will later be fixed (or advanced panel will be hidden from user until
certain combo of button presses will reveal, but these common values should not need changing
once initial machien calibration has found ideal values)
*/
int runState=0;  // states for movement functions (purge, inject, etc), 0 to ensure when no button is pressed, no movement
int readyState=0  // states for non-moving ready states 
int PurgeSpeed;   // speed for purge move, continous move until button release
int GeneralFastSpeed;      // how fast to home, continous move until reaching endstop, also for other general moves..?
int HomingDirection;      // maybe not needed, as maxHomingStepsDist can be -ve..?
int HomeOffSetDist;       // once first reached HomeEndstop, how much to back off before slower approach
int HomeOffSetAccel;      // once first reached HomeEndstop, how much Accel to back off before slower approach
int HomeOffsetSpeed;      // once first reached HomeEndstop, how quic to back off before slower approach
int HomingSlowSpeed;      // how slow to home, 2nd approach, continous move until reaching endstop
int initialCompressionSpeed;  // could be GeneralFastSpeed..?
int minCompressionSpeed;
int AntiDripReverseSpeed; // after purge, to recuperate and avoid drip until mould is placed
int AntiDripReverseDist;  // after purge, to recuperate and avoid drip how far to move before too much (air intro'd)
int ReleaseMouldMoveSpeed; // after hold, must raise the barrel a few mm to free mould for removal.. most likely will be a constant
int ReleaseMouldMoveDist; //  same as above
int constantInjectionParams[];  // array to hold message with above constant parameters, to be assigned to and returned to ESP32..?
                              // ={PurgeSpeed, GeneralFastSpeed, AntiDripReverseSpeed, AntiDripReverseDist, ReleaseMouldMoveSpeed, ReleaseMouldMoveDist}


/* actual injection variables, will be adjusted each time for each different mould, user will have access
via panel on display to change these values whilst calibrating a mould, and will then be stored on Display
ESP32 in Flash as array/struct of values particular to each mould
*/
int MotorDir;           // direction of ANY motor move, or can this be omitted as Dist variables can be -ve..?
int MotorPosition;      // reference position for next ProgrammedMotorMove, set to 0 after Purge is finished
int FillMouldMoveSpeed; // speed of next move, sent from ESP32,  test, is possible different for 2D and 3D moulds, 
                        // higher speeds will lead to earlier over-current errors
int FillMouldMoveDist;  // distance to move plunger to fill mould, sent from ESP32 at the start of each injection, as
                        // per mould in use
int HoldMouldMoveSpeed; // ... and how fast (will be a low speed, test.. this will have to translate into a timed move,
                        // for example has to last 15s to allow 25g of plastic to cool enough, then dist * speed has to take 15s.
                        // make formula that can take time requiered, distance to move, and divide to get speed
int HoldMouldMoveDist;  // after mould fill, apply a little more move/pressure to get good surface finish
int variablePerMouldParams[];  // array to hold received message with above variable parameters... can I include in {variable name1, variable name2, etc}..?
                              // ={FillMouldMoveSpeed, FillMouldMoveDist, HoldMouldMoveSpeed, HoldMouldMoveDist}
int constantInjectionParams[];  // array to hold message with above constant parameters, to be assigned to and returned to ESP32..?
                              // ={PurgeSpeed, GeneralFastSpeed, AntiDripReverseSpeed, AntiDripReverseDist, ReleaseMouldMoveSpeed, ReleaseMouldMoveDist}


// comms between Arduino & ESP32
char messageToDisplay;  // message character to be sent to display, some letters mean errors, some to display text, some as first message
                        // to second message that contain parameter arrays for variables or constants (for confirmation on screen)
int messageArrayToESP32;  // parameter array, can be either variable array or constants array, depending on previous message..?

// encoder & position data from FAS library
int actualFAPosition;   // getPosition from zeroed FastAccel library, to compare to Encoder data, and to send to ESP32 for display
unsigned long actualENPosition;   // same for encoder data.. if cannot zero encoder data (so far only zeros on Arduino reset), then also make 
                        // oldENPosition, and subract to actualOldENPosition on FA zeroing, before substituting old with actual, should
                        // give same value since last zero of FA
long oldPosition = -999;  // encoder initialization postion


// lower keypad button states, could be bool?  Or just use directly digitalRead of pin in any arguments..?
bool DownPinState;    // var for purge function, read from BUTTONDownPin pin
bool UpPinState;    // var for purge function, read from BUTTONUpPin pin
bool SelectPinState;   // var for purge function, read from BUTTONSelectPin pin

// endstop states, active or not, other checks to relay via Serial
int tempNozzleDegrees;  // var to fill from temp sensor, to compare to a min temp to allow any type of movement
bool EMERGENCYStopActive;  // is Emergency Stop true, then cannot allow movement - consider wiring also direct to stepper motor wires
                         // but ONLY if sure that breaking this connection will not damage driver.. otherwise, also wire to driver PSU
bool topPlungerEndstopActive;  // if 1/NC, then can move up, if 0/NC, then triggered
bool bottomPlungerEndstopActive; // if 1/NC, then can move down, if 0/NC, then triggered
bool barrelClampOKEndstopActive; // if 1/NC, then can move inject, if 0/NC, then triggered, barrel has moved!!
bool mouldPresentEndstopActive; // still to see how to implement, to assure that nozzle is blocked by either mould or
bool nozzleBlockPresentEndstopActive;  // purge bar/cap, so can move down w/o displacing barrel
bool endOfDayFlag=0; // 0 = refillAfterInject, 1= skip and, possibly, measure steps since last top endstop - could read from encoder.. as FA library 
                    // is contemplated to be reset after each purge, so if ==0, reset encoder, ==1 do not reset encoder
bool initialHomingDone=0; // on power on, machine needs homing, then mark this flag to show has been done at least once

// not needed..?
int currentSensorValue; // read from currentSensor, use to create comparisons with max current (test to calibrate), to stop, slow down,
                        // reverse, or other actuations depending on current of driver (see MoveSpeedRatioToCurrent function idea)


////////////////////////////////////


// libraries to include

#include <ESP32Encoder.h> // * https://github.com/madhephaestus/ESP32Encoder.git 
#include <FastAccelStepper.h>  // *
#include <elapsedMillis.h>
//#include <ACS712.h> // current sensor, may not be needed, but added 2 to motor phase B + & - just in case
#include <SPI.h>
#include <Adafruit_MAX31855.h>  // temp board, was enclosed with quotes "" in example, not sure why
#include <Adafruit_NeoPixel.h>  // ws2812B RGB addressable LEDs


// END COPIED FROM NANO SKETCH!! "*" in descripcion HAS BEEN MODIFIED FOR ESP32
// END COPIED FROM NANO SKETCH!! "*" in descripcion HAS BEEN MODIFIED FOR ESP32
// END COPIED FROM NANO SKETCH!! "*" in descripcion HAS BEEN MODIFIED FOR ESP32