#ifndef globals_h
#define globals_h

#include <stdint.h>
#include <string.h>

/* global varible definitions & declaractions, some may be moved to local funtions, but
leave here and comment out where they have gone - QUESTION: can LOCAL variables within functions be changed via
serial message updates..? ex: compressionFunction has local speedReductionFactor & comparisonPercentage, during
initial sketch & machine calibration, it may be necessary to update these values, would be easier via Serial
than flashing...? Although in theory, once checked and sure cannot cause infinite loop, should again be purely
local and never need changing
 */

// pin definitions, decided on from "GPIO_Limitations_ESP32_NodeMCU.jpg", and which side may be more convenient

#define UART_TX_ESP32 17              //  TX2 UART comms with ESP32
#define UART_RX_ESP32 16              // 
#define ENCODER_A_PIN 34              //  A+ encoder pin, A- not used  "CLK ENCODER "
#define ENCODER_B_PIN 35              //  B+ encoder pin, B- not used  "DT ENCODER "
#define STEPPER_STEP_PIN 23           //  any GPIO for ESP32
#define STEPPER_DIR_PIN 22            //  may not be needed if using -ve moves..?
#define ENDSTOP_TOP_PLUNGER 19        //  all buttons/endstops with INPUT_PULLUP to avoid floating values
#define ENDSTOP_BOTTOM_PLUNGER 18     //  so all ACTIVE STATES are ==0
#define ENDSTOP_BARREL_CLAMP_OK 5     // 
//#define ENDSTOP_MOULD_PRESENT 21      // do not yet have a way of detecting, possibly pressure sensor under plastic floor on platform..?
#define ENDSTOP_NOZZLE_BLOCK 15       // also as above, not decided hardware method yet
#define TEMP_NOZZLE_VSPI_MISO_DO 12 //  19
#define TEMP_NOZZLE_VSPI_MOSI_CS 13 //  23 these 3 pins are for daughter PCB thermocouple
#define TEMP_NOZZLE_VSPI_SCK_CLK 14  //    18 (possible change to analog component if run out of pins
#define BUTTON_SELECT_PIN 25          //  these 3 pins for lower keypad, to facilite Purge and INject Confirm
#define BUTTON_UP_PIN 26              // 
#define BUTTON_DOWN_PIN 27            //
#define WS2812B_BUTTON_LEDS 33        // * 1 pin to address 3 LEDs buttons...
#define WS2812B_RING_LEDS 32           // 1 pin for 35 LEDs nozzle ring, to change color as per moment of use..
#define EMERGENCY_STOP 0              //  *
// #define CURRENTSensor1   4      // * as yet to be decided on how to use motor phase current readng
// #define CURRENTSensor2   2      // * to be useful for detecting motor current (reaching max compression/fill)

//////////////////////////////////

// global constants
// revise what is now NOT NEEDED, OR what variables should go inside Functions to make LOCAL

/* global common variables for machine limits and moves to offsets, these, once identified the max values
should NEVER need changing, even from advanced Common panel, as superior values run the risk of machine
performing outside of the decided security limits of movement/accel, distances, etc
revise what is  NOT NEEDED, OR what variables should go inside Functions to make LOCAL*/
extern const uint8_t MIN_TEMP_FOR_MOVE = 170;          // min temp considered safe for machine move
extern const uint16_t MAX_SPEED_LIMIT = 2000;          // 2000 corresponds to 5rps NEMA, about 1/4 rps gear, about 18.85/4 = 4.7125cm linear
                                                       // plunger movement, about 25cm3 injected volume, per second MAX speed limited
extern const uint16_t moveContinuosDistSteps = 5000;   // arbitrary large distance for continuous movements, gets reset to 0 each loop anyway
extern const uint16_t DEFAULT_ACCELERATION = 50000;    /* check in reality, maybe large moulds will benefit from slower acceleration together with higher speeds..?
                                               to avoid blocking motor/losing steps with high speed requests with too high accelerrations..
                                              or smaller moulds benefit also from slower acceleration/speed so as not to explode, or fill with
                                              less acceleration tapering off... */
extern const uint16_t MAX_HOMING_STEPS = 30000;        // CHECK THIS NUMBER OF STEPS COVERS WHOLE MOVEMENT RANGE, HEATED ZONE + REFILL + TOPENDSTOP + extra
extern const uint16_t TOTAL_STROKE_STEPS = 21220;      // calc'd value, check against real, this is ONLY HEATED ZONE, need another OFFSET from Encoder 0
extern const uint16_t REFILL_OFFSET_STEPS = 3800;      // ESTIMATED! once homed to top endstop, how far to move to position just above filling hole in barrel
                                                       // THIS VALUE HAS TO BE FOUND BY CALIBRATION AND ADJUSTED TO REALITY!!
extern const uint16_t HEATED_ZONE_OFFSET_STEPS = 7200; // calc'd value, check with reality
extern const uint16_t BUTTON_SHORT_PRESS_MS = 500;     // for short press of button, ms
extern const uint16_t BUTTON_LONG_PRESS_MS = 2000;     // long press...
extern const uint8_t DEBOUNCE_INTERVAL_MS = 5;

/* steps to cm3 & cml conversion
volumen    80steps = 1cm3,  2000steps = 25cm3  (average 2D mould approx)
linear    424steps = 1cml, 21220steps = 50cml  (total useful heating zone, below Refill offset)
*/
extern uint8_t STEPS_TO_CM3 = 80;
extern uint16_t STEPS_TO_CML = 424;

// for motor Funtion, to be re-written depenging on State to these variables
extern uint16_t motorSpeed;        // example, for INJECT state, motorSpeed=FillMouldMoveSpeed, then call ProgrammedMotorMove
                                   // should use this updated value for this next call of ProgrammedMotorMove
extern uint16_t motorAcceleration; // same as above, requires each State that contemplates using ProgrammedMotorMove
                                   // to have all three values, otherwise risk of using older value from previous call!
extern int16_t motorMove;          // same as above, is a RELATIVE move (not moveTo!)
extern int8_t MotorDir;            // direction of ANY motor move, or can this be omitted as DistSteps variables can be -ve..?
extern int16_t MotorPosition;      /* reference position for next ProgrammedMotorMove, set to 0 after Purge is finished... possibly
                        not needed as all moves should be RELATIVE, and resetting motor position does not need a variable */
extern int16_t stepsMoved = 0;     // Track the number of steps moved, to compare to steps made, and also for encoder comparison

/* global common variables, different types of movement parameter storage, to be sent from ESP32 or stored
locally as common constants between all injection/homing/compression/ etc processes..
during initial calibration of machine, will refine these values, probably from advanced access
panel on display, that will later be fixed (or advanced panel will be hidden from user until
certain combo of button presses will reveal, but these common values should not need changing
once initial machien calibration has found ideal values)
FIND IDEAL VALUES OF THESE COMMON PARAMETERS AND INITIALIZE HERE!! Could become constants, but should be changable
under certain (testing & calibration?) purposes
*/
// extern uint16_t runState = 0; // states for movement functions (purge, inject, etc), 0 to ensure when no button is pressed, no movement
// extern uint16_t readyState=0  // states for non-moving ready states  - N/A
extern uint16_t PurgeSpeed = 80;                    // speed for purge move, continous move until button release  80= 1cm3/s
extern uint16_t GeneralFastSpeed = MAX_SPEED_LIMIT; // how fast to home, continous move until reaching endstop, also for other general moves..? max /2..?
// homing params moved to Homng Function as not needed elsewhere
// extern uint16_t HomingDirection;      // maybe not needed, as maxHomingStepsDistSteps can be -ve..?
// extern uint16_t HomeOffSetDistSteps;       // once first reached HomeEndstop, how much to back off before slower approach
// extern uint16_t HomeOffSetAccel;      // once first reached HomeEndstop, how much Accel to back off before slower approach
// extern uint16_t HomeOffsetSpeed;      // once first reached HomeEndstop, how quic to back off before slower approach
// extern uint16_t HomingSlowSpeed;      // how slow to home, 2nd approach, continous move until reaching endstop
extern uint16_t initialCompressionSpeed = GeneralFastSpeed / 2;                     // could be GeneralFastSpeed / 2..?
extern uint16_t minCompressionSpeed = GeneralFastSpeed / 20;                        // at what speed is compression no longer useful..?
extern uint8_t comparisonPercentage = 50;                                           /* EXAMPLE VALUE what % difference between sent motor steps and read encoder steps should a change
                                                                    in speed be commanded?
                                                                    higher % will cuase more skipping before changing speed, lower % will cause more speed changes (o vica versa),
                                                                    but also possibly increase probability of infinite loop, if use line comparisonPercentage *= speedReductionFactor; */
extern float speedReductionFactor = 0.5;                                            // EXAMPLE VALUE Percentage to reduce speed when comparisonPercentage limit reached
extern uint32_t AntiDripReverseTime = 15000;                                        // 15s in case if using ConstMotorMove, instead of ProgrammedMotorMove, how many seconds before cancel..?
extern uint32_t AntiDripReverseMillis;                                              //  after purge, start millis timer
extern uint16_t AntiDripReverseSpeed = STEPS_TO_CML / (AntiDripReverseTime / 1000); // 80steps/15, per sec.. after purge, to recuperate and avoid drip until mould is placed - TEST
// using distance: normally the user SHOULD press INJECT button BEFORE this move is completed, but if is ProgrammedMotorMove,
// can I interrupt this move before it completes..?
extern uint16_t AntiDripFinalOffset;                     // once AntiDrip constMoveBack is interrupted with button press for injection, store actual motor position and sum to 0?
extern uint16_t ReleaseMouldMoveSpeed = MAX_SPEED_LIMIT; // after hold, must raise the barrel a few mm to free mould for removal.. most likely will be a constant
extern int16_t ReleaseMouldMoveDistSteps = -200;         //  same as above, 200 steps is approx 0.47cm, -ve as ProgMotorMove is normally +ve downward
struct constantInjectionParams
{
    uint16_t PurgeSpeed;
    uint16_t GeneralFastSpeed;
    uint16_t initialCompressionSpeed;
    uint16_t minCompressionSpeed;
    uint8_t comparisonPercentage;
    float speedReductionFactor;
    uint32_t AntiDripReverseTime;
    uint16_t AntiDripReverseSpeed;
    uint16_t ReleaseMouldMoveSpeed;
    int16_t ReleaseMouldMoveDistSteps;
};
/* struct to hold message with above constant parameters, to be assigned to and returned to ESP32..?
={PurgeSpeed, GeneralFastSpeed, initialCompressionSpeed, AntiDripReverseSpeed, AntiDripReverseDistSteps, ReleaseMouldMoveSpeed, ReleaseMouldMoveDistSteps}
*/

/* actual injection variables, will be adjusted each time for each different mould, user will have access
via panel on display to change these values whilst calibrating a mould, and will then be stored on Display
ESP32 in Flash as array/struct of values particular to each mould
*/

extern String MouldName;                // unique string name for stuct of mould parameters for below variables
extern uint16_t FillMouldMoveSpeed;     /* speed of next move, sent from ESP32, test, is possible different for 2D and 3D moulds,
    higher speeds will lead to earlier over-current errors */
extern uint16_t FillMouldAccel;         // possibly not needed, have to discover whether small 2D/2.5D moulds benefit from slower accel on fill..?
extern uint16_t FillMouldMoveDistSteps; /* distance to move plunger to fill mould, sent from ESP32 at the start of each injection, as
per mould in use */
extern uint16_t HoldMouldMoveSpeed;     /* ... and how fast (will be a low speed, test.. this will have to translate into a timed move,
    for example has to last 15s to allow 25g of plastic to cool enough, then distSteps * speed has to take 15s.
    make formula that can take time requiered, distance to move, and divide to get speed OR use ContMove for Time..? */
extern uint16_t HoldMouldMoveDistSteps; // after mould fill, apply a little more move/pressure to get good surface finish
struct actualMouldParams
{
    String MouldName;
    uint16_t FillMouldMoveSpeed;
    uint16_t FillMouldAccel;
    uint16_t FillMouldMoveDistSteps;
    uint16_t HoldMouldMoveSpeed;
    uint16_t HoldMouldMoveDistSteps;
} mouldNow;

actualMouldParams mouldNow = {"Posavasos", MAX_SPEED_LIMIT, motorAcceleration, 2160, (MAX_SPEED_LIMIT / 10), 80};
/* struct to hold received message with above variable parameters... can I include in {variable name1, variable name2, etc}..?
={FillMouldMoveSpeed, FillMouldMoveDistSteps, HoldMouldMoveSpeed, HoldMouldMoveDistSteps}, better practice would be struct including name
of mould and list of valies ofr the variables above, which would then be written to these variables ... include check of
message (5 comma separated values with special starting char?) length */

extern char messageToDisplay; /* message character to be sent to display, some letters mean errors, some to display text, some as first message
to second message that contain parameter arrays for variables or constants (for confirmation on screen) */
String messageArrayToESP32;   // parameter array, can be either variable array or constants array, depending on previous message..?

// encoder & position data from FAS library
extern int32_t oldFA0Position = 0; // encoder postion on motor FAS zeroing - maybe not need, revise ComparisonFunction
extern int32_t actualFAPosition;   // getPosition from zeroed FastAccel library, to compare to Encoder data, and to send to ESP32 for display
extern int32_t oldENPosition;      // for referencing encoder positions..
extern int32_t actualENPosition;   // same for encoder data.. if cannot zero encoder data (so far only zeros on Arduino reset), then also make
                                   // oldENPosition, and subract to actualOldENPosition on FA zeroing, before substituting old with actual, should
                                   // give same value since last zero of FA

// endstop states, active or not, other checks to relay via Serial. mostly active LOW, with PULLUP (exceptions as per endstop)
extern uint16_t tempNozzleDegrees;               // var to fill from temp sensor, to compare to a min temp to allow any type of movement
extern bool EMERGENCYStopActive = 0;             // is Emergency Stop true, then cannot allow movement - consider wiring also direct to stepper motor wires
                                                 // but ONLY if sure that breaking this connection will not damage driver.. otherwise, also wire to driver PSU
extern bool topPlungerEndstopActive = 0;         // is 1/NC, then can move up, if 0/NC, then triggered
extern bool bottomPlungerEndstopActive = 0;      // is 1/NC, then can move down, if 0/NC, then triggered
extern bool barrelClampOKEndstopActive = 1;      // is 0/NO, then can move inject, if 1/NC, then triggered, barrel has moved!!
extern bool mouldPresentEndstopActive = 0;       // still to see how to implement, to assure that nozzle is blocked by either mould or
extern bool nozzleBlockPresentEndstopActive = 0; // purge bar/cap, so can move down w/o displacing barrel
extern bool endOfDayFlag = 0;                    // 0 = refillAfterInject, 1= skip and, possibly, measure steps since last top endstop - could read from encoder.. as FA library
                                                 // is contemplated to be reset after each purge, so if ==0, reset encoder, ==1 do not reset encoder
extern bool initialHomingDone = 0;               // on power on, machine needs homing, then mark this flag to show has been done at least once
extern char errorReason;

// lower keypad button states, could be bool?  Or just use directly digitalRead of pin in any arguments/functions..?
extern bool DownPinState;   // var for purge function, read from BUTTONDownPin pin
extern bool UpPinState;     // var for purge function, read from BUTTONUpPin pin
extern bool SelectPinState; // var for purge function, read from BUTTONSelectPin pin

//  LED stuff
extern uint8_t ledLBrightness = 50;      // initial brightness for LEDs, to possible increase to 100 during an actual press..?
extern uint8_t ledHBrightness = 200;     // pressed key brightness for LEDs, to increase to 200 (of 255 max) during an actual press..?
extern const uint8_t buttonLedCount = 3; // 3 for just keypad buttons, +36 for LED ring on end of barrel!
#define GREEN_rgb 0, 128, 0
#define RED_rgb 255, 0, 0
#define YELLOW_rgb 255, 140, 0
#define BLUE_rgb 0, 0, 255
#define BLACK 0, 0, 0

#endif