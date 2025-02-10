/* global varible definitions & declaractions, some may be moved to local funtions, but 
leave here and comment out where they have gone - QUESTION: can LOCAL variables within functions be changed via 
serial message updates..? ex: compressionFunction has local speedReductionFactor & comparisonPercentage, during 
initial sketch & machine calibration, it may be necessary to update these values, would be easier via Serial
than flashing...? Although in theory, once checked and sure cannot cause infinite loop, should again be purely 
local and never need changing
 */


 // pin definitions, decided on from "GPIO_Limitations_ESP32_NodeMCU.jpg", and which side may be more convenient
 
 #define ENCODER_A_PIN 34                           // * A+ encoder pin, A- not used  "CLK ENCODER "
 #define ENCODER_B_PIN 35                           // * B+ encoder pin, B- not used  "DT ENCODER "
 #define WS2812B_RING_LEDS_PIN  32            // 1 pin for 35 LEDs nozzle ring, to change color as per moment of use..
 #define WS2812B_BUTTON_LEDS_PIN 33         // * 1 pin to address 3 LEDs buttons...
 #define BUTTON_SELECT_PIN 25                            // * these 3 pins for lower keypad, to facilite Purge and INject Confirm
 #define BUTTON_UP_PIN 26                      // *
 #define BUTTON_DOWN_PIN 27                         // *
 #define TEMPNozzleVSPI_SCK_CLK 14               //  *  18 (possible change to analog component if run out of pins
 #define TEMPNozzleVSPI_MISO_DO 12              // * 19
 #define TEMPNozzleVSPI_Dpin_MOSI_CS 13         // * 23 these 3 pins are for daughter PCB thermocouple
 
 //////////////////
 
 #define STEPPER_STEP_PIN 23                        // * any GPIO for ESP32
 #define STEPPER_DIR_PIN 22                         // * may not be needed if using -ve moves..?
 
 #define ENDSTOP_TOP_PLUNGER_PIN 19                     // * all buttons/endstops with INPUT_PULLUP to avoid floating values
 #define ENDSTOP_BOTTOM_PLUNGER_PIN 18                  // * so all ACTIVE STATES are ==0
 #define ENDSTOP_BARREL_PLUNGER_PIN 5                   // *
 #define UART_tx_ESP32 17                         // * TX2 UART comms with ESP32
 #define UART_rx_ESP32 16                         // *
 //#define CURRENTSensor1   4      // * as yet to be decided on how to use motor phase current readng
 #define EMERGENCY_STOP_PIN 0                           //  *
 //#define CURRENTSensor2   2      // * to be useful for detecting motor current (reaching amc compression/fill)
 #define ENDSTOP_NOZZLE_BLOCK_COMPRESS_OR_PURGE_PIN 15  // also as above, not decided hardware method yet
 #define EndstopMOULDPresentPin                    // do not yet have a way of detecting, possibly pressure sensor under plastic floor on platform..?
 
 //////////////////////////////////
 
 // global constants
 // revise what is now NOT NEEDED, OR what variables should go inside Functions to make LOCAL
 
 /* global common variables for machine limits and moves to offsets, these, once identified the max values
 should NEVER need changing, even from advanced Common panel, as superior values run the risk of machine
 performing outside of the decided security limits of movement/accel, distances, etc
 revise what is  NOT NEEDED, OR what variables should go inside Functions to make LOCAL*/
 const int minTempForAnyMove = 20; // FIXME              // min temp considered safe for machine move
 const int maxSpeedLimit = 2000;                 // 2000 corresponds to 5rps NEMA, about 1/4 rps gear, about 18.85/4 = 4.7125cm linear
                                                 // plunger movement, about 25cm3 injected volume, per second MAX speed limited
 const int moveContinuosDistSteps = 5000;        // arbitrary large distance for continuous movements, gets reset to 0 each loop anyway
 const int defaultAcceletationNema = 50000;      /* check in reality, maybe large moulds will benefit from slower acceleration together with higher speeds..?
                                             to avoid blocking motor/losing steps with high speed requests with too high accelerrations..
                                            or smaller moulds benefit also from slower acceleration/speed so as not to explode, or fill with
                                            less acceleration tapering off... */
 const int maxHomingStepsDistSteps = 30000;      // CHECK THIS NUMBER OF STEPS COVERS WHOLE MOVEMENT RANGE, HEATED ZONE + REFILL + TOPENDSTOP + extra
 const int totalStrokeStepsDistSteps = 21220;    //calc'd value, check against real, this is ONLY HEATED ZONE, need another OFFSET from Encoder 0
 const int refillOpeningOffsetDistSteps = 3800;  // ESTIMATED! once homed to top endstop, how far to move to position just above filling hole in barrel
                                                 // THIS VALUE HAS TO BE FOUND BY CALIBRATION AND ADJUSTED TO REALITY!!
 const int heatedZoneOffsetDistSteps = 7200;     // calc'd value, check with reality
 const int buttonShortPress = 500;               // for short press of button, ms
 const int buttonLongPress = 2000;               // long press...
 const int debounceInterval = 5;
 
 
 ////////////////////
 /* steps to cm3 & cml conversion 
volumen    80steps = 1cm3,  2000steps = 25cm3  (average 2D mould approx)
linear    424steps = 1cml, 21220steps = 50cml  (total useful heating zone, below Refill offset)
*/
int stepsToCM3 = 80;
int stepsToCML = 424;
