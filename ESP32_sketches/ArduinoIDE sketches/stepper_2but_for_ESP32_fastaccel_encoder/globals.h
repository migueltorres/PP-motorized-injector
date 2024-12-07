#ifndef globals_h
#define globals_h



#define CLK 34 // CLK ENCODER 
#define DT 35 // DT ENCODER 

#define dirPinStepper    22
//#define enablePinStepper 6
#define stepPinStepper   23  // OC1A in case of AVR
#define downPin 27 
#define upPin 26
#define selectPin 25

#define KEYS_LED_PIN 33
#define KEYS_LED_COUNT 3

#define GREEN			0x008000
#define RED			0xFF0000
#define YELLOW			0xFF8C00  //0xFF4500 orangey yellow
#define YELLOW2     0xFF8C00  // 0xFF8C00 nicest yellow so far
#define RED_rgb   255,0,0
#define GREEN_rgb  0,255,0
#define BLUE_rgb  0,0,255

#define TOP_ENDSTOP_PIN 19
#define BOTTOM_ENDSTOP_PIN 18
#define BARREL_ENDSTOP_PIN 5
#define EMERGENCY_STOP_PIN 0

#define DEBOUNCE_INTERVAL 5


int runSpeed;
int runMove;
int runState=0;
const int maxSpeedLimit = 10000;
const int moveDist=5000;
bool downPinState;
bool upPinState;
bool selectPinState;
int potReading =500;
int motorSpeed;
int old_position;
int position;
unsigned int motor_def_accel = 30000;



#endif