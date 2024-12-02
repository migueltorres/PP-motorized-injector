/*   
stepper_2but_for_ESP32_fastaccel.ino
 */
 
#define CLK 34 // CLK ENCODER 
#define DT 35 // DT ENCODER 

#define dirPinStepper    22
//#define enablePinStepper 6
#define stepPinStepper   23  // OC1A in case of AVR
#define downPin 27 
#define upPin 26


int runSpeed;
int runMove;
int runState=0;
const int maxSpeedLimit = 10000;
const int moveDist=5000;
int downPinState;
int upPinState;
int potReading =500;
int motorSpeed;
int old_position;
int position;

#include <RotaryEncoderPCNT.h>
#include <FastAccelStepper.h>
#include <elapsedMillis.h>

RotaryEncoderPCNT encoder(CLK, DT);


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

elapsedMillis printTime;

void setup() 
{
  Serial.begin(115200);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);

old_position = encoder.position();
Serial.println(old_position);


  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) 
  {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setSpeedInHz(runSpeed);  // the parameter is Hz, steps/s !!!
    stepper->setAcceleration(50000);
    stepper->move(moveDist);
  }
}

void executeMotorCommand(int8_t runState)
{
  switch (runState)
  {
    case 0:
//	  stepper1.setAcceleration(200);
      stepper->stopMove();
	    motorSpeed = 0;
	    //stepper->setCurrentPosition(0);
      break;
    case 1:
	    stepper->setSpeedInHz(runSpeed);
	    stepper->runForward();
      break;
    case -1:
	    stepper->setSpeedInHz(runSpeed);
	    stepper->runBackward();
      break;
  }
}


void loop() 
{
  position = encoder.position()/10;
  if (/*(runState!=runState) ||*/ (printTime >= 200))
   {
    Serial.print("Down: ");
    Serial.print(downPinState);
    Serial.print(", Up: ");
    Serial.print(upPinState);
    Serial.print(", rS: ");
    Serial.print(runState);
    Serial.print(", Sp: ");
    Serial.print(runSpeed);
    Serial.print(", En: ");
    Serial.print(position);
    Serial.print(", FAS: ");
    Serial.println(stepper->getCurrentPosition());
   /* Serial.print(" ");
    Serial.print("D");
    Serial.print(" ");
    Serial.println(stepper1.distanceToGo());*/
    printTime=0;
   }
  downPinState = digitalRead(downPin);
  upPinState = digitalRead(upPin);

  if ((downPinState == 0 && upPinState == 0) || (downPinState == 1 && upPinState == 1)) 
  {
    runState = 0;
    executeMotorCommand(runState);
  }
  else if (downPinState == 0 && upPinState == 1)
  {
    runState = 1;
    executeMotorCommand(runState);
  }
  else if (downPinState == 1 && upPinState == 0)
  {
    runState = -1;
    executeMotorCommand(runState);
  }

  motorSpeed =  map(potReading, 0, 1023, 5, maxSpeedLimit);
  runSpeed = (motorSpeed); 
}