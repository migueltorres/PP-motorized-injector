
/* 

bounce_inductive_tests_to_LEDs.ino

 DESCRIPTION
 ====================
 This is an example of the Bounce2::Button class. 
 When the user presses a physical button, it toggles a LED on or off.
 The Button class matches an electrical state to a physical action. 
 Use .setPressedState(LOW or HIGH) to set the detection state for when the button is pressed.


 INSCRUCTIONS
 ====================

 Set BUTTON_PIN to the pin attached to the button.
 Set LED_PIN to the pin attached to a LED.
 
 */

// WE WILL attach() THE BUTTON TO THE FOLLOWING PIN IN setup()
#define TOP_ENDSTOP_PIN 19
#define BOTTOM_ENDSTOP_PIN 18
#define BARREL_ENDSTOP_PIN 5
#define EMERGENCY_STOP_PIN 0

#define DEBOUNCE_INTERVAL 5

// DEFINE THE PIN FOR THE LED :
// 1) SOME BOARDS HAVE A DEFAULT LED (LED_BUILTIN)
//#define LED_PIN LED_BUILTIN
// 2) OTHERWISE SET YOUR OWN PIN
#define PIN 33
#define LED_COUNT 3

// define some colors
#define GREEN 0x008000
#define RED 0xFF0000
#define YELLOW 0xFF8C00   //0xFF4500 orangey yellow
#define YELLOW2 0xFF8C00  // 0xFF8C00 nicest yellow so far
#define RED_rgb 255, 0, 0
#define GREEN_rgb 0, 255, 0
#define BLUE_rgb 0, 0, 255


// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button TOP_ENDSTOP = Bounce2::Button();
Bounce2::Button BOTTOM_ENDSTOP = Bounce2::Button();
Bounce2::Button BARREL_ENDSTOP = Bounce2::Button();
Bounce2::Button EMERGENCY_STOP = Bounce2::Button();

// Create an instance of the Adafruit_NeoPixel class called "leds".
Adafruit_NeoPixel keypadleds(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);


void setup() {

  // BUTTON SETUP

  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR BUTTON HAS AN INTERNAL PULL-UP
  TOP_ENDSTOP.attach(TOP_ENDSTOP_PIN, INPUT_PULLUP);
  BOTTOM_ENDSTOP.attach(BOTTOM_ENDSTOP_PIN, INPUT_PULLUP);    //Top & Bottom NO, 5v
  BARREL_ENDSTOP.attach(BARREL_ENDSTOP_PIN, INPUT_PULLDOWN);  //FINDA sensor is NC, 0v
  EMERGENCY_STOP.attach(EMERGENCY_STOP_PIN, INPUT_PULLUP);    // CHECK THAT IS NO!!

  // DEBOUNCE INTERVAL IN MILLISECONDS
  TOP_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  BOTTOM_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  BARREL_ENDSTOP.interval(DEBOUNCE_INTERVAL);
  EMERGENCY_STOP.interval(DEBOUNCE_INTERVAL);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  TOP_ENDSTOP.setPressedState(LOW);  // NO, with pullup, goes LOW when closed
  BOTTOM_ENDSTOP.setPressedState(LOW);
  BARREL_ENDSTOP.setPressedState(HIGH);  //  NC, with pulldown, goes HIGH when active
  EMERGENCY_STOP.setPressedState(LOW);   //  CHECK THAT IS NO!!

  // LED SETUP
  keypadleds.begin();            // Call this to start up the LED strip.
                                 //  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.show();             // ...but the LEDs don't actually update until you call this.
  keypadleds.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
}

void clearLEDs() {
  for (int i = 0; i < LED_COUNT; i++) {
    keypadleds.setPixelColor(i, 0);
  }
}



void loop() {
  // UPDATE THE BUTTON
  // YOU MUST CALL THIS EVERY LOOP
  BARREL_ENDSTOP.update();
  TOP_ENDSTOP.update();
  BOTTOM_ENDSTOP.update();
  
  EMERGENCY_STOP.update();

  // <Button>.pressed() RETURNS true IF THE STATE CHANGED
  // AND THE CURRENT STATE MATCHES <Button>.setPressedState(<HIGH or LOW>);
  // WHICH IS LOW IN THIS EXAMPLE AS SET WITH button.setPressedState(LOW); IN setup()
  if (BARREL_ENDSTOP.isPressed()) {
    // TOGGLE THE LED STATE :
    keypadleds.setPixelColor(0, RED);
    keypadleds.show();
  }
  else 
  {
    keypadleds.setPixelColor(0, 0);
    keypadleds.show();
  }

  if (TOP_ENDSTOP.isPressed()) {
    // TOGGLE THE LED STATE :
    keypadleds.setPixelColor(1, RED);
    keypadleds.show();
  }
  else 
  {
    keypadleds.setPixelColor(1, 0);
    keypadleds.show();
  }

  if (BOTTOM_ENDSTOP.isPressed()) {
    // TOGGLE THE LED STATE :
    keypadleds.setPixelColor(2, RED);
    keypadleds.show();
  }
  else 
  {
    keypadleds.setPixelColor(2, 0);
    keypadleds.show();
  }


  if (EMERGENCY_STOP.isPressed()) {
    // TOGGLE THE LED STATE :
    keypadleds.setPixelColor(0, RED);
    keypadleds.setPixelColor(1, RED);
    keypadleds.setPixelColor(2, RED);
    keypadleds.show();
  }
  else 
  {
    keypadleds.setPixelColor(0, 0);
    keypadleds.setPixelColor(1, 0);
    keypadleds.setPixelColor(2, 0);
    keypadleds.show();
  }  
}
