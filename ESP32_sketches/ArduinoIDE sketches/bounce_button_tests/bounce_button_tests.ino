
/* 
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
#define DOWN_BUTTON_PIN 21 
#define UP_BUTTON_PIN 22
#define SELECT_BUTTON_PIN 5  

#define DEBOUNCE_INTERVAL 5

// DEFINE THE PIN FOR THE LED :
// 1) SOME BOARDS HAVE A DEFAULT LED (LED_BUILTIN)
//#define LED_PIN LED_BUILTIN
// 2) OTHERWISE SET YOUR OWN PIN
#define PIN 25
#define LED_COUNT 3

// define some colors
#define GREEN			0x008000
#define RED			0xFF0000
#define YELLOW			0xFF8C00  //0xFF4500 orangey yellow
#define YELLOW2     0xFF8C00  // 0xFF8C00 nicest yellow so far
#define RED_rgb   255,0,0
#define GREEN_rgb  0,255,0
#define BLUE_rgb  0,0,255


// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button down_button = Bounce2::Button();
Bounce2::Button up_button = Bounce2::Button();
Bounce2::Button select_button = Bounce2::Button();

// Create an instance of the Adafruit_NeoPixel class called "leds".
Adafruit_NeoPixel keypadleds (LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);


void setup() {

  // BUTTON SETUP 
  
  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR BUTTON HAS AN INTERNAL PULL-UP
  down_button.attach( DOWN_BUTTON_PIN, INPUT_PULLUP);
  up_button.attach( UP_BUTTON_PIN ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  select_button.attach( SELECT_BUTTON_PIN, INPUT_PULLUP );

  // DEBOUNCE INTERVAL IN MILLISECONDS
  down_button.interval(DEBOUNCE_INTERVAL); 
  up_button.interval(DEBOUNCE_INTERVAL); 
  select_button.interval(DEBOUNCE_INTERVAL); 

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  down_button.setPressedState(LOW); 
  up_button.setPressedState(LOW); 
  select_button.setPressedState(LOW); 
  
  // LED SETUP
  keypadleds.begin();  // Call this to start up the LED strip.
//  clearLEDs();   // This function, defined below, turns all LEDs off...
  keypadleds.show();   // ...but the LEDs don't actually update until you call this.
  keypadleds.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void clearLEDs()
{
  for (int i=0; i<LED_COUNT; i++)
  {
    keypadleds.setPixelColor(i, 0);
  }
}



void loop() {
  // UPDATE THE BUTTON
  // YOU MUST CALL THIS EVERY LOOP
  down_button.update();
  up_button.update();
  select_button.update();

  // <Button>.pressed() RETURNS true IF THE STATE CHANGED
  // AND THE CURRENT STATE MATCHES <Button>.setPressedState(<HIGH or LOW>);
  // WHICH IS LOW IN THIS EXAMPLE AS SET WITH button.setPressedState(LOW); IN setup()
  if ( down_button.isPressed() ) 
  {
    // TOGGLE THE LED STATE : 
    keypadleds.setPixelColor(2,GREEN);
    keypadleds.show();
  }
  else 
  {
    keypadleds.setPixelColor(2, 0);
    keypadleds.show();
  }

  if ( up_button.isPressed() ) 
  {
    // TOGGLE THE LED STATE : 
    keypadleds.setPixelColor(1,RED);
    keypadleds.show();
  }
  else
  {
    keypadleds.setPixelColor(1, 0);
    keypadleds.show();
  }

  if ( select_button.isPressed() ) 
  {
    // TOGGLE THE LED STATE : 
    keypadleds.setPixelColor(0,YELLOW2);
    keypadleds.show();
  }
  else
  {
    keypadleds.setPixelColor(0, 0);
    keypadleds.show();
  }

}
