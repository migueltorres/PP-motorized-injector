
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
#define HBRIGHTNESS 200
#define LBRIGHTNESS 50



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
  clearLEDs();   // This function, defined below, turns all LEDs off...

  keypadleds.setBrightness(LBRIGHTNESS);
  keypadleds.setPixelColor(2,RED_rgb);
  keypadleds.setPixelColor(1,GREEN_rgb);
  keypadleds.setPixelColor(0,BLUE_rgb);
  keypadleds.show();

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

   // GET THE STATE WITH <Bounce.read()>
  int downDebouncedState = down_button.read();
  int upDebouncedState = up_button.read();
  int selectDebouncedState = select_button.read();

  /*keypadleds.setBrightness(50);
  keypadleds.setPixelColor(2,RED_rgb);
  keypadleds.setPixelColor(1,GREEN_rgb);
  keypadleds.setPixelColor(0,BLUE_rgb);
  keypadleds.show();*/

  // <Button>.pressed() RETURNS true IF THE STATE CHANGED
  // AND THE CURRENT STATE MATCHES <Button>.setPressedState(<HIGH or LOW>);
  // WHICH IS LOW IN THIS EXAMPLE AS SET WITH button.setPressedState(LOW); IN setup()
  if ( down_button.isPressed() ) 
  {
    // TOGGLE THE LED STATE : 
    keypadleds.setPixelColor(2,RED_rgb);
    keypadleds.setBrightness(HBRIGHTNESS);
    keypadleds.show();
    if ( downDebouncedState == LOW && down_button.currentDuration() > 1000 )
    {
      keypadleds.setPixelColor(1,RED_rgb);
      keypadleds.setBrightness(LBRIGHTNESS);
      keypadleds.show();
    }
  down_button.update();
    if ( !down_button.isPressed() )
    {
      keypadleds.setPixelColor(1,GREEN_rgb);
      keypadleds.setBrightness(LBRIGHTNESS);
      keypadleds.show();
    }
  }
  else 
  {
    keypadleds.setBrightness(50);
    keypadleds.show();
  }

  if ( up_button.isPressed() ) 
  {
    // TOGGLE THE LED STATE : 
    keypadleds.setPixelColor(1,GREEN_rgb);
    keypadleds.setBrightness(HBRIGHTNESS);
    keypadleds.show();
    if ( upDebouncedState == LOW && up_button.currentDuration() > 1000 )
    {
      keypadleds.setPixelColor(0,GREEN_rgb);
      keypadleds.setBrightness(LBRIGHTNESS);
      keypadleds.show();
    }
  up_button.update();
    if ( !up_button.isPressed() ) 
    {
      keypadleds.setPixelColor(0,BLUE_rgb);
      keypadleds.setBrightness(LBRIGHTNESS);
      keypadleds.show();
    }
  }
  else
  {
    keypadleds.setBrightness(50);
    keypadleds.show();
  }

  if ( select_button.isPressed() ) 
  {
    // TOGGLE THE LED STATE : 
    keypadleds.setPixelColor(0,BLUE_rgb);
    keypadleds.setBrightness(HBRIGHTNESS);
    keypadleds.show();
  }
  else
  {
    keypadleds.setBrightness(50);
    keypadleds.show();
  }

}
