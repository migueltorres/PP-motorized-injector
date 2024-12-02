// construtors for above libraries for instances of components
// go here for easier reference for later setup of each instance below

ESP32Encoder encoder;  // should use PCNT timer on ESP32, so should be callable at any moment with updated encoder value

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
#define DRIVER_RMT 1
setAbsoluteSpeedLimit(16000000/maxSpeedLimit);  // 16.000.000 tick/s / 2000steps/s maxSpeedLimit = 8000 ticks/ step
                                                // 16.000.000 tick/s / 5000steps/s maxSpeedLimit = 3200 ticks/ step
elapsedMillis printTime;

// ACS712  ACS1(34, 5.0, 4095, 100);
// ACS712  ACS2(35, 5.0, 4095, 100);

// SPI instance here..?

Adafruit_MAX31855 thermocouple(TEMPNozzleVSPI_Dpin_MOSI_CS);

Adafruit_NeoPixel keypadleds = Adafruit_NeoPixel(ledCount, WS2812B_addressableLEDs, NEO_GRB + NEO_KHZ800);

Bounce SelectButtonBounce = Bounce ();
Bounce UpButtonBounce = Bounce ();
Bounce DownButtonBounce = Bounce ();

