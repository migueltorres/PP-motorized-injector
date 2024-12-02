void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
}

void loop() {
  // read the analog / millivolts value for pin 2:
  int analogValue = analogRead(27);
  int analogVolts = analogReadMilliVolts(27);

  // print out the values you read:
  Serial.printf("ADC analog value = %d\n", analogValue);
  Serial.printf("ADC millivolts value = %d\n", analogVolts);

  delay(100);  // delay in between reads for clear read from serial
}



/* ADJUST TO READ MILLIVOLTS OF ACS712 SENSOR! to check that with 10k resistor
 the voltaje midpoint (2.5v) does not pass 3.3v in positive (MAX FOR ESP32!)
otherwie try with LLC, just to see what happenes..? Or investigate other ways of
changing the midpoint voltaje to abot 1.65v (ideal)
*/