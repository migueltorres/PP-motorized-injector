# PP-motorized-injector
Motorized plastic injector using Arduino code to be open sourced with Precious Plastic initiative once working!

I create this repo as I would like help in getting this finished and resolving issues that Iam having:

after getting a simple 2 button 1 pot FastAccelStepper sketch working fine on Nano, discovered that adding an Encoder was now conflicting with the only available interrupts that FAS was also using, so decided to upgrade to ESP32, to be able to dedicate 1 core toFAS and use PCNT pulse timer for uninterrupted Encoder data collection (my intention of running the stepper until it loses steps to assure maximum pressure requieres an independent feedback of what the stepper is actually doing, and using a Comparison function to keep the stepper at its limit (whilst losing steps, but not innecessarily), to reduce speed and distance commands when the plastic is fully compressed / mould is full (that's the theory!)

however, after setting up a layout of my circuit, strange things are happening, motor does not respond, sometimes jiggers, etc...:

am using 5v 2A PSU to power a AMS1117 3.3V regulator, that powers ESP32 WROOM 32 dev module, and a TXS0108E LLC, as the DM860H driver requires 5v logic to work, and a 80v 12.5A PSU for the driver, motor should only pull 7.2A max per driver specs... the included 2but1pot sketch worked fine on a Nano, but does not want to cooperate on the ESP32.. strangest thing is, that as soon as connect only GND to PUL- & DIR- on the driver, I am seeing 2-2.2v between PUL+ & -, and also DIR + & -, when I then connect to LLC 5v side to PUL+ & DIR +, then the OUTPUTs of the 5v LLC side are actually LOWER than the INPUTS from the ESP32 - and also the inputs are as well reduced, the DIR from 3.3v to about 3v... of the driver is not connected to LLC, ESP32 outputs are correct, and LLC outputs are correct, 5v in case of DIR being positive move, 0v when
negative move commanded...

attached in (my first) repo are photos of setup, datasheets for the components used, just cannot workout what is ging wrong.. the only thing can think of is maybe the LLC is using open-drain and should be using push-pull, as the FAS commands are very fast, but how to tell FAS to use pull-ups on ESP32...? or is this irrelevante for outputs on the dev board...? 
