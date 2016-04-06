Turku Ekotuki trainset controller
=================================

Pedal powered trainset controller, using Arduino and a bunch of other components.

This is very much work in progress at this time.

What does it do ?
=================
Takes 8-12V as input and runs a train set. The model also has lights for various purposes.

Requirements:
=============
* PWM controlled motor controller
* PWM controlled LED controller
* 2 Track sensors connected to IRQ pins
* INA219 for voltage and current measurements
* I2C connected character LCD for informational messages
* I2C connected OLED screen for sponsor information
* (SD card reader, amplifier and speaker for action sounds)

TODO
====
* Sounds
* Adjust lights in some way
* Add switch handling
