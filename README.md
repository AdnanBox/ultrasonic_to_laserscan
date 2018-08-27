# ultrasonic_to_laserscan
This package reads the incoming data from an ultrasonic sensor and publishes it as a LaserScan msg. To overcome the drawbacks of a laserscanner in detecting transparent obstacles,this package may come in handy. 

To use this package, it is required to first clone the wiringPi(c++) library from Hardkernel. 
https://github.com/hardkernel/wiringPi.git

The package was implemented using an odroid XU4. The ultrasonic sensor HC-SR04 supports only 5V. Since all the native GPIO pins on the odroid XU4 operate at 1.8V, a shifter shield is needed to shift the voltage level to 3.3V or 5V.

To determine the GPIO pin locations on the shifter shield and their corresponding wiringPi numbers, refer to the following link :
https://wiki.odroid.com/accessory/add-on_boards/xu4_shift_shield


Pin declaration :
#define pin_A 6 // pin number 22

In this case, the pin at location "22" on the shifter shield is defined with the variable name "pin_A" having a wiringPi number "6".


