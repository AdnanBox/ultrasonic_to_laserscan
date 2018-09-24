To use this repository, it is required to first clone the wiringPi library from Hardkernel. https://github.com/hardkernel/wiringPi.git \
Build the library using the build script available in the root directory of the library. 

This package reads the incoming data from an ultrasonic sensor and publishes it as a LaserScan msg under the topic /scan.

The package was implemented using an odroid XU4. The ultrasonic sensor HC-SR04 supports only 5V. Since all the native GPIO pins on the odroid XU4 operate at 1.8V, a shifter shield is needed to shift the voltage level to 3.3V or 5V.

To determine the GPIO pin locations on the shifter shield and their corresponding wiringPi numbers, refer to the following link :
https://wiki.odroid.com/accessory/add-on_boards/xu4_shift_shield


Example pin declaration :
```
#define pin_A 6 // pin number 22
```
In this case, the pin at location "22" on the shifter shield is defined with the variable name "pin_A" having a wiringPi number "6".

The trigger pin of the ultrasonic sensor is held at HIGH for 20ms to send out an array of pules; and then set to LOW. Next, the value of the input echo pin is monitored continuosly. After striking an obstacle, the pulses are received by the echo end of the sensor and set the echo pin to HIGH. The time for which the echo pin remains at HIGH position corresponds to the distance of the obstacle. 

distance = time(µs)/58  // this formula gives the distance in cm.

For a more detailed explanation of the functioning of an ultrasonic sensor, refer to the following link :
http://arduino-info.wikispaces.com/Ultrasonic+Distance+Sensor

<img src="https://github.com/ipa-fog-ab/ultrasonic_to_laserscan/blob/master/img/setup1.jpg" width="300">
<img src="https://github.com/ipa-fog-ab/ultrasonic_to_laserscan/blob/master/img/setup2.jpg" width="300">


HOW TO:
1. Run the launch file using :
```
roslaunch ultrasonic_to_laserscan scan_publisher.launch
```


NOTE: The nodes fall_sensor.cpp, clearance_publisher.cpp, and scan_publisher.cpp are optional and used only for testing purposes of the ultrasonic sensor.
main_optimised.cpp is a more efficient version of main.cpp, but ultimately serves the same purpose.
