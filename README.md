# 180Radar

This is a project using Raspberry Pi 3, a SG90 Servo and a VL53L0X TOF I2C sensor. I also created a small breakout board that uses a LM317T adjustable voltage regulator,
to power the servo (remember to have the same GND). 

The whole thing works in the following way: it's actually based on an infinite loop... at specific intervals of time the servo increases it's movement by one degree
(0-180 and then 180-0); at each degree the sensor takes a measure and plots it on the GUI, taking as parameters the distance and the degrees (which are converted in
polar coordinates).

The same project can be implemented using other MCU's; for example if you want to use an Arduino or a RPI Pico, you also need to include in the Python file a method
which receives,reads and parses serial data, you just need to send 2 parameters so it's not too hard.
