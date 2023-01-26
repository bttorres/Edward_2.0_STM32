# This... is STM32 Edward

Edward is an STM32 powered, autonomous robot.

**************************** INSERT YOUTUBE ****************************

## The code

The code in the included folders is the code that he uses to function. It's in the C language. STM32CubeIDE was used to write it.

## STM32 Edward's Parts and Their Functions:

In this repository is a PDF of the schematic depicting STM32 Edward's electronic parts and how they are connected.
* 1 NUCLEO-F446RE board
  * controls the sensors, motors, and servo
* 4 IR sensors
  * Detects nearby obstacles and cliffs
* 1 ultrasonic sensor
  * Measures the distance from nearby obstacles
* 1 H-bridge motor driver
  * Drives two DC motors
* 2 DC motors
  * Used as Edward's wheels
* 1 180 degree servo motor (SG90)
  * Used as a neck to pan the ultrasonic sensor
* 1 ball caster
  * Assists with movement
* 4 buck converters
  * Step down the voltages from the batteries to power the motors, motor driver, servo, ultrasonic sensor, and NUCLEO-F446RE
* 1 2000mAh 11.1V Li-ion batteries


Once switched on, STM32 Edward roams free. He uses his sensors to drive and avoid obstacles/cliffs on his own.


