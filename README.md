# This... is Edward 2.0


Edward is an STM32 powered robot that can switch back and forth between an autonomous and controlled mode.

https://youtu.be/uKFYlaIgm4Q

## The Code

The code is split in two. One half is for Edward 2.0 himself and the other half is for his controller. All of the code was written in C language with the STM32CubeIDE.

## Edward 2.0's Parts and Their Functions:

In this repository is a PDF of the schematic depicting STM32 Edward's electronic parts and how they are connected.
* 1 NUCLEO-F446RE board
  * Controls the sensors, motors, and servo
* 1 NRF24L01 module
  * Receives the data sent by the controller
* 4 IR sensors
  * Detects nearby obstacles and cliffs
* 1 Ultrasonic sensor
  * Measures the distance from nearby obstacles
* 1 H-bridge motor driver
  * Drives two DC motors
* 2 DC motors
  * Used as Edward 2.0's wheels
* 1 180 degree servo motor (SG90)
  * Used as a neck to pan the ultrasonic sensor
* 1 Ball caster
  * Assists with movement
* 4 Buck converters
  * Step down the voltage from the battery to power the motors, motor driver, servo, ultrasonic sensor, and NUCLEO-F446RE
* 1 2000mAh 11.1V Li-ion battery

## The Parts and Their Functions of Edward 2.0's Controller:

In this repository is a PDF of the schematics depicting how all of the components are connected through the designed PCB.
* 1 NUCLEO-G431KB board
  * takes the inputs, converts the values, and sends the data using the NRF24L01
* 1 NRF24L01 module
  * Sends the data to Edward 2.0
* 2 Slider potentiometers
  * Controls the speed and head pan of Edward 2.0
* 1 Joystick
  * Controls the movement of Edward 2.0
* 1 Button
  * Controls the mode that Edward 2.0 is in (autonomous or controlled)
* 1 Buck converters
  * Step down the voltage from the battery to power all of the components
* 1 2000mAh 11.1V Li-ion battery

## Controlled Mode

In controlled mode, the user can drive Edward 2.0. They can have Edward go forward, backward, turn left, and turn right. They can also move Edward 2.0's "head" around and control his speed.

## Autonomous Mode

In autonomous mode, Edward roams free. He uses his sensors to drive and avoid obstacles/cliffs on his own. The user can use the controller to switch back to controlled mode at any time.

