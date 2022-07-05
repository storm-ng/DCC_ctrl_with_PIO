# DCC_ctrl_with_PIO
In this project I want to use the Raspberry Pi 2040, as used on the Arduino Nano RP2040 and many other pico based designs to control a train and otherdevices using DCC formatted communication.

Features:
1) Generate DCC formated communication using the PIO of the RP2040
2) Use MQTT over wifi to set variables like address / speed and keep status
3) Use MBed to be able to:
  - run tasks for MQTT message translation to DCC updates
  - run other jobs like led blink updates and other thinks I have not thought up yet.
    
