# Fisherman
This repository contains the payload mechanism code that I flash on my ESP32 board. This document will describe the different parameters you can control and change. 

## ESP32 Pinout 
The board I am using is an ESP32-WROOM-32. ![ESP32 WROOM 38 PIN PINOUT](https://myhomethings.eu/wp-content/uploads/2024/02/ESP32-38pin-Develeopmen-Board-ADC-Gpio-Pins-1.webp)

## Circuit Guide
Do not alter connections among the motor driver unless you know what you are doing. The key components you need to worry about would be using the proper GPIO pins for I/O. 
The motor driver and the motor havn a separate power supply on the end of the board, use a AC-DC power block with a barrel connector *make sure it only supplies up to 9V*

## Parameters to tweak
| Parameter        | Purpose           | Range  |
| ------------- |:-------------:| -----:|
| `K_p`     | PID | 5-30 |
| `K_i`      | PID      |  15-40 |
| `K_d` | PID      |  0-3 |
| `MIN_SPEED` | deadzone | 0-5 |
| `STICTION_KICK` | deadzone input | 30-80 |
| `RELEASE` | preset position | (see below) |
| `EQUILIBRIUM` | preset position | (see below) |
| `PICKUP` | preset position | (see below) |

## Preset positions
Positive positions translate to counter-clock wise rotation.
The conversion rate is about `1 ROTATAION: 232 ENCODER POSITIONS` 
This was determined emperically, it has an error of about 3% per complete rotation, feel free to test further.

## FOR TESTING
* Open serial monitor for that COM_PORT (Baud rate is 115200)
* Input a PWM signal through the pins (Use servo tester or whatever microcontroller)
* Set to any preset position, allow to stabilize if necessary
* Apply Torque
* Note down: PID response under constant load through serial monitor (does it create oscillations? Does it not respond at all?)

## Questions
If there are any questions for this feel free to reach out to me on slack!!!
