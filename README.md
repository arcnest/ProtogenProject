Protogen Mask Control

Created by Arcnest, October 2025.
Released into the public domain.

This program is distributed in the hope that it will be useful,

Used to control LED matrix mask with microphone input for mouth animation and timed eye blinking.
Uses LedControl library for controlling the LED matrix( noah1510/LedController@^1.7.0 ).
MatrixLayout class defines different eye and mouth layouts.

Used hardware:
- ESP32 microcontroller
- 12 x MAX7219 LED 8x8 matrix driver
- MAX9814 microphone amplifier module

Setup connections:

MAX7219:
- DIN  --> Pin 27 (ESP32)
- CLK  --> Pin 25 (ESP32)
- CS   --> Pin 26 (ESP32)
- GND  --> GND 
- VCC  --> 5V 

MAX9814:
- Microphone OUT --> Pin 34 (ESP32 Analog Input)
- VCC           --> 3.3V (ESP32)
- GND           --> GND (ESP32)
 
Button for Mode Switch:
- GPIO12 with internal pull-up resistor enabled

Fan State Detection Circuit:
- Voltage at GPIO14 is given by the voltage divider
- R1 = 660 Ohm
- R2 = 1k Ohm

![circuit diagram](https://github.com/arcnest/ProtogenProject/blob/master/Verkabelung/ProtogenLayout_Steckplatine.png?raw=true)


