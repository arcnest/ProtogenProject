/*
 * Protogen Mask Control
 * Main Program File
 * Created by Arcnest, October 2025.
 * Released into the public domain.
 * This program is distributed in the hope that it will be useful,
 *
 * Used to control LED matrix mask with microphone input for mouth animation and timed eye blinking.
 * Uses LedControl library for controlling the LED matrix.( noah1510/LedController@^1.7.0 )
 * MatrixLayout class defines different eye and mouth layouts.
 *
 * Used hardware:
 * - ESP32 microcontroller
 * - 12 x MAX7219 LED 8x8 matrix driver
 * - MAX9814 microphone amplifier module
 *
 *
 * Setup connections:
 * MAX7219:
 * DIN  --> Pin 27 (ESP32)
 * CLK  --> Pin 25 (ESP32)
 * CS   --> Pin 26 (ESP32)
 * GND  --> GND (ESP32)
 * VCC  --> 5V (ESP32)
 *
 * MAX9814:
 * Microphone OUT --> Pin 34 (ESP32 Analog Input)
 * VCC           --> 3.3V (ESP32)
 * GND           --> GND (ESP32)
 */

#include <Arduino.h>

#include "LedController.hpp"
#include "MatrixLayout.h"

#define ANALOG_PIN_MIC 34

#define DIN 27
#define CS 26
#define CLK 25

#define Segments 12

#define delayTime 100 // Delay between Frames

LedController lc = LedController();
MatrixLayout ml = MatrixLayout();

// Microphone variables
const int sampleWindow = 50;
unsigned int sample;
unsigned long sig = 0;

// Blink secondstart
int blinkInterval = 2000; // milliseconds

// Update LED Matrix from layoutMatrix
void updateLED()
{
    for (int i = 0; i < 12; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                lc.setLed(i, j, k, ml.layoutMatrix[i][j][k]);
            }
        }
    }
}

// Microphone sound level measurement
unsigned long meassureSoundLevel()
{
    unsigned long startMillis = millis();
    unsigned int peakToPeak = 0;
    unsigned int signalMax = 0;
    unsigned int signalMin = 3100;

    while (millis() - startMillis < sampleWindow) // sample for 50 milliseconds
    {
        sample = analogRead(ANALOG_PIN_MIC);
        if (sample < 3100)
        {
            if (sample > signalMax)
            {
                signalMax = sample;
            }
            else if (sample < signalMin)
            {
                signalMin = sample;
            }
        }
    }
    if (signalMax > signalMin) // avoid division by zero
    {
        return ((signalMax - signalMin) * 5.0) / 3100;
    }

    return 0;
}

void setup()
{
    // TEST_SETUP
    // Serial.begin(9600);

    lc.init(DIN, CLK, CS, Segments); // Pins: DIN,CLK,CS, # of Display connected
    lc.setIntensity(1);
    lc.clearMatrix();

    // Setup Layout
    ml.init();
}

void loop()
{
    // Eyes Animation
    unsigned long currentMillis = millis();
    if (currentMillis % blinkInterval < 500) // Blink every blinkInterval milliseconds
    {
        blinkInterval = random(1500, 4000); // next blink between 1.5s and 4s
        ml.eyesType(1);                     // Closed Eyes
    }
    else
    {
        ml.eyesType(0); // open Eyes
    }

    // Microphone
    sig = meassureSoundLevel();

    // Mouth Animation based on sound level
    if (sig < 1.5)
    {
        ml.mouthType(0);
    }
    else if (sig >= 1.5 && sig < 3.5)
    {
        ml.mouthType(1);
    }
    else if (sig >= 3.5 && sig < 5.0)
    {
        ml.mouthType(2);
    }
    else
    {
        ml.mouthType(0);
    }

    updateLED();      // Update LED Matrix
    delay(delayTime); // Delay between Frames
}