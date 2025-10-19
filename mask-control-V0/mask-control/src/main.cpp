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
 * GND  --> GND
 * VCC  --> 5V
 *
 * MAX9814:
 * Microphone OUT --> Pin 34 (ESP32 Analog Input)
 * VCC           --> 3.3V (ESP32)
 * GND           --> GND (ESP32)
 *
 * Button for Mode Switch:
 * GPIO12 with internal pull-up resistor enabled
 *
 * Fan State Detection Circuit:
 * Voltage at GPIO14 is given by the voltage divider:
 * >--+ 5V Power Supply for Fan Circuit --+
 *  |
 * | |
 * | | R1
 * | |
 *  |
 *  +------GPIO14 (Fan Check Pin)
 *  |
 * | |
 * | | R2
 * | |
 *  |
 * GND+-----GND
 *
 * R1 = 660 Ohm
 * R2 = 1k Ohm
 *
 *
 */

#include <Arduino.h>

// NO_LCD // #include "LiquidCrystal_I2C.h"
#include "LedController.hpp"
#include "MatrixLayout.h"

#define ANALOG_PIN_MIC 34
#define FAN_CHECK_PIN 14 // Pin to check fans state
#define BUTTON_PIN 12    // Pin for mode switch button

#define LCD_SDA 21
#define LCD_SCL 22

#define DIN 27
#define CS 26
#define CLK 25

#define Segments 12

#define delayTime 100 // Delay between Frames

LedController lc = LedController();
MatrixLayout ml = MatrixLayout();
// NO_LCD // LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Microphone variables
const int sampleWindow = 50;
unsigned int sample;
unsigned long sig = 0;

// Blink secondstart
int blinkInterval = 2000; // milliseconds
int mode = 0;             // Animation mode
int fanState = 0;         // Fan state
int buttonState = 0;      // Button state
int lastButtonState = 0;  // Last button state

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
    // init LED Matrix
    lc.init(DIN, CLK, CS, Segments); // Pins: DIN,CLK,CS, # of Display connected
    lc.setIntensity(1);
    lc.clearMatrix();

    // Setup LED Layout
    ml.init();

    // init Button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // init Fan Check Pin
    pinMode(FAN_CHECK_PIN, INPUT);

    // NO_LCD //
    // // init LCD
    // lcd.init(LCD_SDA, LCD_SCL);
    // lcd.backlight();
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("Protogen init");
    // lcd.setCursor(0, 1);
    // lcd.print("Time is now!");
    // delay(2000);
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("Protogen online");
    // lcd.setCursor(0, 1);
    // lcd.print("Mode: Normal");
}

void loop()
{
    // Eyes Animation
    unsigned long currentMillis = millis();
    if (currentMillis % blinkInterval < 500) // Blink every blinkInterval milliseconds
    {
        blinkInterval = random(1500, 4000); // next blink between 1.5s and 4s
        ml.eyesType(EyeType::EYE_CLOSED);   // Closed Eyes
    }
    else
    {
        ml.eyesType(EyeType::EYE_OPEN); // open Eyes
    }

    // Microphone input
    sig = meassureSoundLevel(); // generates a delay of ~50ms to sample sound level

    // Mode Switch Button
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == HIGH && lastButtonState == LOW)
    {
        mode = (mode + 1) % 4; // Cycle through modes 0, 1, 2, 3
        lastButtonState = buttonState;
    }

    fanState = digitalRead(FAN_CHECK_PIN); // Read fan state

    // Mouth Animation based on mode
    switch (mode)
    {
    case 0:            // Normal Mode
        if (!fanState) // only animate if fans are off
        {

            // NO_LCD // lcd.setCursor(0, 1);
            // NO_LCD // lcd.print("Mode: Normal");
            // Mouth Animation based on sound level
            if (sig < 1.5)
            {
                ml.mouthType(MouthType::MOUTH_CLOSED);
            }
            else if (sig >= 1.5 && sig < 3.5)
            {
                ml.mouthType(MouthType::MOUTH_OPEN);
            }
            else if (sig >= 3.5 && sig < 5.0)
            {
                ml.mouthType(MouthType::MOUTH_WIDE_OPEN);
            }
            else
            {
                ml.mouthType(MouthType::MOUTH_CLOSED);
            }
            break;
        }

    case 1: // static and/ or fans active
        // NO_LCD // lcd.setCursor(0, 1);
        // NO_LCD // lcd.print("Mode: Static ");
        ml.eyesType(EyeType::EYE_OPEN);
        ml.mouthType(MouthType::MOUTH_CLOSED);
        break;
    case 2: // Sleeping
        // NO_LCD // lcd.setCursor(0, 1);
        // NO_LCD // lcd.print("Mode: Sleeping");
        ml.eyesType(EyeType::EYE_X);
        ml.mouthType(MouthType::MOUTH_SLEEPING);
        break;
    case 3: // Heart Eyes
        // NO_LCD // lcd.setCursor(0, 0);
        // NO_LCD // lcd.print("Mode: Love");
        ml.eyesType(EyeType::EYE_HEART);
        break;

    default:
        // NO_LCD // lcd.setCursor(0, 0);
        // NO_LCD // lcd.print("Mode: Love");
        ml.eyesType(EyeType::EYE_OPEN);
        ml.mouthType(MouthType::MOUTH_CLOSED);
        break;
    }

    updateLED();      // Update LED Matrix
    delay(delayTime); // Delay between Frames
}