#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

//Debug Definitions
//#define RAW_COLOR_DEBUG
//#define COLOR_DEBUG

// Pin definitions
#define IR_SENSOR_PIN A0

#define INTERRUPT_PIN D7

#define COM_PIN_0 D9
#define COM_PIN_1 D8
#define COM_PIN_2 D10

// Calibration values
#define g_CALIBRATION_SAMPLES 3
#define g_CALIBRATION_DELAY 300


/*
typedef enum {
    WHITE_TILE_COLOR = 0,
    BLUE_TILE_COLOR = 1,
    SILVER_TILE_COLOR = 2,
    BLACK_TILE_COLOR = 3
    
} Color_t;
*/

#define COLOR_UNKNOWN 6
#define COLOR_RED     4
#define COLOR_GREEN   5
#define COLOR_BLUE    1
#define COLOR_BLACK   3
#define COLOR_WHITE   0
#define COLOR_SILVER  2

// Function declarations
void initColorSensor();
bool bootButtonPressed();
void waitForButtonPress();
void interrupt();
uint32_t calibrateClear();
void calibrate();
void writeColorToPins();
void writeColorToSerial();
void getTileColor();
void requestEvent();

// External variable declarations (to be defined in colorSensor.cpp)
extern Adafruit_TCS34725 colorSensor;
extern uint16_t r, g, b, c;
extern uint16_t ir;
extern int thresh_clear;
extern int thresh_color;
extern uint8_t current_color;

#endif // COLOR_SENSOR_H