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

// Pin definitions
#define IR_SENSOR_PIN 7
#define BOOTSEL_PIN 21
#define I2C_SLAVE_ADDRESS 0x42
#define INTERRUPT_PIN 6

// Calibration values
#define g_CALIBRATION_SAMPLES 3
#define g_CALIBRATION_DELAY 3000
#define g_BLACK_LOW_CLEAR 1127
#define g_BLACK_HIGH_CLEAR 1136
#define g_WHITE_LOW_CLEAR 8929
#define g_WHITE_HIGH_CLEAR 9351
#define g_SILVER_LOW_CLEAR 7398
#define g_SILVER_HIGH_CLEAR 7439

// Color codes
#define COLOR_UNKNOWN 0
#define COLOR_RED     1
#define COLOR_GREEN   2
#define COLOR_BLUE    3
#define COLOR_BLACK   4
#define COLOR_WHITE   5
#define COLOR_SILVER  6

// Function declarations
void initColorSensor();
bool bootButtonPressed();
void waitForButtonPress();
void interrupt();
uint32_t calibrateClear();
void calibrate();
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