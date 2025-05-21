#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

#define IR_SENSOR_PIN 7
#define g_CALIBRATION_SAMPLES 3 // Number of samples for calibration
#define g_CALIBRATION_DELAY 3000 // Delay in milliseconds for calibration
// Clear calibration values for black, white and silver tiles
#define g_BLACK_LOW_CLEAR 1127
#define g_BLACK_HIGH_CLEAR 1136
#define g_WHITE_LOW_CLEAR 8929
#define g_WHITE_HIGH_CLEAR 9351
#define g_SILVER_LOW_CLEAR 7398
#define g_SILVER_HIGH_CLEAR 7439

Adafruit_TCS34725 colorSensor;
uint16_t r, g, b, c;
uint16_t ir;
int thresh_clear = 200;
int thresh_color = 13;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("TCS34725 Test");

  colorSensor.init();
  colorSensor.setIntegrationTime(TCS34725_INTEGRATIONTIME_101MS);
  colorSensor.setGain(TCS34725_GAIN_16X);

  pinMode(IR_SENSOR_PIN, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  colorSensor.getRawData(&r, &g, &b, &c);
  ir = analogRead(IR_SENSOR_PIN);
  Serial.print("R: ");
  Serial.print(r);
  Serial.print(" G: ");
  Serial.print(g);
  Serial.print(" B: ");
  Serial.print(b);
  Serial.print(" C: ");
  Serial.print(c);
  Serial.print(" IR: ");
  Serial.println(ir);

  switch (c < (g_BLACK_HIGH_CLEAR + thresh_clear)) {
    case true:
      Serial.println("Black");
      break;
    case false:
      switch (c > (g_WHITE_LOW_CLEAR - thresh_clear) || c > (g_SILVER_LOW_CLEAR - thresh_clear)) {
        case true:
          switch (ir < 250) {
            case true:
              Serial.println("Silver");
              break;
            case false:
              Serial.println("White");
              break;
          }
          break;
        case false:
          switch ((r - g > thresh_color) && (r - b > thresh_color)) {
            case true:
              Serial.println("Red");
              break;
            case false:
              switch ((g - r > thresh_color) && (g - b > thresh_color)) {
                case true:
                  Serial.println("Green");
                  break;
                case false:
                  switch ((b - r > thresh_color) && (b - g > thresh_color)) {
                    case true:
                      Serial.println("Blue");
                      break;
                    case false:
                      Serial.println("Unknown");
                      break;
                  }
                  break;
              }
              break;
          }
          break;
      }
      break;
  }

}

