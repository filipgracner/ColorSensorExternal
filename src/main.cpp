#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 colorSensor;
uint16_t r, g, b, c;
uint16_t ir;

#define IR_SENSOR_PIN 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
}

