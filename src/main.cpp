#include <Arduino.h>
#include "colorSensor.h"

void setup() {
  Serial.begin(9600);
  initColorSensor();
}

void loop() {
  getTileColor();
}