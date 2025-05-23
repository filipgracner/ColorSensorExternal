#include <Arduino.h>
#include "colorSensor.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("init");
  initColorSensor();
}

void loop() {
  getTileColor();
}