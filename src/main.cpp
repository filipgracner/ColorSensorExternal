#include <Arduino.h>
#include "colorSensor.h"

void setup() {
  Serial.begin(115200);

  #ifdef RAW_COLOR_DEBUG
    while (!Serial);
  #endif

  initColorSensor();

  //calibrate();
}

void loop() {
  //Serial.println(bootButtonPressed());
  getTileColor();
}