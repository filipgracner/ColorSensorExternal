#include "colorSensor.h"

// Global variables definition
Adafruit_TCS34725 colorSensor;
uint16_t r, g, b, c;
uint16_t ir = 0;
int thresh_clear = 200;
int thresh_color = 13;
uint8_t current_color = COLOR_UNKNOWN;

void initColorSensor() {
  colorSensor.init();
  colorSensor.setIntegrationTime(TCS34725_INTEGRATIONTIME_101MS);
  colorSensor.setGain(TCS34725_GAIN_16X);
  pinMode(IR_SENSOR_PIN, INPUT);
  
  // Set up interrupt pin for black detection
  pinMode(INTERRUPT_PIN, OUTPUT);
  digitalWrite(INTERRUPT_PIN, LOW); // Active LOW interrupt
  
  // Initialize I2C slave
  Wire.setSDA(4);  // Set SDA pin (check XIAO RP2040 pinout)
  Wire.setSCL(5);  // Set SCL pin (check XIAO RP2040 pinout)
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
}

// Function that's called when the master requests data
void requestEvent() {
  // Send the current color value when requested
  Wire.write(current_color);
}

bool bootButtonPressed() {
  // Get the current state of the GPIO connected to the BOOTSEL button
  const uint CS_PIN_INDEX = 1; // CS pin is the second QPI pin
  
  // Disable interrupts
  uint32_t save = save_and_disable_interrupts();
  
  // Set CS as GPIO
  hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                 GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                 IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
  
  // Enable input
  hw_set_bits(&ioqspi_hw->io[CS_PIN_INDEX].ctrl, 
              GPIO_OVERRIDE_HIGH << IO_QSPI_GPIO_QSPI_SS_CTRL_INOVER_LSB);
  
  // Wait for input to settle
  for (volatile int i = 0; i < 10; i++);
  
  // Read current state (active low, so invert result)
  bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));
  
  // Restore interrupts
  restore_interrupts(save);
  
  return button_state;
}

// Function to wait for button press
void waitForButtonPress() {
  Serial.println("Press the BOOT button to continue...");
  
  // Wait for button to be released first (in case it was already pressed)
  while (bootButtonPressed()) {
    delay(10); // Small delay to prevent CPU hogging
  }
  
  // Now wait for a press
  while (!bootButtonPressed()) {
    delay(10);
  }
  
  // Wait for release to prevent multiple triggers
  while (bootButtonPressed()) {
    delay(10);
  }
  
  Serial.println("Button pressed! Continuing...");
}

void interrupt() {
  digitalWrite(INTERRUPT_PIN, HIGH); // Trigger interrupt when black
  // Small delay then reset interrupt
  delay(5);
  digitalWrite(INTERRUPT_PIN, LOW);
}

uint32_t calibrateClear() {
  uint16_t c;
  uint16_t c_low = 65535;
  uint16_t c_high = 0;

  for (int i = 0; i < g_CALIBRATION_SAMPLES; i++) {
    colorSensor.getRawData(&r, &g, &b, &c);

    if (c > c_high) {
      c_high = c;
    }
    if (c < c_low) {
      c_low = c;
    }

    delay(g_CALIBRATION_DELAY);
  }

  // Pack c_low and c_high into a single uint32_t
  return ((uint32_t)c_high << 16) | c_low;
}

void calibrate() {
  uint16_t calibrated_low_clear;
  uint16_t calibrated_high_clear;
  uint32_t calibrated_clear;

  Serial.println("Put the robot on the BLACK tile!\n\r");
  waitForButtonPress();
  calibrated_clear = calibrateClear();
  calibrated_low_clear = (uint16_t)(calibrated_clear & 0xFFFF); // Extract low clear value
  calibrated_high_clear = (uint16_t)((calibrated_clear >> 16) & 0xFFFF); // Extract clear_high
  Serial.printf("Low: %d, High: %d\n\r", calibrated_low_clear, calibrated_high_clear);
  delay(300);

  Serial.println("Put the robot on the WHITE tile!\n\r");
  waitForButtonPress();
  calibrated_clear = calibrateClear();
  calibrated_low_clear = (uint16_t)(calibrated_clear & 0xFFFF); // Extract low clear value
  calibrated_high_clear = (uint16_t)((calibrated_clear >> 16) & 0xFFFF); // Extract clear_high
  Serial.printf("Low: %d, High: %d\n\r", calibrated_low_clear, calibrated_high_clear);
  delay(300);

  Serial.println("Put the robot on the SILVER tile!\n\r");
  waitForButtonPress();
  calibrated_clear = calibrateClear();
  calibrated_low_clear = (uint16_t)(calibrated_clear & 0xFFFF); // Extract low clear value
  calibrated_high_clear = (uint16_t)((calibrated_clear >> 16) & 0xFFFF); // Extract clear_high
  Serial.printf("Low: %d, High: %d\n\r", calibrated_low_clear, calibrated_high_clear);
  delay(300);
}

void getTileColor() {
  uint8_t previous_color = current_color;
  colorSensor.getRawData(&r, &g, &b, &c);
  ir = analogRead(IR_SENSOR_PIN);

  if (c > 0) {
    r = (r * 255) / c;
    g = (g * 255) / c;
    b = (b * 255) / c;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }

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
      current_color = COLOR_BLACK;
      interrupt();
      Serial.println("Black");
      break;
    case false:
      switch (c > (g_WHITE_LOW_CLEAR - thresh_clear) || c > (g_SILVER_LOW_CLEAR - thresh_clear)) {
        case true:
          switch (ir < 250) {
            case true:
              current_color = COLOR_SILVER;
              Serial.println("Silver");
              break;
            case false:
              current_color = COLOR_WHITE;
              Serial.println("White");
              break;
          }
          break;
        case false:
          switch ((r - g > thresh_color) && (r - b > thresh_color)) {
            case true:
              current_color = COLOR_RED;
              Serial.println("Red");
              break;
            case false:
              switch ((g - r > thresh_color) && (g - b > thresh_color)) {
                case true:
                  current_color = COLOR_GREEN;
                  Serial.println("Green");
                  break;
                case false:
                  switch ((b - r > thresh_color) && (b - g > thresh_color)) {
                    case true:
                      current_color = COLOR_BLUE;
                      Serial.println("Blue");
                      break;
                    case false:
                      current_color = COLOR_UNKNOWN;
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
