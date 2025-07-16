#include "colorSensor.h"
#include <Adafruit_NeoPixel.h>

int Power = 11;
int PIN  = 12;
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Global variables definition
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_16X);;
uint16_t r, g, b, c;
uint16_t ir = 0;
int thresh_clear = 200;
int thresh_color = 13;
uint8_t current_color = COLOR_UNKNOWN;

void initColorSensor() {

  pixels.begin();
  pinMode(Power,OUTPUT);
  digitalWrite(Power, HIGH);
  pixels.setPixelColor(0, pixels.Color(100, 100, 100));
  pixels.show();

  Wire.begin();

  if (colorSensor.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  pinMode(COM_PIN_0, OUTPUT);
  pinMode(COM_PIN_1, OUTPUT);
  pinMode(COM_PIN_2, OUTPUT);


  // Set up interrupt pin for black detection
  pinMode(INTERRUPT_PIN, OUTPUT);
  digitalWrite(INTERRUPT_PIN, LOW); // Active LOW interrupt


  Serial.println("Color Sensor Init over");
}


bool __no_inline_not_in_flash_func(bootButtonPressed)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
#if PICO_RP2040
    #define CS_BIT (1u << 1)
#else
    #define CS_BIT SIO_GPIO_HI_IN_QSPI_CSN_BITS
#endif
    bool button_state = !(sio_hw->gpio_hi_in & CS_BIT);

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

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

uint16_t g_BLACK_LOW_CLEAR = 425;
uint16_t g_BLACK_HIGH_CLEAR = 442;
uint16_t g_WHITE_LOW_CLEAR = 4557;
uint16_t g_WHITE_HIGH_CLEAR = 4597;
uint16_t g_SILVER_LOW_CLEAR = 4753;
uint16_t g_SILVER_HIGH_CLEAR = 5141;

void calibrate() {
  uint32_t calibrated_clear;

  Serial.println("Put the robot on the BLACK tile!\n\r");
  pixels.setPixelColor(0, pixels.Color(255, 255, 0));    //YELLOW
  pixels.show();
  waitForButtonPress();
  calibrated_clear = calibrateClear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));         //OFF
  pixels.show();
  g_BLACK_LOW_CLEAR = (uint16_t)(calibrated_clear & 0xFFFF); // Extract low clear value
  g_BLACK_HIGH_CLEAR = (uint16_t)((calibrated_clear >> 16) & 0xFFFF); // Extract clear_high
  Serial.printf("Low: %d, High: %d\n\r", g_BLACK_LOW_CLEAR, g_BLACK_HIGH_CLEAR);


  Serial.println("Put the robot on the WHITE tile!\n\r");
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); //WHITE
  pixels.show();
  waitForButtonPress();
  calibrated_clear = calibrateClear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));         //OFF
  pixels.show();
  g_WHITE_LOW_CLEAR = (uint16_t)(calibrated_clear & 0xFFFF); // Extract low clear value
  g_WHITE_HIGH_CLEAR = (uint16_t)((calibrated_clear >> 16) & 0xFFFF); // Extract clear_high
  Serial.printf("Low: %d, High: %d\n\r", g_WHITE_LOW_CLEAR, g_WHITE_HIGH_CLEAR);


  Serial.println("Put the robot on the SILVER tile!\n\r");
  pixels.setPixelColor(0, pixels.Color(255, 0, 255));     //VIOLET
  pixels.show();
  waitForButtonPress();
  calibrated_clear = calibrateClear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));         //OFF
  pixels.show();
  g_SILVER_LOW_CLEAR = (uint16_t)(calibrated_clear & 0xFFFF); // Extract low clear value
  g_SILVER_HIGH_CLEAR = (uint16_t)((calibrated_clear >> 16) & 0xFFFF); // Extract clear_high
  Serial.printf("Low: %d, High: %d\n\r", g_SILVER_LOW_CLEAR, g_SILVER_HIGH_CLEAR);

}

void writeColorToPins() {
  switch (current_color)
  {
  case COLOR_WHITE:
  //0 000
    digitalWrite(COM_PIN_0, LOW);
    digitalWrite(COM_PIN_1, LOW);
    digitalWrite(COM_PIN_2, LOW);

    digitalWrite(INTERRUPT_PIN, LOW); 

    pixels.setPixelColor(0, pixels.Color(255, 255, 255)); //WHITE
    pixels.show();
    break;

  case COLOR_BLUE:
  //1 001
    digitalWrite(COM_PIN_0, HIGH);
    digitalWrite(COM_PIN_1, LOW);
    digitalWrite(COM_PIN_2, LOW);

    digitalWrite(INTERRUPT_PIN, LOW); 

    pixels.setPixelColor(0, pixels.Color(0, 0, 255));     //BLUE
    pixels.show();
    break;

  case COLOR_SILVER:
  //2 010
    digitalWrite(COM_PIN_0, LOW);
    digitalWrite(COM_PIN_1, HIGH);
    digitalWrite(COM_PIN_2, LOW);

    digitalWrite(INTERRUPT_PIN, LOW);

    pixels.setPixelColor(0, pixels.Color(255, 0, 255));     //VIOLET
    pixels.show();
    break;

  case COLOR_BLACK:
  //3 011
    digitalWrite(COM_PIN_0, HIGH);
    digitalWrite(COM_PIN_1, HIGH);
    digitalWrite(COM_PIN_2, LOW);

  //INTERRUPT
    digitalWrite(INTERRUPT_PIN, HIGH); // Trigger interrupt when black

    pixels.setPixelColor(0, pixels.Color(255, 255, 0));    //YELLOW
    pixels.show();
    break;

  case COLOR_RED:
  //4 100
    digitalWrite(COM_PIN_0, LOW);
    digitalWrite(COM_PIN_1, LOW);
    digitalWrite(COM_PIN_2, HIGH);

    digitalWrite(INTERRUPT_PIN, LOW); 

    pixels.setPixelColor(0, pixels.Color(255, 0, 0));      //RED
    pixels.show();
    break;

  case COLOR_GREEN:
  //5 101
    digitalWrite(COM_PIN_0, HIGH);
    digitalWrite(COM_PIN_1, LOW);
    digitalWrite(COM_PIN_2, HIGH);

    digitalWrite(INTERRUPT_PIN, LOW); 

    pixels.setPixelColor(0, pixels.Color(0, 255, 0));       //GREEN
    pixels.show();
    break;
  case COLOR_UNKNOWN:
  //6 110
    digitalWrite(COM_PIN_0, LOW);
    digitalWrite(COM_PIN_1, HIGH);
    digitalWrite(COM_PIN_2, HIGH);

    digitalWrite(INTERRUPT_PIN, LOW); 

    pixels.setPixelColor(0, pixels.Color(0, 255, 255));   //TOURQI    
    pixels.show();
    break;
  
  default:
  //? 000
    digitalWrite(COM_PIN_0, LOW);
    digitalWrite(COM_PIN_1, LOW);
    digitalWrite(COM_PIN_2, LOW);

    digitalWrite(INTERRUPT_PIN, LOW); 

    pixels.setPixelColor(0, pixels.Color(0, 0, 0));         //OFF
    pixels.show();
    break;
  }
}

void writeColorToSerial() {
  switch (current_color)
  {
  case COLOR_WHITE:
    Serial.println("WHITE");
    break;

  case COLOR_BLUE:
    Serial.println("BLUE");
    break;

  case COLOR_SILVER:
    Serial.println("SILVER");
    break;

  case COLOR_BLACK:
    Serial.println("BLACK");
    break;

  case COLOR_RED:
    Serial.println("RED");
    break;

  case COLOR_GREEN:
    Serial.println("GREEN");
    break;
  case COLOR_UNKNOWN:
    Serial.println("UNKNOWN");
    break;
  
  default:
    Serial.println("WHITE");
    break;
  }
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
  #ifdef RAW_COLOR_DEBUG
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
  #endif

  if(c < (g_BLACK_HIGH_CLEAR + thresh_clear)) {
    current_color = COLOR_BLACK;
  }
  else if ((b - r > thresh_color) && (b - g > thresh_color)) {
    current_color = COLOR_BLUE;
  }
  else if ((r - g > thresh_color) && (r - b > thresh_color)) {
    current_color = COLOR_RED;
  }
  else if ((g - r > thresh_color) && (g - b > thresh_color)) {
    current_color = COLOR_GREEN;
  }
  else if (ir < 72 && c > (g_SILVER_LOW_CLEAR - thresh_clear)) {
    current_color = COLOR_SILVER;
  }
  else if (c > (g_WHITE_LOW_CLEAR - thresh_clear)) {
    current_color = COLOR_WHITE;
  }
  else {
    current_color = COLOR_UNKNOWN;
  }

  writeColorToPins();
  writeColorToSerial();
}
