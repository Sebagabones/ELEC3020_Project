#include "HC-SR04/main.hpp"
#include "WiFi.h"
#include "debouncing.hpp"
#include <AsyncDelay.h>
#include <Mocha_TFT_eSPI.h>
#include <TFT_eSPI.h>
#include <cstdlib>
#define CHANNEL0 0
#define CHANNEL1 1
#define TOTAL_TIME_PERIOD 20
// ask lab facilaitator about why i needed to set this to 10
#define PWM_RESOLUTION 10
#define INITIAL_FREQ 50
// Define TX and RX pins for UART (change if needed)
// #define TXD1 18
// #define RXD1 17
// AsyncDelay async_delay;

// Use Serial1 for UART communication
TFT_eSPI tft = TFT_eSPI(170, 320); // Init screen size

// The max duty cycle value based on PWM resolution (will be 255 if resolution
// is 8 bits)  borrowed from here,
// https://lastminuteengineers.com/esp32-pwm-tutorial/ - makes sense, just nice
// to not have it hardcoded, and only run it once

void setup() {
  setupButtonPresses();
  // setupHCSR();
  Serial.begin(115200);
  tft.init();                          // Display init
  tft.fillScreen(TFT_CATPPUCCIN_BASE); // Clear screen

  // Debouncing
}

void loop() {
  // static int state = 0, L_old = 1, R_old = 1, L, R;
  tft.setTextColor(TFT_CATPPUCCIN_MAUVE, TFT_CATPPUCCIN_BASE, true);
  // if (getButtonPressedL()) {
  unsigned int distance = sendPulse();
  tft.drawString("About " + String(distance) + "cm away", 00, 00);
  // }
  // resetButtonL();
  // tft.drawString("KillDozer says Hello", 00, 00); //
}
