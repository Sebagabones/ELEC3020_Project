#include "MotorControl/MotorControl.hpp"
#include "Sensors/HC-SR04.hpp"
#include "WiFi.h"
#include "debouncing.hpp"
#include <AsyncDelay.h>
#include <Mocha_TFT_eSPI.h>
#include <TFT_eSPI.h>
#include <cstdlib>

#define TRIGGER_PIN_1 12
#define ECHO_PIN_1 13
#define MAX_DISTANCE 100
#define MOTOR_PIN_A 43
#define MOTOR_PIN_B 44

// #define CHANNEL0 0
// #define CHANNEL1 1
// #define TOTAL_TIME_PERIOD 20
// ask lab facilaitator about why i needed to set this to 10
// #define PWM_RESOLUTION 10
// #define INITIAL_FREQ 50
// Define TX and RX pins for UART (change if needed)
// #define TXD1 18
// #define RXD1 17
// AsyncDelay async_delay;

// Use Serial1 for UART communication
TFT_eSPI tft = TFT_eSPI(170, 320); // Init screen size
UltraSonic sonic1 = UltraSonic(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
Motor1 motor1 = Motor1(MOTOR_PIN_A, MOTOR_PIN_B);
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
  unsigned int distance = sonic1.sendPulse();
  tft.drawString("About " + String(distance) + "cm away", 00, 00);
  if (distance) {
    signed int speed = map(distance - 30, -30, 30, -255, 255);

    tft.drawString("Speed: " + String(speed) + "  ", 00, 10);

    motor1.setSpeedDir(abs(speed), !signbit(speed));
  }
  // }
  // resetButtonL();
  // tft.drawString("KillDozer says Hello", 00, 00); //
}
