#include "MotorControl/MotorControl.hpp"
#include "Sensors/HC-SR04.hpp"
#include "Sensors/TCS34725.hpp"
#include "Sensors/VLX53L0.hpp"
#include "WiFi.h"
#include "debouncing.hpp"
#include "getHex.hpp"
#include "i2cBusWire.hpp"
#include <Adafruit_TCS34725.h>
#include <AsyncDelay.h>
#include <Mocha_TFT_eSPI.h>
#include <Wire.h>

#include <TFT_eSPI.h>
#include <cstdlib>

#define TRIGGER_PIN_1 21
#define ECHO_PIN_1 16
#define MAX_DISTANCE 100
#define MOTOR1_PIN_A 10
#define MOTOR1_PIN_B 11
#define MOTOR2_PIN_A 12
#define MOTOR2_PIN_B 13

#define turn90Counter 1000
///////////////////////////////////////////////
// I2C stuff:
// #define WIRE1_I2C_PIN_SDA 18
// #define WIRE1_I2C_PIN_SCL 17
///////////////////////////////////////////////

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

///////////////////////////////////////////////
// I2C stuff:
// TwoWire WireOne = TwoWire(1);
///////////////////////////////////////////////

// TwoWire WireTwo = TwoWire(1);

UltraSonic sonic1 = UltraSonic(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
Motor1 motor1 = Motor1(MOTOR1_PIN_A, MOTOR1_PIN_B);
Motor2 motor2 = Motor2(MOTOR2_PIN_A, MOTOR2_PIN_B);

// ToFSensor *tof1;

// Adafruit_TCS34725 colourSensor;

// Adafruit_TCS34725 tcs;
// void leftTurn() {
//   for (int i = 0; i < turn90Counter; i++) {
//     motor1.setSpeedDir(255, 1);
//     motor2.setSpeedDir(255, 0);
//   }
// } // This is janky af, find a better way

void setup() {

  Serial.begin(115200);
  Serial.println("Starting setup...");
  // wireSetup(WireOne, WIRE1_I2C_PIN_A, WIRE1_I2C_PIN_B, 400000);
  // Serial.println("WireOne initialized");

  // need to but using as example

  setupButtonPresses();
  // setupHCSR();

  tft.init();                          // Display init
  Serial.println("TFT initialized");   //
  tft.fillScreen(TFT_CATPPUCCIN_BASE); // Clear screen
  Serial.println("Screen filled");
  ///////////////////////////////////////////////
  // I2C stuff:
  // WireOne.begin(WIRE1_I2C_PIN_SDA, WIRE1_I2C_PIN_SCL, 400000);
  //
  // Wire.begin(43, 44); // SDA (21), SCL (22) on ESP32, 400 kHz rate
  //
  // I2Cscan(&WireOne);
  ///////////////////////////////////////////////

  // tof1 = new ToFSensor(&WireOne, TOF_INTERUPT_PIN, 100000,
  //                      0); // Set for high accuracy at cost of speed
  // colourSensor = ColourSensor(&WireOne, 0x12, TCS34725_INTEGRATIONTIME_50MS,
  //                             TCS34725_GAIN_1X);

  ///////////////////////////////////////////////
  // ColourSensor
  // tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
  ///////////////////////////////////////////////
}

void loop() {

  // static int state = 0, L_old = 1, R_old = 1, L, R;
  tft.setTextColor(TFT_CATPPUCCIN_MAUVE, TFT_CATPPUCCIN_BASE, true);
  // if (getButtonPressedL()) {
  unsigned int distance = sonic1.sendPulse();
  int distanceToF = -1;
  // if (tof1->isInitalised) {
  // int distanceToF = tof1->readSensor();
  // Serial.println("distance to ToF is: " + String(distanceToF));
  // }
  // tft.drawString("About " + String(distance) + "cm away", 00, 00);
  // tft.drawString("ToF says " + String(distanceToF) + "cm away", 00, 10);

  if (distance) {
    if (distance < 15) {
      tft.drawString("STOOOOOP   ", 00, 00);
      motor1.setSpeedDir(255, !signbit(1));
      // motor1.setSpeedDir(255, 1);

      motor2.setSpeedDir(255, signbit(1));
      // // tft.drawString("Turning left   ", 00, 10);
      // // leftTurn();
      // // tft.drawString("Turned left   ", 00, 10);
      // Then turn
    } else {
      tft.drawString("GOOOOOOOOO  ", 00, 00);
      signed int speed = map(distance, 10, 100, 30, 255);
      tft.drawString("                ", 00, 10);

      if (speed > 50) { // Send it
        motor1.setSpeedDir(255, 1);
        motor2.setSpeedDir(255, 1);
      } else {
        motor1.setSpeedDir(abs(speed), !signbit(speed));
        motor2.setSpeedDir(abs(speed), !signbit(speed));
      }
    }
  }
  // else {
  //   motor1.setSpeedDir(0, 0);
  //   motor2.setSpeedDir(0, 0);
  // }
  ///////////////////////////////////////////////
  // ColourSensor
  // uint16_t c, colorTemp, lux;
  // float r, g, b;
  // // tcs.getRawData(&r, &g, &b, &c);
  // tcs.getRGB(&r, &g, &b);
  // int red = (int)r;
  // int green = (int)g;
  // int blue = (int)b;
  //
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  // lux = tcs.calculateLux(r, g, b);
  // char hexColour[8];
  // std::snprintf(hexColour, sizeof hexColour, "#%02x%02x%02x", red, green,
  // blue);
  //
  // // int hexVal = (int)get_hex(red, green, blue);
  //
  // Serial.print("#");
  // Serial.print(String(hexColour));
  // Serial.println("");
  // Serial.print("R: ");
  // Serial.print(r, DEC);
  // Serial.print(" ");
  // Serial.print("G: ");
  // Serial.print(g, DEC);
  // Serial.print(" ");
  // Serial.print("B: ");
  // Serial.print(b, DEC);
  // Serial.print(" ");
  // Serial.print("C: ");
  // Serial.print(c, DEC);
  // Serial.print(" ");
  // Serial.println("");
  ///////////////////////////////////////////////
}
