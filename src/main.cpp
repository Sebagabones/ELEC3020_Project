#include "WiFi.h"
#include <AsyncDelay.h>
#include <Mocha_TFT_eSPI.h>
#include <TFT_eSPI.h>
#include <cstdlib>
#define BUTTON_L 0
#define BUTTON_R 14
#define PIN10 10
#define PIN11 11
#define PIN12 12
#define CHANNEL0 0
#define CHANNEL1 1
#define TOTAL_TIME_PERIOD 20
// ask lab facilaitator about why i needed to set this to 10
#define PWM_RESOLUTION 10
#define INITIAL_FREQ 50
// Define TX and RX pins for UART (change if needed)
#define TXD1 18
#define RXD1 17
AsyncDelay async_delay;
const unsigned long debounceDelay = 50; // Debounce delay time (in milliseconds)
volatile unsigned long lastPressTimeL = 0;
volatile unsigned long lastPressTimeR = 0;

volatile bool buttonPressedL = false; // Flag to indicate button press
volatile bool buttonPressedR = false; // Flag to indicate button press

// Use Serial1 for UART communication
TFT_eSPI tft = TFT_eSPI(170, 320); // Init screen size

// The max duty cycle value based on PWM resolution (will be 255 if resolution
// is 8 bits)  borrowed from here,
// https://lastminuteengineers.com/esp32-pwm-tutorial/ - makes sense, just nice
// to not have it hardcoded, and only run it once
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);
String incoming;

HardwareSerial mySerial(2);
void IRAM_ATTR handleButtonPressL() {
  unsigned long currentTime = millis();
  if (currentTime - lastPressTimeL > debounceDelay) {
    lastPressTimeL = currentTime;
    buttonPressedL = true; // Set the flag when button is pressed
  }
}

void IRAM_ATTR handleButtonPressR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPressTimeR > debounceDelay) {
    lastPressTimeR = currentTime;
    buttonPressedR = true; // Set the flag when button is pressed
  }
}

float getDutyCycleFromUptime(
    double uptime) { // duty cycle = 100 * Uptime/total time
  float dutyCycle = MAX_DUTY_CYCLE * (uptime / TOTAL_TIME_PERIOD);
  return (dutyCycle);
}

double getDowntimeFromUptime(double uptime) {
  return (TOTAL_TIME_PERIOD - uptime);
}
float slope(int x1, int y1, int x2, int y2) { return (y2 - y1) / (x2 - x1); }
void setup() {
  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(BUTTON_R, INPUT_PULLUP);
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1); // UART setup
  tft.init();                                   // Display init
  tft.fillScreen(TFT_CATPPUCCIN_BASE);          // Clear screen

  async_delay.start(500, AsyncDelay::MILLIS);
  ledcSetup(CHANNEL0, INITIAL_FREQ, PWM_RESOLUTION);
  ledcSetup(CHANNEL1, INITIAL_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PIN11, CHANNEL0);
  ledcAttachPin(PIN10, CHANNEL1);
  // Debouncing
  attachInterrupt(digitalPinToInterrupt(BUTTON_L), handleButtonPressL,
                  RISING); // Set up interrupt on button press (falling edge)
  attachInterrupt(digitalPinToInterrupt(BUTTON_R), handleButtonPressR,
                  RISING); // Set up interrupt on button press (falling edge)
}

void loop() {
  // static int state = 0, L_old = 1, R_old = 1, L, R;
  static int L, R = 0;
  static double uptime = 0.6;
  static int upper, lower;
  static int x1, x2;
  static String message;

  int lookuptable[19] = {222, 206, 190, 176, 162, 149, 141, 134, 125, 119,
                         115, 109, 105, 102, 98,  95,  91,  88,  85};
  double downtime = getDowntimeFromUptime(uptime);
  int averageSum = 0;
  for (int l = 0; l < 1000; l++) {
    averageSum += map(analogRead(PIN12), 0, 4095, 0, 255);
  }
  int average = averageSum / 1000;

  // int analogValue = analogRead(PIN12);
  int analogValue = average;

  // Serial.println("analogVal : " + String(analogValue));

  for (int i = 0; i <= 18; i++) {
    int j = i + 1;
    if (lookuptable[i] > analogValue) {
      // Serial.println("lookuptable[j] : " + String(lookuptable[j]));

      if (lookuptable[j] < analogValue) {
        upper = i, lower = j;
        break;
      }
    }
  }
  x1 = upper + 8;
  x2 = lower + 8;

  float slopeVal = slope(x1, lookuptable[upper], x2, lookuptable[lower]);

  float c = lookuptable[upper] - (slopeVal * x1);
  // Serial.println("c: " + String(c));

  int distanceVal = ((analogValue - c) / slopeVal);

  L = buttonPressedL;
  R = buttonPressedR;

  tft.setTextSize(1); // Text size [1..7]
  tft.setTextColor(TFT_CATPPUCCIN_SKY, TFT_CATPPUCCIN_BASE, true);
  tft.drawString("Button Control (pin 10): ", 00, 00);

  tft.setTextColor(TFT_CATPPUCCIN_GREEN, TFT_CATPPUCCIN_BASE,
                   true); // color foreground/background

  tft.drawString("U: " + String(uptime), 00, 10);
  tft.setTextColor(TFT_CATPPUCCIN_RED, TFT_CATPPUCCIN_BASE,
                   true); // color foreground/background
  tft.drawString("D: " + String(downtime), 00, 20);
  tft.setTextColor(TFT_CATPPUCCIN_MAROON, TFT_CATPPUCCIN_BASE,
                   true); // color foreground/background
  tft.drawString("Duty Cycle: " + String(getDutyCycleFromUptime(uptime)) + " ",
                 00, 30);

  tft.setTextColor(TFT_CATPPUCCIN_SKY, TFT_CATPPUCCIN_BASE, true);
  tft.drawString("Potentiometer (pin 11): ", 00, 50);
  tft.setTextColor(TFT_CATPPUCCIN_PEACH, TFT_CATPPUCCIN_BASE,
                   true); // color foreground/background
  // tft.setTextSize(2);     // Text size [1..7]

  // tft.drawString(String(average) + "   ", 00, 60);
  tft.setTextColor(TFT_CATPPUCCIN_MAUVE, TFT_CATPPUCCIN_BASE,
                   true); // color foreground/background
  // tft.drawString("Scaled Output Value: " + String(scaledPotentiometer) + " ",
  //                00, 70);
  tft.drawString("Scaled Output Value: " + String(distanceVal) + " ", 00, 70);

  if (L) {
    uptime = std::round(uptime * 100) / 100;
    if (uptime >= 0.7) {
      uptime -= 0.1;
    }
    Serial.println("L pressed");
    Serial.println("Uptime is now: " + String(uptime));
    Serial.println("DutyCycle is now: " +
                   String(getDutyCycleFromUptime(uptime)));
    buttonPressedL = false;
  }
  if (R) {
    uptime = std::round(uptime * 100) / 100;
    if (uptime <= 2.2) {
      uptime += 0.1;
    }
    Serial.println("R pressed");
    Serial.println("Uptime is now: " + String(uptime));
    Serial.println("DutyCycle is now: " +
                   String(getDutyCycleFromUptime(uptime)));

    buttonPressedR = false;
  }

  ledcWrite(CHANNEL1, getDutyCycleFromUptime(uptime)); //
  int scaledPotentiometer = map(average, 0, 255, 0, 1024) / 10;
  Serial.println("DutyCycle is now: " + String(scaledPotentiometer));
  // if (mySerial.available()) {
  mySerial.println(String(scaledPotentiometer));
  // }
  // ledcWrite(CHANNEL0, scaledPotentiometer);

  tft.setTextColor(TFT_CATPPUCCIN_PINK, TFT_CATPPUCCIN_BASE, true);
  tft.setTextSize(2); // Text size [1..7]

  // L = buttonPressedL;
  // R = buttonPressedR;

  // Check if data is available to read

  // increment the counter

  // if (mySerial.available()) {
  //   // Read data and display it
  //   // tft.drawString("", 00, 00);
  //
  //   message = mySerial.readStringUntil('\n');
  //
  //   if (message.toInt() == 1 && host) {
  //     // not the master anymore
  //     // tft.setCursor(00, 80);
  //     // tft.print("Now the client           ");
  //     host = false;
  //     started = true;
  //   }
  //   Serial.println("Received: " + message);
  //   tft.setCursor(00, 00);
  //   tft.print("Received: " + message + "   ");
  // }
  // if (async_delay.isExpired() && started) {
  //   mySerial.println(String((message.toInt()) + 1));
  //   Serial.println("Sent: " + String(message.toInt() + 1));
  //   if (started) {
  //     async_delay.restart();
  //   }
  // }
  //
  // tft.drawString("Sent: " + String(message.toInt() + 1) + "   ", 00, 20);
  // tft.setTextColor(TFT_CATPPUCCIN_MAUVE, TFT_CATPPUCCIN_BASE, true);
  //
  // if (L && !started) {
  //   mySerial.println(String(1));
  //   Serial.println("Sent: " + String(1));
  //   // tft.drawString("Bossy af            ", 00, 80);
  //   host = true;
  //   started = true;
  //   buttonPressedL = false;
  // }
  // if (R && host) {
  //   while (true) {
  //     tft.fillScreen(TFT_CATPPUCCIN_BASE); // Clear screen
  //     sleep(5);
  //   }
  //   buttonPressedR = false;
  // }
  // if (host && started) {
  //   tft.drawString("Host            ", 00, 80);
  //
  // } else if (!host && started) {
  //   tft.drawString("Client          ", 00, 80);
  // }
}
