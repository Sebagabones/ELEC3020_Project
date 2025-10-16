#define BUTTON_L 0
#define BUTTON_R 14
#include "WiFi.h"

// Globals - these must be set back to false after handling a button press

volatile bool buttonPressedL = false; // Flag to indicate button press
volatile bool buttonPressedR = false; // Flag to indicate button press

const unsigned long debounceDelay = 50; // Debounce delay time (in milliseconds)
volatile unsigned long lastPressTimeL = 0;
volatile unsigned long lastPressTimeR = 0;

bool getButtonPressedL() { return (buttonPressedL); }
bool getButtonPressedR() { return (buttonPressedR); }

void resetButtonL() {
  // Updates if the left button has been pressed down - call after the action
  // that should have been run after left button press to "reset" it
  buttonPressedL = false;
}
void resetButtonR() {
  // Updates if the right button has been pressed down - call after the action
  // that should have been run after right button press to "reset" it
  buttonPressedR = false;
}

void toggleButtonPressedL() {
  if (buttonPressedL) {
    buttonPressedL = false;
  } else {
    buttonPressedL = true;
  }
}

void toggleButtonPressedR() {

  if (buttonPressedR) {
    buttonPressedR = false;
  } else {
    buttonPressedR = true;
  }
}

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

void setupButtonPresses() {
  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(BUTTON_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_L), handleButtonPressL,
                  FALLING); // Set up interrupt on button press (rising edge)
  attachInterrupt(digitalPinToInterrupt(BUTTON_R), handleButtonPressR,
                  FALLING); // Set up interrupt on button press (rising edge)
}
