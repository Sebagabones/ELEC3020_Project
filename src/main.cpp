#include "MotorControl/MotorControl.hpp"
#include "Sensors/HC-SR04.hpp"
#include "Sensors/TCS34725.hpp"
#include "Sensors/VLX53L0.hpp"
#include "WiFi.h"
#include "debouncing.hpp"
#include "distance.hpp"
#include "getHex.hpp"
#include "i2cBusWire.hpp"
#include "interupts.cpp"
#include <Adafruit_TCS34725.h>
#include <AsyncDelay.h>
#include <DFRobot_URM09.h>
#include <Mocha_TFT_eSPI.h>
#include <Wire.h>

#include <TFT_eSPI.h>
#include <cstdlib>

// TODO:
#define turnDirection                                                          \
  1 // update this to turn depending on which button was pressed

/* TODO
   - RGB sensors connect to task that checks them
   - ToF sensors logic/connection
   - Driving logic (partly done)
   - Drive outside of bounds logic
   - cycle back to the start of the program  (probably want to check that a
   robot is directly in front of us and within like, 5 cm and that we have both
   front sensors touching the edge, and if it is, start program again after
   turning around
   - logic for if we cannot find anyone after a few full turns then think of
   something else to do
   - reactions for if about to be pushed out
   - LEDs
   - Testing

 */

///////////////////////////////////////////////
// Pins
#define PIN_POWER_ON 15 // LCD and battery Power Enable
#define PIN_LCD_BL 38   // BackLight enable pin (see Dimming.txt)

#define ULTRA1 18
#define ULTRA2 17

#define TOF_INTERUPT_PIN 3

#define MOTOR1_PIN_A 10
#define MOTOR1_PIN_B 11
#define MOTOR2_PIN_A 12
#define MOTOR2_PIN_B 13
///////////////////////////////////////////////

///////////////////////////////////////////////
// Directions
#define FORWARDS 1
#define BACKWARDS 0
#define LEFT 0
#define RIGHT 1
///////////////////////////////////////////////

///////////////////////////////////////////////
// TCS
#define TOF_I2C_NUMBER 2
#define FRONT_RIGHT_TCS_I2C_NUMBER 3
#define TCAADDR 0x70

// TCA Helper function
void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
///////////////////////////////////////////////

///////////////////////////////////////////////
// I2C stuff:
#define WIRE1_I2C_PIN_SDA 18
#define WIRE1_I2C_PIN_SCL 17
///////////////////////////////////////////////

// #define CHANNEL0 0
// #define CHANNEL1 1
// #define TOTAL_TIME_PERIOD 20

// #define PWM_RESOLUTION 10
// #define INITIAL_FREQ 50
// Define TX and RX pins for UART (change if needed)
// #define TXD1 18
// #define RXD1 17
// AsyncDelay async_delay;

// Use Serial1 for UART communication

///////////////////////////////////////////////
/// Tasks
TaskHandle_t mainTaskHandle = nullptr;
TaskHandle_t interruptHandlerHandle = nullptr;
volatile bool restartTask = false;
///////////////////////////////////////////////

TFT_eSPI tft = TFT_eSPI(170, 320); // Init screen size

///////////////////////////////////////////////
// I2C stuff:
TwoWire WireOne = TwoWire(1);
// TwoWire WireTwo = TwoWire(1);
///////////////////////////////////////////////

// UltraSonic sonic1 = UltraSonic(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
Motor1 rightMotor = Motor1(MOTOR1_PIN_A, MOTOR1_PIN_B);
Motor2 leftMotor = Motor2(MOTOR2_PIN_A, MOTOR2_PIN_B);
int Ultra1Pin = ULTRA1; // select the input pin
int Ultra2Pin = ULTRA2; // select the input pin
                        //
ToFSensor *tof1;

Adafruit_TCS34725 tcs_FR;
// void leftTurn() {
//   for (int i = 0; i < turn90Counter; i++) {
//     rightMotor.setSpeedDir(255, 1);
//     leftMotor.setSpeedDir(255, 0);
//   }
// } // This is janky af, find a better way
///////////////////////////////////////////////
struct ultraDistances {
  double left;
  double right;
};

ultraDistances getDistancesAverage(int numOfAvg = 5) {
  // get an average reading from both sensors
  double distanceR = 0;
  double distanceL = 0;
  for (int i = 0; i < numOfAvg; i++) {
    distanceR += GetDistance(Ultra1Pin);
    distanceL += GetDistance(Ultra2Pin);
  }
  ultraDistances ultraDistanceValues;
  ultraDistanceValues.left = distanceL / numOfAvg;
  ultraDistanceValues.right = distanceR / numOfAvg;
  return (ultraDistanceValues);
}

/**
 * @brief Turns the robot
 * @details Takes in a time (in `ms`) to turn for, and the direction, and then
 * does a zero turn (ie, one motor goes backwards and the other goes forwards)
 * for that period of time.
 *
 * @param timeToTurnFor The time in milliseconds to turn the robot for - later
 * once we have done testing we can convert this to the number of degrees to
 * turn
 * @param direction The direction to turn - `0` is left, `1` is right
 * @param speed The speed to turn at - defaults to 255
 */
void turn(int timeToTurnFor, int direction, int speed = 255) {
  unsigned long initalMillis = millis();
  if (direction) { // Turn right
    unsigned long currentMillis = millis();
    while (currentMillis - initalMillis < timeToTurnFor) {
      rightMotor.setSpeedDir(speed, BACKWARDS);
      // rightMotor.setSpeedDir(255, 1);
      leftMotor.setSpeedDir(speed, FORWARDS);
      currentMillis = millis();
      vTaskDelay(2);
    }
  } else { // Turn left
    unsigned long currentMillis = millis();
    while (currentMillis - initalMillis < timeToTurnFor) {
      rightMotor.setSpeedDir(speed, FORWARDS);
      // rightMotor.setSpeedDir(255, 1);
      leftMotor.setSpeedDir(speed, BACKWARDS);
      currentMillis = millis();
      vTaskDelay(2);
    }
  }
}

bool searchFunction() {
  ultraDistances distances = getDistancesAverage(5);
  while (distances.right > 160 || distances.left > 160) {
    turn(50, turnDirection, 125);
    vTaskDelay(5); // 10 ms delay, yields to other tasks
  }
  return (true);
}
/**
 * @brief Returns the difference in values between the reading of the left
 * ultrasonic sensors and the right ultrasonic sensors
 *
 *
 * @return signed int, `distance.left - distance.right`
 */
signed int getDirection(int avg = 5) {
  ultraDistances distances = getDistancesAverage(avg);
  return (distances.left - distances.right);
}

/**
 * @brief Drive forwards, with possible bias
 *
 * @param timeToTurnFor same as in `turn()`
 * @param speed speed to drive forwards at
 * @param turnBias This is the bias at which to turn - basically difference in
 * speed between motors. negative biases towards the right, positive biases
 * towards the left, defaults to zero, keep between -163 <= x <= 163
 */
void driveForwards(int timeToTurnFor, int speed = 255,
                   signed int turnBias = 0) {
  int speedL = speed;
  int speedR = speed;
  if (turnBias) {
    speedR = speed + (turnBias / 2);
    speedL = speed - (turnBias / 2); // errr, hopefully that works lmao
  }
  unsigned long initalMillis = millis();
  unsigned long currentMillis = millis();
  while (currentMillis - initalMillis < timeToTurnFor) {
    rightMotor.setSpeedDir(speedR, FORWARDS);
    leftMotor.setSpeedDir(speedL, FORWARDS);
    currentMillis = millis();
    vTaskDelay(2);
  }
}

/**
 * @brief Effectively the main looping function
 * @details This is the "main" task - pretty much it is `loop()` - important
 * tasks like if we detect the edge of the circle will interuppt this task
 * with a higher priority.
 *
 * see below for other tasks
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core0_MainTask(void *pvParameters) {
  static bool searching = true, robotFound = false;
  // static int state = 0, L_old = 1, R_old = 1, L, R;
  searching = searchFunction();
  while (!restartTask) {
    signed int direction = getDirection();
    if (abs(direction) > 5) {
      driveForwards(10, 200, direction);
    } else {
      driveForwards(10, 255);
    }
    vTaskDelay(10); // 10 ms delay, yields to other tasks
  }
  vTaskDelay(10); // 10 ms delay, yields to other tasks
}

/**
 *@brief Makes sure the robot doesn't drive out of the circle going forwards
 *
 *@details Higher priority task on the main core (core 0) - gets called by
 *`Core1_CircleDetectionFront()` and takes over from `Core0_MainTask()`
 *
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core0_FrontCircleInterruptHandler(void *pvParameters) {
  // here we will want to make it turn, until both are in the circle.
  // Sleep until Core1 triggers it
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // Serial.printf("Core0: *** INTERRUPT HANDLER RUNNING ***\n");

  // vTaskDelay(10); // 10 ms delay, yields to other tasks
  unsigned long initalMillis = millis();
  unsigned long currentMillis = millis();
  while (currentMillis - initalMillis <
         500) { // probably want to have a better way of doing this
    // DRIVEEEE
    rightMotor.setSpeedDir(255, !signbit(-1));
    leftMotor.setSpeedDir(255, !signbit(-1));
    currentMillis = millis();
    vTaskDelay(1);
  }
  Serial.printf("Core0: Interrupt handler done, resuming work\n");
  restartTask = true;
}

/**
 * Runs on Core 1 (the *non*-main core), constantly checks to see if we are
 * at the edge of the circle, if we are, panik (and call
 * `Core0_FrontCircleInterruptHandler()` to interupt `Core0_MainTask()`
 *
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core1_CircleDetectionFront(void *pvParameters) {
  int lastCallTime = millis(); // just for testing, remove
  for (;;) {
    ///////////////////////////////////////////////
    /// Replace this section with the RGB detection parts
    Serial.printf("Core1: doing work\n");
    int timeSinceLast = millis() - lastCallTime;
    ///////////////////////////////////////////////

    if (timeSinceLast > 5000) { // likewise, replace
      Serial.println("Core1: Condition met, triggering interrupt on Core0!");
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTaskNotifyFromISR(interruptHandlerHandle, 0, eNoAction,
                         &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
      lastCallTime = millis(); // can be removed
    }
    vTaskDelay(
        10); // May want to keep this but change the timing, dont want to
             // call the task multiple times on the same "occurance" of
             // the edge of the circle, but also don't want to miss it -
             // we will may want to see if there is a way to share
             // the i2c connection between cores, and set it so that we
             // can check if we are out of the edge yet - alternatively, if
             // we have this set to occur at a resonable rate, this could be
             // the way we tell if we are out of the circle yet, dunno
  }
}
/**
 * Ye old `setup()`
 *
 */
void setup() {
  pinMode(PIN_POWER_ON, OUTPUT); // triggers the LCD backlight
  pinMode(PIN_LCD_BL, OUTPUT);   // BackLight enable pin

  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);

  Serial.begin(115200);
  analogSetPinAttenuation(
      Ultra1Pin, ADC_2_5db); // Locks our max range for ultrasonic sensors
                             // to ~163cm - i think this is enough

  analogSetPinAttenuation(
      Ultra2Pin, ADC_2_5db); // Locks our max range for ultrasonic sensors
                             // to ~163cm - i think this is enough
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
  WireOne.begin(WIRE1_I2C_PIN_SDA, WIRE1_I2C_PIN_SCL, 400000);
  // Wire.begin(43, 44); // SDA (21), SCL (22) on ESP32, 400 kHz rate
  //
  // I2Cscan(&WireOne);
  ///////////////////////////////////////////////
  tcaselect(TOF_I2C_NUMBER);
  tof1 = new ToFSensor(&WireOne, TOF_INTERUPT_PIN, 200000,
                       0); // Set for high accuracy at cost of speed
  // colourSensor = ColourSensor(0x12,
  // TCS34725_INTEGRATIONTIME_50MS,
  //                             TCS34725_GAIN_1X);
  tcaselect(FRONT_RIGHT_TCS_I2C_NUMBER);
  tcs_FR =
      ColourSensor(&WireOne, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

  tft.fillScreen(TFT_CATPPUCCIN_BASE); // Clear screen

  // Core 0 main work (low priority)
  xTaskCreatePinnedToCore(Core0_MainTask, "MainTask", 4096, nullptr, 2,
                          &mainTaskHandle, 0);

  // Core 0 interrupt handler (high priority)
  xTaskCreatePinnedToCore(Core0_FrontCircleInterruptHandler, "InterruptHandler",
                          4096, nullptr, 5, &interruptHandlerHandle, 0);

  // Core 1 monitor (medium priority)
  xTaskCreatePinnedToCore(Core1_CircleDetectionFront, "MonitorTask", 4096,
                          nullptr, 3, nullptr, 1);
}

/**
 * Needs to be here, but I think for the most part we will be using tasks?
 *
 * TBH it may not need to be here, idk
 */
void loop() {}
// old main
// tft.setTextColor(TFT_CATPPUCCIN_MAUVE, TFT_CATPPUCCIN_BASE, true);
// // if (getButtonPressedL()) {
// double distanceR = GetDistance(Ultra1Pin);
// double distanceL = GetDistance(Ultra2Pin);
// // vTaskDelay(10); // 10 ms delay, yields to other tasks
//
// tft.drawString(String(distanceR) + "cm R", 110, 100);
// tft.drawString(String(distanceL) + "cm L", 110, 130);
// // vTaskDelay(10); // 10 ms delay, yields to other tasks
//
// double distance = min(distanceL, distanceR);
// int distanceToF = -1;
// // if (tof1->isInitalised) {
// // int distanceToF = tof1->readSensor();
// // Serial.println("distance to ToF is: " + String(distanceToF));
// // }
// // tft.drawString("About " + String(distance) + "cm away", 00, 00);
// // tft.drawString("ToF says " + String(distanceToF) + "cm away", 00,
// // 10);
//
// if (distance) {
//   if (distance < 35) {
//     tft.drawString("STOOOOOP   ", 00, 00);
//     if (distanceL >
//         distanceR + 1) { // turn left, this does bias torwards turning to
//                          // the left, probs a good idea to fix someday
//       turn(200, 0);
//
//     } else { // turn right
//       turn(200, 1);
//     }
//     // // tft.drawString("Turning left   ", 00, 10);
//     // // leftTurn();
//     // // tft.drawString("Turned left   ", 00, 10);
//     // Then turn
//   } else {
//     tft.drawString("GOOOOOOOOO  ", 00, 00);
//     signed int speed = map(distance, 10, 100, 30, 255);
//     tft.drawString("                ", 00, 10);
//
//     if (speed > 50) { // Send it
//       rightMotor.setSpeedDir(255, FORWARDS);
//       leftMotor.setSpeedDir(255, FORWARDS);
//     } else {
//       rightMotor.setSpeedDir(abs(speed), !signbit(speed));
//       leftMotor.setSpeedDir(abs(speed), !signbit(speed));
//     }
//   }
// }
// // else {
// //   rightMotor.setSpeedDir(0, 0);
// //   leftMotor.setSpeedDir(0, 0);
// // }
// ///////////////////////////////////////////////
// // ColourSensor
// // uint16_t c, colorTemp, lux;
// // float r, g, b;
// // // tcs_FR.getRawData(&r, &g, &b, &c);
// // tcs_FR.getRGB(&r, &g, &b);
// // int red = (int)r;
// // int green = (int)g;
// // int blue = (int)b;
// //
// // colorTemp = tcs_FR.calculateColorTemperature(r, g, b);
// // lux = tcs_FR.calculateLux(r, g, b);
// // char hexColour[8];
// // std::snprintf(hexColour, sizeof hexColour, "#%02x%02x%02x", red,
// // green, blue);
// //
// // // int hexVal = (int)get_hex(red, green, blue);
// //
// // Serial.print("#");
// // Serial.print(String(hexColour));
// // Serial.println("");
// // Serial.print("R: ");
// // Serial.print(r, DEC);
// // Serial.print(" ");
// // Serial.print("G: ");
// // Serial.print(g, DEC);
// // Serial.print(" ");
// // Serial.print("B: ");
// // Serial.print(b, DEC);
// // Serial.print(" ");
// // Serial.print("C: ");
// // Serial.print(c, DEC);
// // Serial.print(" ");
// // Serial.println("");
// ///////////////////////////////////////////////
