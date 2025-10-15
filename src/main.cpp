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
TFT_eSPI tft = TFT_eSPI(170, 320); // Init screen size
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

#define TIME_TO_TURN_AT_EDGE 50
#define TIME_TO_TURN_90_DEGREE 500
///////////////////////////////////////////////

///////////////////////////////////////////////
// TCS (I2C multiplexer)
#define TOF_I2C_NUMBER 2
#define FRONT_RIGHT_TCS_I2C_NUMBER 3
#define FRONT_LEFT_TCS_I2C_NUMBER 4

#define TCAADDR 0x70

//// TCA Helper function
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
TaskHandle_t interruptHandlerHandleFR = nullptr;
TaskHandle_t interruptHandlerHandleFL = nullptr;
TaskHandle_t interruptHandlerHandleBoth = nullptr;

volatile bool restartTask = false;
///////////////////////////////////////////////

///////////////////////////////////////////////
// I2C stuff:
TwoWire WireOne = TwoWire(1);
// TwoWire WireTwo = TwoWire(1);
///////////////////////////////////////////////

// UltraSonic sonic1 = UltraSonic(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);

///////////////////////////////////////////////
// Prephials
//// Motors
Motor1 rightMotor = Motor1(MOTOR1_PIN_A, MOTOR1_PIN_B);
Motor2 leftMotor = Motor2(MOTOR2_PIN_A, MOTOR2_PIN_B);
//// Sensors
int Ultra1Pin = ULTRA1; // select the input pin for the first ultrasonic sensor
int Ultra2Pin = ULTRA2; // select the input pin for the second ultrasonic sensor

ToFSensor *tof1;          // Time of Flight Sensors
Adafruit_TCS34725 tcs_FR; // Front Right RGB Sensor
Adafruit_TCS34725 tcs_FL; // Front Left RGB Sensor
                          //
///////////////////////////////////////////////

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
  for (;;) {
    static bool searching = true, robotFound = false;
    // static int state = 0, L_old = 1, R_old = 1, L, R;
    tft.setTextColor(TFT_CATPPUCCIN_RED);
    tft.setTextSize(2);
    tft.drawString("Searching ", 85, 160);

    searching = searchFunction();
    tft.setTextColor(TFT_CATPPUCCIN_GREEN);
    tft.drawString("Found     ", 85, 160);

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
}

/**
 *@brief Makes sure the robot doesn't drive out of the circle going forwards
 *when right rgb sensor detects black
 *
 *@details Higher priority task on the main core (core 0) - gets called by
 *`Core1_CircleDetectionFront()` and takes over from `Core0_MainTask()`
 *
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core0_FrontRightCircleInterruptHandler(void *pvParameters) {
  // here we will want to make it turn, until both are in the circle.
  // Sleep until Core1 triggers it
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    tft.drawString("PANIK @ FR     ", 85, 160);

    turn(TIME_TO_TURN_AT_EDGE, LEFT);

    vTaskDelay(1);
  }
  restartTask = true;
}

/**
 *@brief Makes sure the robot doesn't drive out of the circle going forwards
 *when left rgb sensor detects black
 *
 *@details Higher priority task on the main core (core 0) - gets called by
 *`Core1_CircleDetectionFront()` and takes over from `Core0_MainTask()`
 *
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core0_FrontLeftCircleInterruptHandler(void *pvParameters) {
  // here we will want to make it turn, until both are in the circle.
  // Sleep until Core1 triggers it
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    tft.drawString("PANIK @ FL     ", 85, 160);

    turn(TIME_TO_TURN_AT_EDGE, RIGHT);

    vTaskDelay(1);
  }
  restartTask = true;
}

/**
 *@brief Makes sure the robot doesn't drive out of the circle going forwards
 *when both rgb sensors on the front detect black
 *
 *@details Higher priority task on the main core (core 0) - gets called by
 *`Core1_CircleDetectionFront()` and takes over from `Core0_MainTask()`
 *
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core0_FrontBothCircleInterruptHandler(void *pvParameters) {
  // here we will want to make it turn, until both are in the circle.
  // Sleep until Core1 triggers it
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    tft.drawString("PANIK @ BOTH   ", 85, 160);

    turn(TIME_TO_TURN_90_DEGREE, LEFT);

    vTaskDelay(1);
  }
  restartTask = true;
}

/**
 * Runs on Core 1 (the *non*-main core), constantly checks to see if we are
 * at the edge of the circle, if we are, panik (and call
 * `Core0_FrontRightCircleInterruptHandler()`/`Core0_FrontLeftCircleInterruptHandler()`
 * to interupt `Core0_MainTask()`
 *
 * @param pvParameters parameters for the task, see `setup()`
 */
void Core1_CircleDetectionFront(void *pvParameters) {
  int lastCallTime = millis(); // just for testing, remove
  for (;;) {
    float rFR, gFR, bFR, rFL, gFL, bFL;
    float totalFR, totalFL;
    tcaselect(FRONT_RIGHT_TCS_I2C_NUMBER);
    tcs_FR.getRGB(&rFR, &gFR, &bFR);
    tcaselect(FRONT_LEFT_TCS_I2C_NUMBER);
    tcs_FL.getRGB(&rFL, &gFL, &bFL);
    totalFR = rFR + bFR + gFR;
    totalFL = rFL + bFL + gFL;

    if (totalFR < 350 && totalFL < 350) { // both are probably not white
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTaskNotifyFromISR(interruptHandlerHandleBoth, 0, eNoAction,
                         &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
    } else if (totalFR < 350) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTaskNotifyFromISR(interruptHandlerHandleFR, 0, eNoAction,
                         &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
    } else if (totalFL < 350) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTaskNotifyFromISR(interruptHandlerHandleFL, 0, eNoAction,
                         &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
    }
    vTaskDelay(
        20); // May want to keep this but change the timing, dont want to
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

  setupButtonPresses();

  tft.init(); // Display init

  tft.setRotation(90);
  tft.fillScreen(TFT_CATPPUCCIN_BASE); // Clear screen

  ///////////////////////////////////////////////
  // I2C stuff:
  WireOne.begin(WIRE1_I2C_PIN_SDA, WIRE1_I2C_PIN_SCL, 400000);

  //
  // I2Cscan(&WireOne);
  ///////////////////////////////////////////////
  // tcaselect(TOF_I2C_NUMBER);
  // tof1 = new ToFSensor(&WireOne, TOF_INTERUPT_PIN, 200000,
  // 0); // Set for high accuracy at cost of speed
  tcaselect(FRONT_RIGHT_TCS_I2C_NUMBER);
  tcs_FR =
      ColourSensor(&WireOne, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
  tcaselect(FRONT_LEFT_TCS_I2C_NUMBER);
  tcs_FL =
      ColourSensor(&WireOne, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
  tft.fillScreen(TFT_CATPPUCCIN_BASE); // Clear screen

  // Core 0 main work (low priority)
  xTaskCreatePinnedToCore(Core0_MainTask, "MainTask", 4096, nullptr, 2,
                          &mainTaskHandle, 0);

  // Core 0 Front Right interrupt handler
  xTaskCreatePinnedToCore(Core0_FrontRightCircleInterruptHandler,
                          "InterruptHandlerFR", 4096, nullptr, 4,
                          &interruptHandlerHandleFR, 0);

  // Core 0 Front Left intterupt handler
  xTaskCreatePinnedToCore(Core0_FrontLeftCircleInterruptHandler,
                          "InterruptHandlerFL", 4096, nullptr, 4,
                          &interruptHandlerHandleFL, 0);

  xTaskCreatePinnedToCore(Core0_FrontBothCircleInterruptHandler,
                          "InterruptHandlerBoth", 4096, nullptr, 5,
                          &interruptHandlerHandleBoth, 0);

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
