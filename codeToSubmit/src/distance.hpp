#include "WiFi.h"

float GetDistance(int SensorPin) {
  /**
   * Gets the distance reading from the ultrasonic sensor from the specified pin
   *
   * @param SensorPin the ping to get the distance reading from
   * @return distance (in cm)
   */
  float millivolts = analogReadMilliVolts(SensorPin);
  // Serial.println(String(millivolts));

  float distance = millivolts * 520 / 4095.0;
  return (distance);
}
