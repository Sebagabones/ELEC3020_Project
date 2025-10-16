#include "VLX53L0.hpp"
#include "Wire.h"

ToFSensor::ToFSensor(TwoWire *wire, int interuptPin, int timingBudget = 30000,
                     int longRange = 0) {
  // Timing budget is in ms
  // default is 30ms (30000)
  // high speed is 20ms
  // high accuracy is 200000
  // Set longRange to 1 to enable long range mode. This
  // increases the sensitivity of the sensor and extends its
  // potential range, but increases the likelihood of getting
  // an inaccurate reading because of reflections from objects
  // other than the intended target. It works best in dark
  // conditions.
  sensor.setBus(wire);

  sensor.init();

  sensor.setTimeout(500);

  if (longRange) {
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  }

  sensor.setMeasurementTimingBudget(timingBudget);

  isInitalised = 1;
  Serial.println("tof initalised");

  sensor.startContinuous(300);
  pinMode(interuptPin, INPUT);
}

int ToFSensor::readSensor() {
  Serial.println("Reading tof sensor");
  int distance = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.println("tof timeout");
    sensor.init();
  }
  // Serial.println(distance);

  return (distance);
}
void ToFSensor::setSensorI2CAddress(int newAddress) {
  sensor.setAddress(newAddress);
}

int ToFSensor::getSensorI2CAddress() { return sensor.getAddress(); }
