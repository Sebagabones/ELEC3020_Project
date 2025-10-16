#ifndef TOFWRAPPER_H
#define TOFWRAPPER_H
#include <VL53L0X.h>
#include <Wire.h>
class ToFSensor {
public:
  ToFSensor(TwoWire *wire, int interuptPin, int timingBudget, int longRange);
  int readSensor();
  void setSensorI2CAddress(int newAddress);
  int getSensorI2CAddress();
  int lastDistanceMeasured();
  int isInitalised;

private:
  VL53L0X sensor;
};
#endif
