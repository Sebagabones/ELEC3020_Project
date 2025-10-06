#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 ColourSensor(TwoWire *wire, uint8_t address,
                               int integrationTime, tcs34725Gain_t gain) {
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(integrationTime, gain);
  // bool began = tcs.begin(address, wire);
  return (tcs);
}

#endif
