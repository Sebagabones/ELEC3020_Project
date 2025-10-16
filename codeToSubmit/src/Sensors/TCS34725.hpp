#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H
#include <Adafruit_TCS34725.h>
#define TCS34725_ADDRESS (0x29) /**< I2C address **/

Adafruit_TCS34725 ColourSensor(TwoWire *wire, int integrationTime,
                               tcs34725Gain_t gain) {
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(integrationTime, gain);
  bool began = tcs.begin(TCS34725_ADDRESS, wire);
  return (tcs);
}

#endif
