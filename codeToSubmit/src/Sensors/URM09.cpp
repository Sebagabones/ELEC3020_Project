/*!
 * @file URM09.cpp
 * @brief Define the basic structure of URM09 class, the implementation
 * of basic method
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author ZhixinLiu(zhixin.liu@dfrobot.com)
 * @version V1.2
 * @date 2021-09-30
 * @url https://github.com/DFRobot/URM09
 */
#include "URM09.h"
TwoWire WireTwo = TwoWire(0);

URM09::URM09() {}

URM09::~URM09() {}
bool URM09::begin(uint8_t address) {
  this->_addr = address;
  WireTwo.begin(43, 44, 400000);
  WireTwo.beginTransmission(_addr);
  if (WireTwo.endTransmission() == 0)
    return true;
  return false;
}

void URM09::setModeRange(uint8_t range, uint8_t mode) {
  txbuf[0] = (uint8_t)(range | mode);
  i2cWriteTemDistance(eCFG_INDEX, &txbuf[0], 1);
}

void URM09::measurement() {
  txbuf[0] = CMD_DISTANCE_MEASURE;
  i2cWriteTemDistance(eCMD_INDEX, &txbuf[0], 1);
}

float URM09::getTemperature() {
  uint8_t i = 0;
  uint8_t rxbuf[10] = {0};
  WireTwo.beginTransmission(_addr);
  WireTwo.write(eTEMP_H_INDEX);
  WireTwo.endTransmission();
  WireTwo.requestFrom(_addr, (uint8_t)2);
  while (WireTwo.available()) {
    rxbuf[i++] = WireTwo.read();
  }
  return (((int16_t)rxbuf[0] << 8) + rxbuf[1]) / 10;
}

int16_t URM09::getDistance() {
  uint8_t i = 0;
  uint8_t rxbuf[10] = {0};

  WireTwo.beginTransmission(_addr);
  WireTwo.write(eDIST_H_INDEX);
  WireTwo.endTransmission();
  WireTwo.requestFrom(_addr, (uint8_t)2);
  while (WireTwo.available()) {
    rxbuf[i++] = WireTwo.read();
  }
  return ((int16_t)rxbuf[0] << 8) + rxbuf[1];
}

void URM09::i2cWriteTemDistance(uint8_t Reg, uint8_t *pdata, uint8_t datalen) {
  WireTwo.beginTransmission(_addr);
  WireTwo.write(Reg);
  for (uint8_t i = 0; i < datalen; i++) {
    WireTwo.write(pdata[i]);
  }
  WireTwo.endTransmission();
}

int16_t URM09::scanDevice() {
  WireTwo.begin(43, 44, 400000); // SDA, SCL
  uint8_t error, address;
  for (address = 1; address < 127; address++) {
    WireTwo.beginTransmission(address);
    error = WireTwo.endTransmission();
    if (error == 0) {
      return address;
    }
  }
  return -1;
}

void URM09::modifyI2CAddress(uint8_t Address) {
  txbuf[0] = Address;
  i2cWriteTemDistance(eSLAVEADDR_INDEX, &txbuf[0], 1);
}

uint8_t URM09::getI2CAddress() {
  uint8_t i = 0;
  uint8_t rxbuf[10] = {0};
  WireTwo.beginTransmission(_addr);
  WireTwo.write(eSLAVEADDR_INDEX);
  WireTwo.endTransmission();
  WireTwo.requestFrom(_addr, (uint8_t)1);
  while (WireTwo.available()) {
    rxbuf[i++] = WireTwo.read();
  }
  return rxbuf[0];
}
