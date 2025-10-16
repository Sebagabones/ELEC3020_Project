// Can have up to two Wires at the same time
// Go read this
// https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
#include <Wire.h>

void wireSetup(TwoWire &I2CInstance, int i2c_sda, int i2c_scl,
               int frequency = 100000) { // unless you know what you are
                                         // doing here, be careful,
                                         // shit is janky af
                                         // WireNumber should be either 0 or 1

  I2CInstance.begin(i2c_sda, i2c_scl, frequency);
}

// // I2C scan function
void I2Cscan(TwoWire *wire) {
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    wire->beginTransmission(address);
    error = wire->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
