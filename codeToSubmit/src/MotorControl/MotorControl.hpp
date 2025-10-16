#ifndef MOTORWRAPPER_H
#define MOTORWRAPPER_H

#include <CytronMotorDriver.h>

class Motor1 {
public:
  Motor1(int pinA, int pinB);

  void
  setSpeedDir(int speed,
              signed int direction); // Speed can be from 0 - 255, direction is
                                     // 0 for backwards, 1 for forwards
  void setSpeed(signed int speed);   // speed can be from -255 to 255 - negative
                                     // means backwards
  void stopMotor();

private:
  CytronMD motor1;
};
class Motor2 {
public:
  Motor2(int pinA, int pinB);

  void
  setSpeedDir(int speed,
              signed int direction); // Speed can be from 0 - 255, direction is
                                     // 0-1 for backwards, 1 for forwards
  void setSpeed(signed int speed);   // speed can be from -255 to 255 - negative
                                     // means backwards
  void stopMotor();

private:
  CytronMD motor2;
};
#endif
