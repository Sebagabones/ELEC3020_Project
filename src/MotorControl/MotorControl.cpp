#include "MotorControl.hpp"
signed int getSpeedForDir(int speed, signed int direction) {
  speed = abs(speed);
  if (direction) {
    return (speed);
  } else { // for some reason the driver takes -255 as backwards of speed 0??
           // idk
    if (speed > 255) {
      speed = 255;
    }
    speed = 255 - speed; // (speed is negative here?)
    return (-speed);
  }
}
Motor1::Motor1(int pinA, int pinB) : motor1(PWM_DIR, pinA, pinB) {}
Motor2::Motor2(int pinA, int pinB) : motor2(PWM_DIR, pinA, pinB) {}

void Motor1::stopMotor() { motor1.setSpeed(0); }

void Motor1::setSpeed(signed int speed) { motor1.setSpeed(speed); }

void Motor1::setSpeedDir(int speed, signed int direction) {
  motor1.setSpeed(getSpeedForDir(speed, direction));
}

void Motor2::stopMotor() { motor2.setSpeed(0); }

void Motor2::setSpeed(signed int speed) { motor2.setSpeed(speed); }

void Motor2::setSpeedDir(int speed, signed int direction) {
  motor2.setSpeed(getSpeedForDir(speed, direction));
}
