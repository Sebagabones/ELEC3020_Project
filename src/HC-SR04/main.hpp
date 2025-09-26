// #include "../../.piolibdeps/KillDozer2/AsyncDelay/src/AsyncDelay.h"
#include <NewPing.h>

// #include <esp32-hal-gpio.h>

// #define SOUND_SPEED 0.034

#define TRIGGER_PIN_1 12
#define ECHO_PIN_1 13
#define MAX_DISTANCE 100

NewPing sonar(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);

unsigned int sendPulse() {
  unsigned int distance = sonar.convert_cm(sonar.ping_median());

  return distance;
}
