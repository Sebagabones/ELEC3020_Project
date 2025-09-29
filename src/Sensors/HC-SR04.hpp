#ifndef SONARWRAPPER_H
#define SONARWRAPPER_H
#include <NewPing.h>

class UltraSonic {
public:
  UltraSonic(int triggerPin, int echoPin, int maxDistance);
  unsigned int sendPulse();

private:
  NewPing sonar;
};
#endif
