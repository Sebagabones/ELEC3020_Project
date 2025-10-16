#include "HC-SR04.hpp"

UltraSonic::UltraSonic(int triggerPin, int echoPin, int maxDistance)
    : sonar(triggerPin, echoPin, maxDistance) {}

unsigned int UltraSonic::sendPulse() {
  return (sonar.convert_cm(sonar.ping_median(5)));
}
