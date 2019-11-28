#ifndef TRACKBAR_h
#define TRACKBAR_h

#include <string.h>
#include <inttypes.h>

class Trackbar {
private:
public:
  char key[10];
  double & value;

  static Trackbar * instances[20];
  static uint8_t instancesCount;

  Trackbar(const char * key, double & value)
    : value(value){
    strcpy(this->key, key);
    instances[instancesCount] = this;
    instancesCount++;
  }
};
#endif

