/*
  Library for DHT11 and DHT22/RHT03/AM2302
  https://github.com/MORA99/Stokerbot/tree/master/Libraries/dht
  Released into the public domain - http://unlicense.org
*/

#ifndef DHT_H_
#define DHT_H_
#include "Energia.h"

class dht{
  private:
    uint8_t _pin;
  public:
    dht(uint8_t pin);
    int8_t readRawData(uint8_t *temperature, uint8_t *humidity);
};

#endif
