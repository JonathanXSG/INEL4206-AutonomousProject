#ifndef ADXL335_H_
#define ADXL335_H_
#include "Energia.h"

class ADXL{
  private:
    uint8_t _xPin, _yPin, _zPin;
  public:
    ADXL(uint8_t xPin, uint8_t yPin, uint8_t zPin);
    void readRawData(int16_t *xValue, int16_t *yValue, int16_t *zValue);
    void readGData(double *xValue, double *yValue, double *zValue);
    void getPitch(double *pitchValue);
    void getRoll(double *rollValue);

};

#endif
