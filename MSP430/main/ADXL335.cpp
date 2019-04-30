#include <math.h>
#include "ADXL.hpp"

ADXL::ADXL(uint8_t xPin, uint8_t yPin, uint8_t zPin) {
  _xPin = xPin;
  _yPin = yPin;
  _zPin = zPin;
}

void ADXL::readRawData(int16_t *xValue, int16_t *yValue, int16_t *zValue){
  *xValue = analogRead(_xPin); // Raw X Acceleration
  *yValue = analogRead(_yPin); // Raw Y Acceleration 
  *zValue = analogRead(_zPin); // Raw Z Acceleration
}
void ADXL::readGData(double *xValue, double *yValue, double *zValue){
  *xValue = ((((double)(analogRead(_xPin) * 3.3)/1024) - 1.65 ) / 0.330 );
  *yValue = ((((double)(analogRead(_yPin) * 3.3)/1024) - 1.65 ) / 0.330 );
  *zValue = ((((double)(analogRead(_zPin) * 3.3)/1024) - 1.80 ) / 0.330 );
}


void ADXL::getPitch(double *pitchValue){
  double xValue, zValue;
  xValue = ((((double)(analogRead(_xPin)*3.3)/1024)-1.65)/0.330);
  zValue = ((((double)(analogRead(_zPin)*3.3)/1024)-1.80)/0.330);

  *pitchValue =(((atan2(zValue,xValue)*180)/3.14)+180 ); // Formula for pitch
}

void ADXL::getRoll(double *rollValue){
  double yValue, zValue;
  yValue = ((((double)(analogRead(_yPin)*3.3)/1024)-1.65)/0.330);
  zValue = ((((double)(analogRead(_zPin)*3.3)/1024)-1.80)/0.330);

  *rollValue = (((atan2(yValue,zValue)*180)/3.14)+180); //Formula for pitch
}
//  yaw = ( ( (atan2(x_g_value,y_g_value) * 180) / 3.14 ) + 180 ); /* Formula for yaw */
