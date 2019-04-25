#include "ADXL.hpp"

#define ADXL_DEBUG true

#ifdef ADXL_DEBUG
  #define ADXL_DEBUG_PRINT(...) {Serial.print(__VA_ARGS__);}
  #define ADXL_DEBUG_PRINTLN(...) {Serial.println(__VA_ARGS__);}
#else
  #define ADXL_DEBUG_PRINT(...) {}
  #define ADXL_DEBUG_PRINTLN(...) {}
#endif

// Pin definitions
#define ADXL_X_PIN P1_3
#define ADXL_Y_PIN P1_4
#define ADXL_Z_PIN P1_5

void setup(){
  Serial.begin(9600);
  Serial.println("ADXL test"); 
}

void loop(){
  int16_t xValueInit, yValueInit, zValueInit;
  int16_t xValue, yValue, zValue;
  boolean init = false;
  double xGValue, yGValue, zGValue, pitch, roll;

  ADXL sensor(ADXL_X_PIN, ADXL_Y_PIN, ADXL_Z_PIN);
  if(!init){
    sensor.readRawData(&xValueInit, &yValueInit, &zValueInit);
    init = true;
  }
  sensor.readRawData(&xValue, &yValue, &zValue);

  ADXL_DEBUG_PRINT("Raw X = ");
  ADXL_DEBUG_PRINT(xValue-xValueInit);
  ADXL_DEBUG_PRINT("\tY = ");
  ADXL_DEBUG_PRINT(yValue-yValueInit);
  ADXL_DEBUG_PRINT("\tZ = ");
  ADXL_DEBUG_PRINT(zValue-zValueInit);
  ADXL_DEBUG_PRINTLN(""); 
}
