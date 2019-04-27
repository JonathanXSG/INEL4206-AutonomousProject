#include "src/TinyWireS/TinyWireS.h"

#define ULTRASONIC_LEFT_PIN PB1
#define ULTRASONIC_MIDDLE_PIN PB3
#define ULTRASONIC_RIGHT_PIN PB4

#define U_LEFT_INDEX 0
#define U_MIDDLE_INDEX 1
#define U_RIGHT_INDEX 2


// Max distance we will allow Ultrasound sensor to sens, so we can send
// through the I2C bus
#define MAX_DISTANCE 260

// I2C Address of ATTiny85
const int I2CAddress = 8;

// DFatastrcutures for storing distances
int distance[3];
int responseCount = 0;
unsigned long start;

void setup(){
  // Begin I2C Communication
  TinyWireS.begin(I2CAddress);
  // When IC2 request arrives
  TinyWireS.onRequest(transmit);
}

void loop(){
  readDistances();
}

void readDistances(){
  // TODO: Implement Ultrasound sensors
}

void transmit(){
  byte byteData;
  // Select data to send to Master, first one is the marker
  switch (responseCount) {
    case 0:
      byteData = 255;
      break;
    case 1:
      byteData = distance[U_LEFT_INDEX];
      break;
    case 2:
      byteData = distance[U_MIDDLE_INDEX];
      break;
    case 3:
      byteData = distance[U_RIGHT_INDEX];
      break;
  }
  //Send response to Master
  TinyWireS.send(byteData);
  // Increment the response count or reset
  responseCount = responseCount + 1;
  if (responseCount > 3) responseCount = 0;
}
