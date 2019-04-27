#include <Wire.h>

int place;
byte dist[3];
long entryP = 0;

const int I2CSlaveAddress = 8;      // I2C Address.

void setup(){
  Wire.begin();
  Serial.begin(96000);
  Serial.println("Starting IC2");
}

void loop(){
    while (readTiny(I2CSlaveAddress) < 255) {
    Serial.println("Waiting for Data..."); // wait for first byte
  }
  
  dist[0] = readTiny(I2CSlaveAddress);
  dist[1] = readTiny(I2CSlaveAddress);
  dist[2] = readTiny(I2CSlaveAddress);

  Serial.print("Left ");
  Serial.print(dist[0]);
  Serial.print(" ");
  Serial.print("Middle ");
  Serial.print(dist[1]);
  Serial.print(" ");
  Serial.print("Right ");
  Serial.print(dist[2]);
  Serial.print(" ");
  Serial.println();
  delay(200);
}

byte readTiny(int address) {
  byte byteData ;
  long entry = millis();

  // Send request for data
  Wire.requestFrom(address, 1);
  while (Wire.available() == 0 && (millis() - entry) < 100)  Serial.print("W");
  if  (millis() - entry < 100) byteData = Wire.read();
  return byteData;
}
