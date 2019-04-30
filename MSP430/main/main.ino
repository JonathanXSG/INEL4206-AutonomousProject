#include <Wire.h>
#include "ADXL.hpp"
#include "dht.h"

#define DHT_DEBUG false
#define ADXL_DEBUG true

// Setup debug printing macros.
#ifdef DHT_DEBUG
  #define DEBUG_PRINT(...) { Serial.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif
#ifdef ADXL_DEBUG
  #define ADXL_DEBUG_PRINT(...) {Serial.print(__VA_ARGS__);}
  #define ADXL_DEBUG_PRINTLN(...) {Serial.println(__VA_ARGS__);}
#else
  #define ADXL_DEBUG_PRINT(...) {}
  #define ADXL_DEBUG_PRINTLN(...) {}
#endif

// Pin definitions
#define DHT_PIN P2_3
#define ADXL_X_PIN P1_3
#define ADXL_Y_PIN P1_4
#define ADXL_Z_PIN P1_5

const int I2CSlaveAddress = 8;      // I2C Address.

//Pin numbers definition
const int motorEnableLeft = 9;
const int motorEnableRight = 11;
const int motorForwardLeft = 7;
const int motorBackLeft = 8;
const int motorForwardRight = 12;
const int motorBackRight = 10;
//Motor Variables
int leftMotorSpeed = 160;
int rightMotorSpeed = 255;
const float maxSpeed = 2.0;
const float minSpeed = 1.0;
const float delayTime = 1.0;

//Sensor Pins
const int dhtPin =  P2_3;
const int ap1 = A5;
const int ap2 = A4;
const int ap3 = A3;
const int I2CSlaveAddress = 8;
//Sensor Varibles
float distanceFront, distanceLeft, distanceRight;
const int minFrontDistance = 30;
const int minSideDistance = 20;
const int stuckDistance = 10;
float xa = 0.0;
float ya = 0.0;
float za = 0.0;
float yaw, pitch, roll; //TODO eliminate the ones that are not needed
float t;
float v;

//Variables fro I2C
int place;
byte dist[3];
long entryP = 0;


//Directions
void stopCar () {
  digitalWrite(motorForwardLeft, LOW);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorForwardRight, LOW);
  digitalWrite(motorBackRight, LOW);
  analogWrite(motorEnableLeft, 0);
  analogWrite(motorEnableRight, 0);
}

void goForwardFull () {
  stopCar();
  digitalWrite(motorForwardLeft, HIGH);
  digitalWrite(motorForwardRight, HIGH);
  analogWrite(motorEnableLeft, leftMotorSpeed);
  analogWrite(motorEnableRight, rightMotorSpeed);
}

void goLeft () {
  stopCar();
  digitalWrite(motorForwardRight, HIGH);
  analogWrite(motorEnableRight, rightMotorSpeed);
}

void goRight () {
  stopCar();
  digitalWrite(motorForwardLeft, HIGH);
  analogWrite(motorEnableLeft, leftMotorSpeed);
}

void goBack () {
  stopCar();
  digitalWrite(motorBackLeft, HIGH);
  digitalWrite(motorBackRight, HIGH);
  analogWrite(motorEnableLeft, leftMotorSpeed);
  analogWrite(motorEnableRight, rightMotorSpeed);
}

//Sensor Functions
float getSpeedOfSound() {
  dht DHT11(dhtPin);
  uint8_t rawtemperature, rawhumidity, checksum;
  checksum = DHT11.readRawData(&rawtemperature, &rawhumidity);
  if (checksum != 0)
    return 340.3; //Return default value in case of error with sensor.
  }
  if(DHT_DEBUG){
    DEBUG_PRINT("T: ");
    DEBUG_PRINT(rawtemperature);
    DEBUG_PRINT(" H: ");
    DEBUG_PRINT(rawhumidity);
  }
  return 331.4 + (0.606 * rawtemperature) + (0.0124 * rawhumidity);
}

void ADXLRead(){
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

void RequestATTiny(){
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
}

float readTiny(int address) {
  byte byteData ;
  long entry = millis();

  // Send request for data
  Wire.requestFrom(address, 1);
  while (Wire.available() == 0 && (millis() - entry) < 100)  Serial.print("W");
  if  (millis() - entry < 100) byteData = Wire.read();
  return ((float)byteData);
}

void updateGyro() {
  analogReference(DEFAULT);
  float svx = analogRead(ap1);
  float svy = analogRead(ap2);
  float svz = analogRead(ap3);

  xa = (((svx * 3.3) / 1024) - 1.65) / 0.330;
  ya = (((svy * 3.3) / 1024) - 1.65) / 0.330;
  za = (((svz * 3.3) / 1024) - 1.65) / 0.330;

  roll = atan2(ya, za) * 57.29577951 + 180;
  pitch = atan2(za, xa) * 57.29577951 + 180;
  yaw = atan2(xa, ya) * 57.29577951 + 180;
}

void sensorRead () {
  float s = getSpeedOfSound();
  distanceLeft = readTiny(I2CSlaveAddress) * s;
  distanceFront = readTiny(I2CSlaveAddress) * s;
  distanceRight = readTiny(I2CSlaveAddress) * s;
  updateGyro();
  v = v + xa * (t - (millis() / 1000.0)); // TODO verify that this is xa
  t = millis() / 1000.0;
}
void fixSpeed() {
  if (v > maxSpeed) {
    leftMotorSpeed -= 5;
    rightMotorSpeed -= 5;
  } else if (v < minSpeed) {
    leftMotorSpeed += 5;
    rightMotorSpeed += 5;
  }
}

//Main
void setup() {
  Wire.begin();
  pinMode(motorEnableLeft, OUTPUT);
  pinMode(motorForwardLeft, OUTPUT);
  pinMode(motorBackLeft, OUTPUT);
  pinMode(motorEnableRight, OUTPUT);
  pinMode(motorForwardRight, OUTPUT);
  pinMode(motorBackRight, OUTPUT);

  t = millis() / 1000.0;
  sensorRead();
}

void loop() {
  sensorRead();
  fixSpeed();
  if ((distanceFront <= minFrontDistance) || (distanceLeft <= minSideDistance) || (distanceRight <= minSideDistance)) {
    if ((distanceLeft < stuckDistance) || (distanceRight < stuckDistance) || (distanceFront < stuckDistance)) {
      goBack();
      delay(1.5 * delayTime);
    }
    else if ((distanceFront <= minFrontDistance) && (distanceLeft <= minSideDistance) && (distanceRight <= minSideDistance)) {
      goBack();
      delay(1.5 * delayTime);
    }
    else if (distanceLeft > distanceRight ) {
      goLeft();
      delay(delayTime);
    }
    else {
      goRight();
      delay(delayTime);
    }
  }
  else
    goForwardFull();
}
