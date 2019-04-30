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
const int motorForwardLeft = 7;
const int motorBackLeft = 8;
const int motorEnableRight = 11;
const int motorForwardRight = 12;
const int motorBackRight = 10;
const int trigPinFront = A1;
const int echoPinFront = 2;
const int trigPinLeft = 3;
const int echoPinLeft = 4;
const int trigPinRight = 5;
const int echoPinRight = 6;
const int irPin = A0;

//Variables for the Motors (could change)*****
const int leftMotorSpeed = 160; 
const int rightMotorSpeed = 255;
const int delayTime = 150;

//Variables for Ultrasonic Sensors (could change)*****
long durationFront;
int distanceFront;
long durationLeft;
int distanceLeft;
long durationRight;
int distanceRight;
const int minFrontDistance = 30;
const int minSideDistance = 20;
const int stuckDistance = 10;

//Variables for IR Sensor
#include <IRremote.h>
IRrecv irrecv(irPin);
decode_results results;
boolean onoff = 0;
//Control IR numbers (could change)*******
const long PLAY = 16761405;
const long PREV = 16720605;

//Variables fro I2C
int place;
byte dist[3];
long entryP = 0;

//Anibal's note: de aqui en adelante estan la programacion basica para las direcciones 

void stopCar () {
  digitalWrite(motorForwardLeft, LOW);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorForwardRight, LOW);
  digitalWrite(motorBackRight, LOW);
  analogWrite(motorEnableLeft, 0);
  analogWrite(motorEnableRight, 0);
}

void goForwardFull () {
  digitalWrite(motorForwardLeft, HIGH);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorForwardRight, HIGH);
  digitalWrite(motorBackRight, LOW);
  analogWrite(motorEnableLeft, leftMotorSpeed);
  analogWrite(motorEnableRight, rightMotorSpeed);
}

void goLeft () {
  digitalWrite(motorForwardLeft, LOW);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorForwardRight, HIGH);
  digitalWrite(motorBackRight, LOW);
  analogWrite(motorEnableLeft, 0);
  analogWrite(motorEnableRight, rightMotorSpeed);
}

void goRight () {
  digitalWrite(motorForwardLeft, HIGH);
  digitalWrite(motorBackLeft, LOW);
  digitalWrite(motorForwardRight, LOW);
  digitalWrite(motorBackRight, LOW);
  analogWrite(motorEnableLeft, leftMotorSpeed);
  analogWrite(motorEnableRight, 0);
}

void goBack () {
  digitalWrite(motorForwardLeft, LOW);
  digitalWrite(motorBackLeft, HIGH);
  digitalWrite(motorForwardRight, LOW);
  digitalWrite(motorBackRight, HIGH);
  analogWrite(motorEnableLeft, leftMotorSpeed);
  analogWrite(motorEnableRight, rightMotorSpeed);
}

float getSpeedOfSound() {
  dht DHT11(dhtPin);
  uint8_t rawtemperature, rawhumidity, checksum;
  checksum = DHT11.readRawData(&rawtemperature, &rawhumidity);
  if (checksum != 0)
    return 340.3; //Return default value in case of error with sensor.
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
//Anibal's note: la parte de los sensores lo saque del internet, para tenerlos de una vez. Lo puedes modificar para que funcione con nuestro robot si es necesario

void sensorRead () {
  //Read front sensor value
  digitalWrite(trigPinFront, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
  durationFront = pulseIn(echoPinFront, HIGH);
  distanceFront = durationFront * 0.034 / 2;
  //Read left sensor value
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  durationLeft = pulseIn(echoPinLeft, HIGH);
  distanceLeft = durationLeft * 0.034 / 2;
  //Read right sensor value
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);
  durationRight = pulseIn(echoPinRight, HIGH);
  distanceRight = durationRight * 0.034 / 2;
}


void setup() {
   Wire.begin();
  pinMode(motorEnableLeft, OUTPUT);
  pinMode(motorForwardLeft, OUTPUT);
  pinMode(motorBackLeft, OUTPUT);
  pinMode(motorEnableRight, OUTPUT);
  pinMode(motorForwardRight, OUTPUT);
  pinMode(motorBackRight, OUTPUT);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  irrecv.enableIRIn();
}

void loop() {
  sensorRead();
  if (irrecv.decode(&results)) {
    irrecv.resume();
    if (results.value == PLAY)
      onoff = 1;
    else if (results.value == PREV) // Anibal's note: Tengo entendido que una parte de aqui esta controlada por un control, por lo menos la parte de prende y abagar. Lo puedes modifacar como tu quieras
      onoff = 0;
  }
  if (onoff == 1) {
    results.value = 0;
    if ((distanceFront <= minFrontDistance) || (distanceLeft <= minSideDistance) || (distanceRight <= minSideDistance)) {
      if ((distanceLeft < stuckDistance) || (distanceRight < stuckDistance) || (distanceFront < stuckDistance)) {
        goBack();
        delay(1.5*delayTime);
      }
      else if ((distanceFront <= minFrontDistance) && (distanceLeft <= minSideDistance) && (distanceRight <= minSideDistance)) {
        goBack();
        delay(1.5*delayTime);
      }
      else if (distanceLeft > distanceRight ) {
        goLeft();
        delay(delayTime);
      }
      else if (distanceLeft <= distanceRight) {
        goRight();
        delay(delayTime);
      }
      else
        goForwardFull();
    }
    else
      goForwardFull();
  }
  else if (onoff == 0) {
    results.value = 0;
    stopCar();
  }
}
