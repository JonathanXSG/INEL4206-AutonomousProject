#include "dht.h"

#define DHT_DEBUG false

// Setup debug printing macros.
#ifdef DHT_DEBUG
  #define DEBUG_PRINT(...) { Serial.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { Serial.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif

// Pin definitions
#define DHT_PIN P2_3

void setup(){
  Serial.begin(9600);
  Serial.println("DHT11 test start"); 
}

void loop(){
  uint8_t rawtemperature, rawhumidity, checksum;
  float speedSound;
  dht DHT11(DHT_PIN);

  do{
    checksum = DHT11.readRawData(&rawtemperature, &rawhumidity);
    if(checksum != 0 && DHT_DEBUG) DEBUG_PRINTLN("Error reading!");
  }
  while(checksum != 0);

  speedSound = 331.4 + (0.606 * rawtemperature) + (0.0124 * rawhumidity);
  if(DHT_DEBUG){
    DEBUG_PRINT("T: ");
    DEBUG_PRINT(rawtemperature);
    DEBUG_PRINT(" H: ");
    DEBUG_PRINT(rawhumidity);
    DEBUG_PRINT(" Speed: ");
    DEBUG_PRINT(speedSound);
    DEBUG_PRINT("\n");
  }
  
  delay(1000);
}
