/*
  Library for DHT11 and DHT22/RHT03/AM2302
  https://github.com/MORA99/Stokerbot/tree/master/Libraries/dht
  Released into the public domain - http://unlicense.org
*/

#include "dht.h"

dht::dht(uint8_t pin) {
  _pin = pin;
}

int8_t dht::readRawData(uint8_t *temperature, uint8_t *humidity) {
	uint8_t bits[5];
	uint8_t i,j = 0;

	memset(bits, 0, sizeof(bits));

// Sending request to DHT11: low for 18ms
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
  delay(18);

	pinMode(_pin, INPUT);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(40);

	//check response condition 1
	if(digitalRead(_pin)==HIGH) {
		return -1;
	}
	delayMicroseconds(80);
	//check response condition 2
	if(digitalRead(_pin)==LOW) {
		return -2;
	}
  //Sensor init ok, now read 5 bytes ...
  for (j=0; j<5; j++){
    for (int8_t i=7; i>=0; i--){
      if (pulseIn(_pin, HIGH, 1000) > 30)
        bitSet(bits[j], i);
    }
  }
        
	//reset port
	pinMode(_pin, INPUT);        

	if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) == bits[4]) {
		//return temperature and humidity
		*humidity = bits[0];
    *temperature = bits[2];
		return 0;
	}
	return -5;
}
