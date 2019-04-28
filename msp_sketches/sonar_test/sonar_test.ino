#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

DHT_Unified dht(11, DHT11);

uint32_t delayMS;

void setup() {
  Serial.begin(9600); 
  dht.begin();
  Serial.println("DHT11 Test");
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000; 
}

void loop() { 
  delay(delayMS);
  sensors_event_t event;  
  dht.temperature().getEvent(&event);  
  Serial.print("Temperature : ");
  Serial.print(event.temperature);
  Serial.println(" *C");
  dht.humidity().getEvent(&event); 
  Serial.print("Humidity : ");
  Serial.print(event.relative_humidity);
  Serial.println("%");        
}
