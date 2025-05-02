#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "header.h"
// http://www.noveldevices.co.uk/esdl/es-measure-supply-voltage.ino

String hostName = "";
float volts = 0.0;

float getVolts() {
  return volts;
}

void setHostName(String s) {
  hostName = s;
}

String getHostName() {
  return hostName;
}

void emergencyReset(){
#ifdef ESP32
 esp_sleep_enable_timer_wakeup(10000000); //10s
 esp_deep_sleep_start();  
#else 
 delay(1000);
 ESP.restart(); 
#endif
}
