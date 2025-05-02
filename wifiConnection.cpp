#include "header.h"
#include <WiFi.h>
#include <esp_wifi.h>

void initWifiConnection(){
  
  // accende il led per il debug
  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, HIGH);
 
  // Setup wifi
  WiFi.mode(WIFI_STA);
  //esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]); // per recuperare il MAC address del sensore bruciato
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");

  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.print("IP ");
  Serial.println(WiFi.localIP());
  setHostName(WiFi.getHostname());
  
  // prova la cnnessione per 10 volte
  int i = 0;
  while ((WiFi.status() != WL_CONNECTED) && i < 10) {
    Serial.print("connect ");
    delay(2000);
    
    //myDelay(3000);
    i++;
  }

  if (i >= 10){
    emergencyReset();
  }  
  Serial.println(" Done... ");  
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  // spegne il led di debug
   
  //Serial.println();
  digitalWrite(LED_PIN, LOW);
}



