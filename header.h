#ifndef HEADER_H
#define HEADER_H

  #include <Arduino.h>
  #include <WiFi.h>
  #include "node.h"

  #define NAME_FILE "** 0012.02particolato_ESPxx_0.0 **"

  #define WIFI_SSID "Pixel_7943" 
  #define WIFI_PASSWORD "1234567890"
  
  #define MAX_TELNET_CLIENTS 4

  #define PI 3.14159265359
  #define WHEEL_DIAMETER  61.3  // mm
  #define ROBOT_TRACK 140.0    // Distanza tra le ruote in mm (da regolare in base alla tua configurazione)
  
  // Costanti (assicurati che siano definite anche in header.h se necessarie)
  // Costante per la distanza di sicurezza in cm (10 cm in questo caso)
  #define SAFE_DISTANCE 10
  
  // Numero di passi da avanzare dopo una manovra di evitamento
  #define AVANZO_STEPS_AVVIO 2
  #define SOGLIA_ARRIVO 10  
  #define AVANZO_MM 10

  // Definizione della struttura per il robot
  struct RobotState {
    float x;  // coordinata x
    float y;  // coordinata y
  };

  // Dichiarazione della variabile globale robot, così da poterla utilizzare in tutti i file del progetto.
  extern RobotState robot;
  extern bool destinationSet;

  extern int goalX, goalY;
  extern float currentHeading;
  extern float traveledDistance;

  const int MAX_STUCK_ITER = 10;      // Iterazioni senza progresso per considerare il robot "bloccato"

  #define GIRO 2048       // steps necessary for a spin of the wheel
  #define MM1 106         // 2048 / (66.5*3.14) = 1 mm 
  //#define MM10 1060     // 10 mm 
  #define G15 191         // turns 15 degrees
  #define G30 383         // turns 30 degrees
  #define G45 574         // turns 45 degrees
  #define G90 1148        // turns 90 degrees    //cambia funzione per entrambe ruote

  #define LED_PIN 4

  // MOTOR STEPPER
  #define MOTORSTEP_SX_1 26
  #define MOTORSTEP_SX_2 25
  #define MOTORSTEP_SX_3 33
  #define MOTORSTEP_SX_4 32

  #define MOTORSTEP_DX_1 13
  #define MOTORSTEP_DX_2 12
  #define MOTORSTEP_DX_3 14
  #define MOTORSTEP_DX_4 27

  #define SPED_MOTOR      3 // 3

  // HC-SR04 
  #define DIST_TRIG 15
  #define DIST_SX_ECHO 19
  #define DIST_CE_ECHO 18
  #define DIST_DX_ECHO 5
  
  #define WIFI_SSID "Pixel_7943" //"Laboratorio-STI" 
  #define WIFI_PASSWORD "1234567890"
  
  
  #define INFLUXDB_URL "http://informatica-iot.freeddns.org:8086" //"http://iot.digit.srl:38983"
  #define INFLUXDB_TOKEN "RtJQ92waC__dxd9tQGXPyNkNuj2r0PpRmxis7J4riJ76K0WAuv0Ez1xgTKZnvqYczjaq66tMQ1eGZwNhkzeE1Q=="
  #define INFLUXDB_ORG "uniurb"
  #define INFLUXDB_BUCKET "test" 

  #define HOST "ESP32-53" 
  
  #define LOCATION "urbino"
  #define ROOM "Collegio_Raffaello" 
  

  // myUtils
  void emergencyReset();
  void setHostName(String s);
  String getHostName();

  // wifiConnection
  void initWifiConnection();
  void myPowerOff();
  void printWiFiDiagnostic();  // Nuova funzione di diagnostica
  void testTelnetSystem();     // Nuova funzione per testare il sistema telnet

  // sensore ultrasuoni
  void initHcsr04();
  void readHcsr04();
  float getDistanceSx();
  float getDistanceCe();
  float getDistanceDx();

  // motore
  void initMotor();
  void goMotor(char m, int n);
  void rotate(int angle, char direction);
  float stepsToMM(int steps);
  void lowPin();

  // my_engine
  float distanzaAlGoal();
  void navigateToDestination();
  void testOdometria();
  void updateGridMap();
  void printGridMap();
  void startNavigation();
  bool executeNextPathStep();
  /*
  enum NavState : uint8_t {
    NAV_IDLE,
    NAV_INIT,
    NAV_PATHFINDING,
    NAV_EXECUTING_PATH,
    NAV_RECALCULATING,
    NAV_DONE,
  };

  // Variabile globale di stato
  extern NavState navState;
  */
  bool processNavigation();


  // a_star
  void initializeGrid();
  bool findPath(int startX, int startY, int goalX, int goalY);
  void reconstructPath();
  bool isPathValid();

  // telnet
  void broadcastMessage(const char* msg);
  void broadcastMessagef(const char* format, ...);
  void handleTelnet();
  extern WiFiServer telnetServer;
  extern WiFiClient telnetClients[MAX_TELNET_CLIENTS];

  //ota
  void otaTask(void * parameter);

#endif