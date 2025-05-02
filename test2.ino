
#include "header.h"        
#include <WiFi.h>
#include <ArduinoOTA.h>

// Contatore watchdog
unsigned long lastWatchdogFeed = 0;
const unsigned long WATCHDOG_FEED_INTERVAL = 500; // ms

void setup() {
  // Inizializza la comunicazione seriale
  Serial.begin(115200);
  Serial.println("\n\n=== ESP32 Robot Navigation v2.0 ===");
  Serial.println("Sistema con A* ottimizzato per evitare watchdog reset");
  
  // Inizializzazione hardware
  Serial.println("[INIT] Inizializzazione motori...");
  initMotor();
  
  Serial.println("[INIT] Inizializzazione sensori...");
  initHcsr04();
  
  // Connessione WiFi
  Serial.print("[WIFI] Connessione a ");
  Serial.print(WIFI_SSID);
  Serial.println("...");
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Attendi connessione WiFi
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WIFI] Connesso! IP: ");
    Serial.println(WiFi.localIP());
    
    // Avvia servizi di rete
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    Serial.println("[TELNET] Server avviato sulla porta 23");
    
    ArduinoOTA.begin();
    Serial.println("[OTA] Servizio avviato");
  } else {
    Serial.println("\n[WIFI] Connessione fallita. Continuo senza rete.");
  }
  
  // Inizializza posizione del robot
  robot.x = 0;
  robot.y = 0;
  traveledDistance = 0.0;
  currentHeading = 0.0f;
  
  // Inizializza griglia per A*
  initializeGrid();
  
  // Stampa configurazione
  Serial.println("\n=== CONFIGURAZIONE ===");
  Serial.printf("Dimensione griglia: %dx%d celle\n", GRID_SIZE, GRID_SIZE);
  Serial.printf("Risoluzione: %d mm/cella\n", GRID_RESOLUTION);
  Serial.printf("Area coperta: %.1f x %.1f metri\n", 
                (GRID_SIZE * GRID_RESOLUTION) / 1000.0f, 
                (GRID_SIZE * GRID_RESOLUTION) / 1000.0f);
  Serial.printf("Massima esplorazione: %d nodi\n", MAX_NODES_TO_EXPLORE);
  Serial.println("====================\n");
  
  // Istruzioni
  Serial.println("=== ISTRUZIONI ===");
  Serial.println("1. Connettiti al server telnet all'indirizzo IP mostrato sopra, porta 23");
  Serial.println("2. Invia 'DEST x y' per impostare una destinazione (es: DEST 1000 500)");
  Serial.println("3. Il robot si muoverà verso la destinazione usando A* ottimizzato");
  Serial.println("4. Usa 'STATUS' per verificare la posizione attuale");
  Serial.println("=================\n");
  
  Serial.println("[SISTEMA] Inizializzazione completata. In attesa di comandi...");
  
  // Inizializza timer per watchdog
  lastWatchdogFeed = millis();
}

// Funzione per "nutrire" il watchdog e prevenire reset
void feedWatchdog() {
  // Rilascia il controllo brevemente
  yield();
  
  // Aggiorna timestamp
  lastWatchdogFeed = millis();
}

void loop() {
  // Timestamp per calcoli di tempo
  unsigned long currentMillis = millis();
  
  // "Nutrimento" del watchdog
  if (currentMillis - lastWatchdogFeed >= WATCHDOG_FEED_INTERVAL) {
    feedWatchdog();
  }
  
  // Gestione servizi di rete
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    handleTelnet();
  }
  
  // Gestione navigazione
  if (destinationSet) {
    // Esegui un ciclo di navigazione
    navigateToDestination();
    
    // Se la navigazione è completata (flag destinationSet resettato all'interno della funzione)
    if (!destinationSet) {
      Serial.println("[SISTEMA] Navigazione completata. In attesa di nuovi comandi...");
    }
  }
  
  // Breve pausa per evitare high CPU usage
  delay(10);
}

