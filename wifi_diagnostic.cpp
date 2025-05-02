#include "header.h"
#include <WiFi.h>

void printWiFiDiagnostic() {
  Serial.println("\n==== DIAGNOSTICA WI-FI ====");
  
  // Controlla stato della connessione
  Serial.print("Stato WiFi: ");
  switch (WiFi.status()) {
    case WL_CONNECTED:
      Serial.println("Connesso");
      break;
    case WL_NO_SHIELD:
      Serial.println("Modulo WiFi non rilevato");
      break;
    case WL_IDLE_STATUS:
      Serial.println("In attesa");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("Nessuna rete disponibile");
      break;
    case WL_SCAN_COMPLETED:
      Serial.println("Scansione completata");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("Connessione fallita");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("Connessione persa");
      break;
    case WL_DISCONNECTED:
      Serial.println("Disconnesso");
      break;
    default:
      Serial.println("Sconosciuto");
  }
  
  // Informazioni di rete
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    
    Serial.print("BSSID: ");
    Serial.println(WiFi.BSSIDstr());
    
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
    
    Serial.print("Subnet Mask: ");
    Serial.println(WiFi.subnetMask());
    
    Serial.print("DNS: ");
    Serial.println(WiFi.dnsIP());
    
    Serial.print("Potenza segnale (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
  }
  
  // Verifica se il server telnet è attivo
  Serial.println("\nTest server Telnet:");
  Serial.print("- Porta server: ");
  Serial.println("23 (default Telnet)");
  
  // Verifica connessioni attive
  int activeConnections = 0;
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (telnetClients[i] && telnetClients[i].connected()) {
      activeConnections++;
      Serial.print("- Client ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(telnetClients[i].remoteIP());
      Serial.print(":");
      Serial.println(telnetClients[i].remotePort());
    }
  }
  
  Serial.print("- Connessioni attive: ");
  Serial.println(activeConnections);
  
  // Consigli di debugging
  Serial.println("\nSUGGERIMENTI PER LA RISOLUZIONE PROBLEMI:");
  Serial.println("1. Verificare che l'indirizzo IP del client Python corrisponda a quello mostrato sopra");
  Serial.println("2. Assicurarsi che non ci siano firewall che bloccano la porta 23");
  Serial.println("3. Provare a fare ping all'ESP32 dal computer client");
  Serial.println("4. Verificare che ESP32 e client siano sulla stessa rete");
  
  Serial.println("============================\n");
}

// Aggiunge questa funzione per testare il sistema telnet
void testTelnetSystem() {
  Serial.println("\n==== TEST DEL SISTEMA TELNET ====");
  
  // Emula un client telnet
  Serial.println("Simulazione di un comando telnet...");
  
  // Crea un client fittizio
  WiFiClient testClient;
  
  // Tenta di connettersi a se stesso
  if (testClient.connect(WiFi.localIP(), 23)) {
    Serial.println("- Connessione riuscita al proprio server telnet");
    
    // Invia un comando di test
    testClient.println("DEST 10 20");
    delay(100);
    
    // Controlla se ci sono risposte
    Serial.print("- Risposta: ");
    if (testClient.available()) {
      String response = "";
      while (testClient.available()) {
        char c = testClient.read();
        response += c;
      }
      Serial.println(response);
    } else {
      Serial.println("Nessuna risposta ricevuta");
    }
    
    testClient.stop();
  } else {
    Serial.println("- Impossibile connettersi al proprio server telnet - questo è un problema!");
    Serial.println("  Verifica che il server telnet sia avviato correttamente");
  }
  
  Serial.println("===============================\n");
}