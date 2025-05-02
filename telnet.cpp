#include "header.h"
#include <WiFi.h>
#include <ESPTelnet.h>

WiFiServer telnetServer(23);
WiFiClient telnetClients[MAX_TELNET_CLIENTS];

// Flag per debug
const bool TELNET_DEBUG = true;

// Funzione per inviare messaggi a tutti i client Telnet
void broadcastMessage(const char* msg) {
  if (TELNET_DEBUG) {
    Serial.printf("[TELNET] Broadcast: %s", msg);
  }
  
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (telnetClients[i] && telnetClients[i].connected()) {
      telnetClients[i].print(msg);
      // Assicura che i dati vengano inviati subito
      telnetClients[i].flush();
    }
  }
}

void broadcastMessagef(const char* format, ...) {
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  broadcastMessage(buffer);
}

// Funzione per gestire la connessione Telnet e ricevere comandi
void handleTelnet() {
  // Controlla nuove connessioni
  if (telnetServer.hasClient()) {
    bool slotTrovato = false;
    
    // Cerca uno slot libero
    for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
      if (!telnetClients[i] || !telnetClients[i].connected()) {
        // Chiudi eventuali client esistenti nello slot
        if(telnetClients[i]) {
          telnetClients[i].stop();
        }
        
        // Accetta il nuovo client
        telnetClients[i] = telnetServer.available();
        
        // Invio messaggio di benvenuto
        telnetClients[i].println("TELNET CONNECTED. Send command: DEST x y");
        telnetClients[i].flush(); // Assicura invio immediato
        
        if (TELNET_DEBUG) {
          Serial.printf("[TELNET] Nuovo client connesso (slot %d). IP: %s\r\n", 
                      i, telnetClients[i].remoteIP().toString().c_str());
        }
        
        slotTrovato = true;
        break;
      }
    }
    
    // Se non ci sono slot disponibili, rifiuta la connessione
    if (!slotTrovato) {
      WiFiClient refusedClient = telnetServer.available();
      Serial.println("[TELNET] Rifiutato nuovo client: nessuno slot disponibile");
      refusedClient.stop();
    }
  }

  // Controlla dati in arrivo dai client connessi
  for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
    if (telnetClients[i] && telnetClients[i].connected()) {
      if (telnetClients[i].available()) {
        // Leggi la linea di comando (supporta sia \n che \r\n)
        String command = "";
        while (telnetClients[i].available()) {
          char c = telnetClients[i].read();
          if (c == '\n' || c == '\r') {
            // Se è \r\n, consuma anche il \n
            if (c == '\r' && telnetClients[i].peek() == '\n') {
              telnetClients[i].read(); // Consuma il \n
            }
            if (command.length() > 0) {
              break; // Abbiamo letto un comando completo
            }
            // Altrimenti è una linea vuota, continua a leggere
          } else {
            command += c;
          }
        }
        
        command.trim(); // Rimuove spazi iniziali e finali
        
        if (command.length() > 0) {
          Serial.printf("[TELNET] Comando ricevuto: '%s'\r\n", command.c_str());
          
          // Elabora il comando
          if (command.startsWith("DEST")) {
            int firstSpace = command.indexOf(' ');
            int secondSpace = command.indexOf(' ', firstSpace + 1);
            
            if (firstSpace != -1 && secondSpace != -1) {
              String xStr = command.substring(firstSpace + 1, secondSpace);
              String yStr = command.substring(secondSpace + 1);
              
              // Controlla che i valori siano numeri
              bool validX = true;
              bool validY = true;
              
              for (int j = 0; j < xStr.length(); j++) {
                if (!isDigit(xStr[j]) && !(j == 0 && xStr[j] == '-')) {
                  validX = false;
                  break;
                }
              }
              
              for (int j = 0; j < yStr.length(); j++) {
                if (!isDigit(yStr[j]) && !(j == 0 && yStr[j] == '-')) {
                  validY = false;
                  break;
                }
              }
              
              if (validX && validY) {
                goalX = xStr.toInt();
                goalY = yStr.toInt();
                destinationSet = true;
                
                String response = "DESTINATION SET -> (" + String(goalX) + ", " + String(goalY) + ")\r\n";
                telnetClients[i].print(response);
                telnetClients[i].flush();
                
                Serial.printf("[TELNET] Destination set to (%d, %d)\r\n", goalX, goalY);
                
                // Broadcast agli altri client
                for (int j = 0; j < MAX_TELNET_CLIENTS; j++) {
                  if (j != i && telnetClients[j] && telnetClients[j].connected()) {
                    telnetClients[j].print(response);
                    telnetClients[j].flush();
                  }
                }
              } else {
                telnetClients[i].println("INVALID COMMAND: Coordinate devono essere numeri. Usa: DEST x y\r\n");
                telnetClients[i].flush();
              }
            } else {
              telnetClients[i].println("INVALID COMMAND: Formato errato. Usa: DEST x y\r\n");
              telnetClients[i].flush();
            }
          } else if (command.startsWith("HELP")) {
            telnetClients[i].println("Comandi disponibili:");
            telnetClients[i].println("  DEST x y   - Imposta la destinazione (x,y) per il robot");
            telnetClients[i].println("  HELP       - Mostra questa guida");
            telnetClients[i].println("  STATUS     - Mostra la posizione attuale del robot\r\n");
            telnetClients[i].flush();
          } else if (command.startsWith("STATUS")) {
            String status = "Posizione robot: (" + String(robot.x) + ", " + String(robot.y) + ")\r\n";
            status += "Heading: " + String(currentHeading * 180 / PI) + " gradi\r\n";
            status += "Distanza percorsa: " + String(traveledDistance) + " mm\r\n";
            
            if (destinationSet) {
              status += "Destinazione: (" + String(goalX) + ", " + String(goalY) + ")\r\n";
              status += "Distanza alla meta: " + String(distanzaAlGoal()) + " mm\r\n";
            } else {
              status += "Nessuna destinazione impostata\r\n";
            }
            
            telnetClients[i].print(status);
            telnetClients[i].flush();
          } else {
            telnetClients[i].println("COMANDO NON RICONOSCIUTO. Usa HELP per la lista comandi.\r\n");
            telnetClients[i].flush();
          }
        }
      }
    }
  }
}