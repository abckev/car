#include "header.h" 
#include "a_star.h"
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>

// Variabili globali per la navigazione
RobotState robot;              // Posizione corrente del robot
int goalX = 0;                 // Impostata tramite comando Telnet
int goalY = 0;
bool destinationSet = false;

float traveledDistance = 0.0;  // Distanza percorsa (in mm)
float currentHeading = 0;      // In radianti

// Indice del passo corrente nel percorso
int currentPathIndex = 0;
// Contatore per il tentativo di ricalcolo del percorso
int recalculationAttempts = 0;
const int MAX_RECALCULATION_ATTEMPTS = 3;

// Contatore di aggiornamenti della mappa
uint16_t mapUpdateCounter = 0;
const uint16_t MAP_UPDATE_INTERVAL = 5; // Numero di passi dopo cui aggiornare la mappa

// Per navigazione gerarchica
bool usingHierarchicalNavigation = false;
int intermediateGoalX = 0;
int intermediateGoalY = 0;

// Stato della navigazione cooperativa
enum NavigationState {
  NAV_IDLE,                // Nessuna navigazione in corso
  NAV_INIT,                // Inizializzazione
  NAV_PATHFINDING,         // Calcolo percorso in corso
  NAV_EXECUTING_PATH,      // Esecuzione percorso
  NAV_RECALCULATING,       // Ricalcolo percorso in corso
  NAV_DONE                 // Navigazione completata
};

NavigationState navState = NAV_IDLE;

// Calcola la distanza euclidea dalla posizione corrente al goal
float distanzaAlGoal() {
  return sqrt(pow(goalX - robot.x, 2) + pow(goalY - robot.y, 2));
}

/**
 * Aggiorna la mappa della griglia in base alle letture dei sensori
 */
void updateGridMap() {
  // Ottieni le letture degli ultrasuoni
  readHcsr04();
  float distanzaSinistra = getDistanceSx();
  float distanzaCentro = getDistanceCe();
  float distanzaDestra = getDistanceDx();
  
  // Calcola le coordinate in griglia del robot
  int robotGridX = GRID_CENTER + round(robot.x / GRID_RESOLUTION);
  int robotGridY = GRID_CENTER + round(robot.y / GRID_RESOLUTION);
  
  // Verifica se il robot è all'interno della griglia
  if (robotGridX < 0 || robotGridX >= GRID_SIZE || robotGridY < 0 || robotGridY >= GRID_SIZE) {
    Serial.println("[MAP] ATTENZIONE: Robot fuori dalla griglia! Ricentraggio mappa...");
    
    // Ricentra la mappa attorno al robot spostando i dati
    int offsetX = GRID_CENTER - robotGridX;
    int offsetY = GRID_CENTER - robotGridY;
    
    // Crea una copia temporanea della griglia
    Node tempGrid[GRID_SIZE][GRID_SIZE];
    memset(tempGrid, 0, sizeof(tempGrid));
    
    // Sposta solo i dati che rimangono all'interno della griglia
    for (int y = 0; y < GRID_SIZE; y++) {
      for (int x = 0; x < GRID_SIZE; x++) {
        int newX = x + offsetX;
        int newY = y + offsetY;
        
        if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE) {
          // Conserva solo lo stato dell'ostacolo
          if (NODE_IS_OBSTACLE(grid[y][x])) {
            NODE_SET_OBSTACLE(tempGrid[newY][newX]);
          }
        }
      }
    }
    
    // Copia la griglia temporanea nella griglia principale
    for (int y = 0; y < GRID_SIZE; y++) {
      for (int x = 0; x < GRID_SIZE; x++) {
        grid[y][x].x = x;
        grid[y][x].y = y;
        grid[y][x].flags = tempGrid[y][x].flags;
        grid[y][x].parent = NULL;
      }
    }
    
    // Aggiorna le coordinate del robot dopo il ricentraggio
    robotGridX = GRID_CENTER;
    robotGridY = GRID_CENTER;
    
    // Cedi il controllo per evitare watchdog reset
    yield();
  }
  
  // Converti heading in radianti per il calcolo delle direzioni
  float headingRad = currentHeading;
  
  // Definisci le direzioni dei sensori rispetto all'heading del robot
  float angleSinistra = headingRad - PI/4;  // -45 gradi rispetto alla direzione
  float angleCentro = headingRad;           // Direzione di movimento
  float angleDestra = headingRad + PI/4;    // +45 gradi rispetto alla direzione
  
  // Aggiungi ostacoli in base alle letture dei sensori (sinistra)
  if (distanzaSinistra > 0 && distanzaSinistra < SAFE_DISTANCE * 3) {
    int obstacleDistance = round(distanzaSinistra / GRID_RESOLUTION);
    int obstacleX = robotGridX + round(cos(angleSinistra) * obstacleDistance);
    int obstacleY = robotGridY + round(sin(angleSinistra) * obstacleDistance);
    
    // Aggiungi un margine di sicurezza attorno all'ostacolo
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        int ox = obstacleX + dx;
        int oy = obstacleY + dy;
        
        if (ox >= 0 && ox < GRID_SIZE && oy >= 0 && oy < GRID_SIZE) {
          NODE_SET_OBSTACLE(grid[oy][ox]);
        }
      }
    }
    
    Serial.printf("[MAP] Ostacolo a sinistra in (%d, %d)\r\n", obstacleX, obstacleY);
    
    // Periodicamente rilascia il controllo
    yield();
  }
  
  // Aggiungi ostacoli in base alle letture dei sensori (centro)
  if (distanzaCentro > 0 && distanzaCentro < SAFE_DISTANCE * 3) {
    int obstacleDistance = round(distanzaCentro / GRID_RESOLUTION);
    int obstacleX = robotGridX + round(cos(angleCentro) * obstacleDistance);
    int obstacleY = robotGridY + round(sin(angleCentro) * obstacleDistance);
    
    // Aggiungi un margine di sicurezza attorno all'ostacolo
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        int ox = obstacleX + dx;
        int oy = obstacleY + dy;
        
        if (ox >= 0 && ox < GRID_SIZE && oy >= 0 && oy < GRID_SIZE) {
          NODE_SET_OBSTACLE(grid[oy][ox]);
        }
      }
    }
    
    Serial.printf("[MAP] Ostacolo al centro in (%d, %d)\r\n", obstacleX, obstacleY);
    
    // Periodicamente rilascia il controllo
    yield();
  }
  
  // Aggiungi ostacoli in base alle letture dei sensori (destra)
  if (distanzaDestra > 0 && distanzaDestra < SAFE_DISTANCE * 3) {
    int obstacleDistance = round(distanzaDestra / GRID_RESOLUTION);
    int obstacleX = robotGridX + round(cos(angleDestra) * obstacleDistance);
    int obstacleY = robotGridY + round(sin(angleDestra) * obstacleDistance);
    
    // Aggiungi un margine di sicurezza attorno all'ostacolo
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        int ox = obstacleX + dx;
        int oy = obstacleY + dy;
        
        if (ox >= 0 && ox < GRID_SIZE && oy >= 0 && oy < GRID_SIZE) {
          NODE_SET_OBSTACLE(grid[oy][ox]);
        }
      }
    }
    
    Serial.printf("[MAP] Ostacolo a destra in (%d, %d)\r\n", obstacleX, obstacleY);
  }
  
  // Marca la posizione del robot come non ostacolo (per sicurezza)
  if (robotGridX >= 0 && robotGridX < GRID_SIZE && robotGridY >= 0 && robotGridY < GRID_SIZE) {
    NODE_CLEAR_OBSTACLE(grid[robotGridY][robotGridX]);
  }
}

/**
 * Stampa la mappa della griglia corrente
 */
void printGridMap() {
  int robotGridX = GRID_CENTER + round(robot.x / GRID_RESOLUTION);
  int robotGridY = GRID_CENTER + round(robot.y / GRID_RESOLUTION);
  int goalGridX = GRID_CENTER + round(goalX / GRID_RESOLUTION);
  int goalGridY = GRID_CENTER + round(goalY / GRID_RESOLUTION);
  
  Serial.println("[MAP] Mappa corrente:");
  
  // Visualizza solo l'area intorno al robot per risparmiare output
  int viewRadius = 5;
  int startX = max(0, robotGridX - viewRadius);
  int endX = min(GRID_SIZE - 1, robotGridX + viewRadius);
  int startY = max(0, robotGridY - viewRadius);
  int endY = min(GRID_SIZE - 1, robotGridY + viewRadius);
  
  for (int y = startY; y <= endY; y++) {
    String line = "  ";
    for (int x = startX; x <= endX; x++) {
      if (x == robotGridX && y == robotGridY) {
        line += "R "; // Robot
      } else if (x == goalGridX && y == goalGridY) {
        line += "G "; // Goal
      } else if (NODE_IS_PATH(grid[y][x])) {
        line += "* "; // Percorso
      } else if (NODE_IS_OBSTACLE(grid[y][x])) {
        line += "# "; // Ostacolo
      } else {
        line += ". "; // Spazio libero
      }
    }
    Serial.println(line);
    
    // Rilascia il controllo ogni poche righe per evitare watchdog reset
    if ((y - startY) % 5 == 0) {
      yield();
    }
  }
}

/**
 * Calcola l'angolo necessario per ruotare verso una direzione specifica
 */
float calculateRotationAngle(float targetHeading) {
  // Normalizza l'heading corrente e target tra -PI e PI
  float currentNormalized = fmod(currentHeading + 2*PI, 2*PI);
  float targetNormalized = fmod(targetHeading + 2*PI, 2*PI);
  
  // Calcola la differenza di angolo
  float angleDiff = targetNormalized - currentNormalized;
  
  // Normalizza la differenza tra -PI e PI
  if (angleDiff > PI) angleDiff -= 2*PI;
  if (angleDiff < -PI) angleDiff += 2*PI;
  
  return angleDiff;
}

/**
 * Esegue il prossimo passo del percorso calcolato
 * Restituisce true se il percorso è completato o non valido
 */
bool executeNextPathStep() {
  // Controlla se abbiamo finito il percorso
  if (currentPathIndex >= pathLength) {
    Serial.println("[NAV] Percorso completato!");
    return true;
  }
  
  // Ottieni il nodo corrente e il prossimo nodo nel percorso
  Node* currentNode = path[currentPathIndex];
  
  // Avanza all'indice successivo per il prossimo step
  currentPathIndex++;
  
  if (currentPathIndex >= pathLength) {
    Serial.println("[NAV] Ultimo nodo del percorso raggiunto!");
    return true;
  }
  
  Node* nextNode = path[currentPathIndex];
  
  // Calcola la differenza di coordinate in griglia
  int dx = nextNode->x - currentNode->x;
  int dy = nextNode->y - currentNode->y;
  
  // Converti la direzione in un angolo in radianti
  float targetHeading = atan2(dy, dx);
  
  // Calcola l'angolo di rotazione necessario
  float rotationAngle = calculateRotationAngle(targetHeading);
  
  // Converti l'angolo in gradi
  int rotationDegrees = round(rotationAngle * 180.0 / PI);
  
  Serial.printf("[NAV] Rotazione: %d gradi\r\n", rotationDegrees);
  
  // Rilascia il controllo
  yield();
  
  // Esegui la rotazione
  if (abs(rotationDegrees) > 5) {  // Soglia minima di rotazione
    if (rotationDegrees > 0) {
      rotate(abs(rotationDegrees), 'L');
    } else {
      rotate(abs(rotationDegrees), 'R');
    }
    
    // Aggiorna l'heading corrente
    currentHeading = targetHeading;
  }
  
  // Rilascia il controllo
  yield();
  
  // Calcola la distanza da percorrere in mm
  float distance = GRID_RESOLUTION * sqrt(dx*dx + dy*dy);
  int steps = round(distance * MM1);
  
  Serial.printf("[NAV] Avanzamento: %.2f mm (%d passi)\r\n", distance, steps);
  
  // Esegui il movimento
  goMotor('a', steps);
  
  // Aggiorna la posizione del robot e la distanza percorsa
  traveledDistance += distance;
  robot.x += distance * cos(targetHeading);
  robot.y += distance * sin(targetHeading);
  
  Serial.printf("[NAV] Nuova posizione: (%.2f, %.2f)\r\n", robot.x, robot.y);
  
  // Aggiorna occasionalmente la mappa
  mapUpdateCounter++;
  if (mapUpdateCounter >= MAP_UPDATE_INTERVAL) {
    updateGridMap();
    mapUpdateCounter = 0;
  }
  
  return false; // Percorso non ancora completato
}

/**
 * Avvia la navigazione verso la destinazione
 * Questa funzione imposta solo lo stato iniziale
 */
void startNavigation() {
  // Reset dei contatori
  currentPathIndex = 0;
  recalculationAttempts = 0;
  mapUpdateCounter = 0;
  
  // Inizializza la griglia
  initializeGrid();
  
  // Imposta modalità diretta
  setNavigationMode(NAVIGATION_MODE_DIRECT);
  usingHierarchicalNavigation = false;
  
  // Stampa informazioni
  Serial.printf("[NAV] Inizio navigazione verso destinazione: (%d, %d)\r\n", goalX, goalY);
  Serial.printf("[NAV] Posizione attuale: (%.2f, %.2f)\r\n", robot.x, robot.y);
  
  // Imposta lo stato di navigazione
  navState = NAV_INIT;
}

/**
 * Funzione principale di navigazione verso la destinazione usando A*
 * Questa implementa un'architettura cooperativa per evitare watchdog reset
 * Ritorna true quando la navigazione è completata
 */
bool processNavigation() {
  // Rilascia periodicamente per evitare watchdog reset
  yield();
  
  // Gestisci la navigazione in base allo stato
  switch (navState) {
    case NAV_IDLE:
      // Nessuna navigazione in corso
      return true;
      
    case NAV_INIT:
      {
        // Controlla se siamo già vicini alla meta
        float distanzaGoal = distanzaAlGoal();
        Serial.printf("[NAV] Distanza iniziale al goal: %.2f mm\r\n", distanzaGoal);
        
        if (distanzaGoal <= SOGLIA_ARRIVO) {
          Serial.println("[NAV] Già alla destinazione!");
          broadcastMessage("Già alla destinazione!\r\n");
          navState = NAV_DONE;
          return true;
        }
        
        // Controlla se il percorso è troppo lungo per una navigazione diretta
        if (distanzaGoal > MAX_DISTANCE) {
          Serial.println("[NAV] Destinazione lontana, attivazione navigazione gerarchica");
          usingHierarchicalNavigation = true;
          setNavigationMode(NAVIGATION_MODE_EXPLORATION);
        }
        
        // Converti in coordinate di griglia
        int robotGridX = GRID_CENTER + round(robot.x / GRID_RESOLUTION);
        int robotGridY = GRID_CENTER + round(robot.y / GRID_RESOLUTION);
        int goalGridX = GRID_CENTER + round(goalX / GRID_RESOLUTION);
        int goalGridY = GRID_CENTER + round(goalY / GRID_RESOLUTION);
        
        // Controlla se il goal è all'interno della griglia
        bool goalInGrid = (goalGridX >= 0 && goalGridX < GRID_SIZE && 
                          goalGridY >= 0 && goalGridY < GRID_SIZE);
        
        // Se il goal è fuori dalla griglia o stiamo usando navigazione gerarchica
        if (!goalInGrid || usingHierarchicalNavigation) {
          Serial.println("[NAV] Goal fuori dalla griglia o troppo lontano, calcolo punto intermedio");
          
          // Calcola direzione verso il goal
          float dx = goalX - robot.x;
          float dy = goalY - robot.y;
          float distance = sqrt(dx*dx + dy*dy);
          
          // Normalizza direzione
          float norm_dx = dx / distance;
          float norm_dy = dy / distance;
          
          // Calcola punto intermedio
          float maxGridDistance = (GRID_SIZE/2 - 2) * GRID_RESOLUTION;
          float targetDistance = min(maxGridDistance, distance / 2);
          
          // Imposta meta intermedia
          intermediateGoalX = robot.x + norm_dx * targetDistance;
          intermediateGoalY = robot.y + norm_dy * targetDistance;
          
          // Converti in coordinate griglia
          goalGridX = GRID_CENTER + round(intermediateGoalX / GRID_RESOLUTION);
          goalGridY = GRID_CENTER + round(intermediateGoalY / GRID_RESOLUTION);
          
          Serial.printf("[NAV] Punto intermedio: (%.2f, %.2f) mm [griglia: %d, %d]\r\n",
                      intermediateGoalX, intermediateGoalY, goalGridX, goalGridY);
        }
        
        // Aggiorna la mappa iniziale
        updateGridMap();
        
        // Avvia il pathfinding
        startPathFinding(robotGridX, robotGridY, goalGridX, goalGridY);
        navState = NAV_PATHFINDING;
        return false;
      }
      
    case NAV_PATHFINDING:
      // Processa un passo dell'algoritmo A*
      if (processPathFindingStep()) {
        // Pathfinding completato
        if (pathFound) {
          // Percorso trovato, passa all'esecuzione
          reconstructPath();
          printGridMap();
          currentPathIndex = 0;
          navState = NAV_EXECUTING_PATH;
          return false;
        } else {
          // Nessun percorso trovato
          recalculationAttempts++;
          
          if (recalculationAttempts >= MAX_RECALCULATION_ATTEMPTS) {
            Serial.println("[NAV] Troppi tentativi falliti di trovare un percorso!");
            navState = NAV_DONE;
            return true;
          }
          
          // Prova con un punto intermedio diverso
          int robotGridX = GRID_CENTER + round(robot.x / GRID_RESOLUTION);
          int robotGridY = GRID_CENTER + round(robot.y / GRID_RESOLUTION);
          int goalGridX = GRID_CENTER + round(usingHierarchicalNavigation ? intermediateGoalX : goalX / GRID_RESOLUTION);
          int goalGridY = GRID_CENTER + round(usingHierarchicalNavigation ? intermediateGoalY : goalY / GRID_RESOLUTION);
          
          int intermediateX, intermediateY;
          if (findIntermediatePoint(robotGridX, robotGridY, goalGridX, goalGridY, intermediateX, intermediateY)) {
            Serial.printf("[NAV] Provando punto intermedio: (%d, %d)\r\n", intermediateX, intermediateY);
            startPathFinding(robotGridX, robotGridY, intermediateX, intermediateY);
            return false;
          } else {
            // Rotazione casuale per cercare una via d'uscita
            int randomDir = random(0, 2);
            rotate(90, randomDir == 0 ? 'L' : 'R');
            currentHeading += (randomDir == 0 ? PI/2 : -PI/2);
            
            // Riprova pathfinding dal punto attuale
            navState = NAV_INIT;
            return false;
          }
        }
      }
      return false;
      
    case NAV_EXECUTING_PATH:
      // Esegui un passo del percorso
      if (executeNextPathStep()) {
        // Percorso completato o non valido
        
        // Se stiamo usando navigazione gerarchica, controlla se abbiamo raggiunto il punto intermedio
        if (usingHierarchicalNavigation) {
          float distanzaIntermedia = sqrt(pow(intermediateGoalX - robot.x, 2) + pow(intermediateGoalY - robot.y, 2));
          
          if (distanzaIntermedia <= SOGLIA_ARRIVO * 2) {
            Serial.println("[NAV] ✅ Punto intermedio raggiunto! Ricalcolo...");
            navState = NAV_INIT; // Ricalcola un nuovo percorso
            return false;
          }
        }
        
        // Controlla se abbiamo raggiunto la meta finale
        float distanzaFinale = distanzaAlGoal();
        if (distanzaFinale <= SOGLIA_ARRIVO) {
          Serial.println("[NAV] ✅ DESTINAZIONE RAGGIUNTA!");
          broadcastMessage("✅ DESTINAZIONE RAGGIUNTA!\r\n");
          navState = NAV_DONE;
          return true;
        }
        
        // Altrimenti, ricalcola percorso
        navState = NAV_RECALCULATING;
        return false;
      }
      return false;
      
    case NAV_RECALCULATING:
      // Aggiorna la mappa
      updateGridMap();
      
      // Prova a ricalcolare il percorso
      {
        int robotGridX = GRID_CENTER + round(robot.x / GRID_RESOLUTION);
        int robotGridY = GRID_CENTER + round(robot.y / GRID_RESOLUTION);
        int goalGridX = GRID_CENTER + round(usingHierarchicalNavigation ? intermediateGoalX : goalX / GRID_RESOLUTION);
        int goalGridY = GRID_CENTER + round(usingHierarchicalNavigation ? intermediateGoalY : goalY / GRID_RESOLUTION);
        
        Serial.println("[NAV] Ricalcolo percorso...");
        startPathFinding(robotGridX, robotGridY, goalGridX, goalGridY);
        navState = NAV_PATHFINDING;
      }
      return false;
      
    case NAV_DONE:
      // Navigazione completata
      return true;
      
    default:
      // Stato non valido, ripristina
      navState = NAV_IDLE;
      return true;
  }
}

/**
 * Funzione principale di navigazione verso la destinazione
 * Questa è la versione da chiamare nel loop principale
 */
void navigateToDestination() {
  // Se la navigazione non è attiva, avviala
  if (navState == NAV_IDLE) {
    startNavigation();
  }
  
  // Tempo massimo per l'esecuzione in un singolo ciclo (ms)
  unsigned long maxExecutionTime = 100;
  unsigned long startTime = millis();
  
  // Processa la navigazione fino al completamento o fino al timeout
  bool completed = false;
  while (!completed && (millis() - startTime < maxExecutionTime)) {
    completed = processNavigation();
    
    // Breve rilascio per permettere ad altri processi di eseguire
    delay(0);
  }
  
  // Se completato, resetta lo stato
  if (completed) {
    navState = NAV_IDLE;
    destinationSet = false;
  }
}

// Funzione di test per l'odometria
void testOdometria() {
  int stepsSingle = MM1;
  float mmPerStep = stepsToMM(stepsSingle);
  Serial.printf("[ODOM] %d step = %.4f mm\r\n", stepsSingle, mmPerStep);

  int stepsFullRev = 2048;
  float mmFullRev = stepsToMM(stepsFullRev);
  Serial.printf("[ODOM] 2048 step (1 rev) = %.2f mm\r\n", mmFullRev);

  float stepSPEDrMM = 2048.0 / (3.14159265359 * WHEEL_DIAMETER);
  Serial.printf("[ODOM] Steps per 1 mm = %.2f\r\n", stepSPEDrMM);
}