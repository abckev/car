#include "a_star.h"
#include <Arduino.h>

// Variabili globali per l'algoritmo A*
Node grid[GRID_SIZE][GRID_SIZE];
Node* openSet[MAX_NODES_TO_EXPLORE];
int openSetCount = 0;
Node* path[GRID_SIZE * 2]; // Dimensione ridotta per risparmiare memoria
int pathLength = 0;
bool pathFound = false;
uint8_t navigationMode = NAVIGATION_MODE_DIRECT;

// Stato dell'algoritmo A* per l'implementazione cooperativa
enum AStarState {
  A_STAR_IDLE,
  A_STAR_INIT,
  A_STAR_SEARCH,
  A_STAR_RECONSTRUCT,
  A_STAR_DONE
};

// Struttura per memorizzare lo stato dell'algoritmo
struct AStarContext {
  int startX;
  int startY;
  int goalX;
  int goalY;
  AStarState state;
  Node* currentNode;
  int nodesExplored;
  unsigned long lastYieldTime;
};

AStarContext astarContext;

// Le direzioni di movimento possibili (4 cardinali + 4 diagonali)
const Direction directions[8] = {
    {0, -1, MOVE_COST_STRAIGHT},   // Nord
    {1, -1, MOVE_COST_DIAGONAL},   // Nord-Est
    {1, 0, MOVE_COST_STRAIGHT},    // Est
    {1, 1, MOVE_COST_DIAGONAL},    // Sud-Est
    {0, 1, MOVE_COST_STRAIGHT},    // Sud
    {-1, 1, MOVE_COST_DIAGONAL},   // Sud-Ovest
    {-1, 0, MOVE_COST_STRAIGHT},   // Ovest
    {-1, -1, MOVE_COST_DIAGONAL}   // Nord-Ovest
};

/**
 * Inizializza la griglia per l'algoritmo A*
 */
void initializeGrid() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            grid[y][x].x = x;
            grid[y][x].y = y;
            grid[y][x].flags = 0; // Reset di tutti i flag
            grid[y][x].parent = NULL;
        }
    }
    
    // Resetta contatori
    openSetCount = 0;
    pathLength = 0;
    pathFound = false;
    
    // Inizializza il contesto
    astarContext.state = A_STAR_IDLE;
    astarContext.nodesExplored = 0;
    astarContext.currentNode = NULL;
    astarContext.lastYieldTime = millis();
}

/**
 * Calcola il costo euristico usando la distanza di Manhattan
 */
uint16_t calculateHCost(int x1, int y1, int x2, int y2) {
    return (abs(x2 - x1) + abs(y2 - y1)) * MOVE_COST_STRAIGHT;
}

/**
 * Aggiunge un nodo all'insieme dei nodi aperti
 */
void addToOpenSet(Node* node) {
    if (openSetCount < MAX_NODES_TO_EXPLORE) {
        openSet[openSetCount] = node;
        openSetCount++;
    }
}

/**
 * Trova il nodo con il costo f più basso nell'insieme dei nodi aperti
 */
Node* getLowestFCostNode() {
    if (openSetCount == 0) return NULL;
    
    int lowestIndex = 0;
    for (int i = 1; i < openSetCount; i++) {
        if (openSet[i]->fCost < openSet[lowestIndex]->fCost) {
            lowestIndex = i;
        }
        else if (openSet[i]->fCost == openSet[lowestIndex]->fCost && 
                 openSet[i]->hCost < openSet[lowestIndex]->hCost) {
            lowestIndex = i;
        }
    }
    
    Node* result = openSet[lowestIndex];
    removeFromOpenSet(lowestIndex);
    return result;
}

/**
 * Rimuove un nodo dall'insieme dei nodi aperti
 */
void removeFromOpenSet(int index) {
    openSet[index] = openSet[openSetCount - 1];
    openSetCount--;
}

/**
 * Verifica se un nodo è nell'insieme dei nodi aperti
 */
bool isInOpenSet(Node* node) {
    for (int i = 0; i < openSetCount; i++) {
        if (openSet[i] == node) return true;
    }
    return false;
}

/**
 * Pulisce il percorso corrente
 */
void clearPath() {
    for (int i = 0; i < pathLength; i++) {
        NODE_CLEAR_PATH(*path[i]);
    }
    pathLength = 0;
}

/**
 * Semplifica il percorso rimuovendo punti collineari
 */
void simplifyPath() {
    if (pathLength < 3) return; // Impossibile semplificare percorsi molto corti
    
    int newPathLength = 0;
    Node* simplifiedPath[GRID_SIZE * 2];
    
    // Primo punto sempre incluso
    simplifiedPath[newPathLength++] = path[0];
    
    for (int i = 1; i < pathLength - 1; i++) {
        // Calcola i vettori tra punti consecutivi
        int dx1 = path[i]->x - path[i-1]->x;
        int dy1 = path[i]->y - path[i-1]->y;
        int dx2 = path[i+1]->x - path[i]->x;
        int dy2 = path[i+1]->y - path[i]->y;
        
        // Se c'è un cambio di direzione, includi il punto
        if (dx1 != dx2 || dy1 != dy2) {
            simplifiedPath[newPathLength++] = path[i];
        }
    }
    
    // Ultimo punto sempre incluso
    simplifiedPath[newPathLength++] = path[pathLength - 1];
    
    // Copia il percorso semplificato nell'array originale
    for (int i = 0; i < newPathLength; i++) {
        path[i] = simplifiedPath[i];
    }
    
    pathLength = newPathLength;
    Serial.printf("[A*] Percorso semplificato da %d a %d nodi\r\n", pathLength, newPathLength);
}

/**
 * Inizia il processo di ricerca del percorso A*
 * Questa funzione imposta solo i parametri iniziali
 */
void startPathFinding(int startX, int startY, int goalX, int goalY) {
    // Verifica i limiti
    if (startX < 0 || startX >= GRID_SIZE || startY < 0 || startY >= GRID_SIZE ||
        goalX < 0 || goalX >= GRID_SIZE || goalY < 0 || goalY >= GRID_SIZE) {
        Serial.println("[A*] Coordinate fuori griglia!");
        astarContext.state = A_STAR_DONE;
        pathFound = false;
        return;
    }
    
    // Se la meta è un ostacolo
    if (NODE_IS_OBSTACLE(grid[goalY][goalX])) {
        Serial.println("[A*] La destinazione è un ostacolo!");
        astarContext.state = A_STAR_DONE;
        pathFound = false;
        return;
    }
    
    // Pulisci eventuali percorsi precedenti
    clearPath();
    
    // Resetta stati rilevanti
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            grid[y][x].gCost = 0;
            grid[y][x].hCost = 0;
            grid[y][x].fCost = 0;
            NODE_CLEAR_VISITED(grid[y][x]);
            NODE_CLEAR_PATH(grid[y][x]);
            grid[y][x].parent = NULL;
        }
    }
    
    // Reset contatori
    openSetCount = 0;
    astarContext.nodesExplored = 0;
    
    // Imposta contesto
    astarContext.startX = startX;
    astarContext.startY = startY;
    astarContext.goalX = goalX;
    astarContext.goalY = goalY;
    astarContext.state = A_STAR_INIT;
    astarContext.lastYieldTime = millis();
    
    Serial.printf("[A*] Inizio ricerca percorso da (%d,%d) a (%d,%d)\r\n", startX, startY, goalX, goalY);
}

/**
 * Esegue un passo dell'algoritmo A* 
 * Restituisce true quando il processo è completato, false se ancora in corso
 */
bool processPathFindingStep() {
    const unsigned long YIELD_INTERVAL = 10; // ms tra yield
    
    // Se richiesto, cedi il controllo periodicamente
    unsigned long currentTime = millis();
    if (currentTime - astarContext.lastYieldTime >= YIELD_INTERVAL) {
        astarContext.lastYieldTime = currentTime;
        yield(); // Cede il controllo per evitare watchdog reset
    }
    
    switch (astarContext.state) {
        case A_STAR_IDLE:
            // Niente da fare
            return true;
            
        case A_STAR_INIT:
            {
                // Inizializza il nodo di partenza
                Node* startNode = &grid[astarContext.startY][astarContext.startX];
                startNode->gCost = 0;
                startNode->hCost = calculateHCost(astarContext.startX, astarContext.startY, 
                                                 astarContext.goalX, astarContext.goalY);
                startNode->fCost = startNode->hCost;
                
                // Aggiungi alla lista aperta
                addToOpenSet(startNode);
                
                // Passa allo stato di ricerca
                astarContext.state = A_STAR_SEARCH;
                return false;
            }
            
        case A_STAR_SEARCH:
            {
                // Se non ci sono più nodi da esplorare, termina
                if (openSetCount == 0) {
                    Serial.println("[A*] Nessun percorso trovato");
                    astarContext.state = A_STAR_DONE;
                    pathFound = false;
                    return true;
                }
                
                // Ottieni il nodo più promettente
                Node* currentNode = getLowestFCostNode();
                
                // Verifica se abbiamo raggiunto la meta
                if (currentNode->x == astarContext.goalX && currentNode->y == astarContext.goalY) {
                    astarContext.currentNode = currentNode;
                    astarContext.state = A_STAR_RECONSTRUCT;
                    Serial.println("[A*] Meta raggiunta! Ricostruzione percorso...");
                    return false;
                }
                
                // Marca come visitato
                NODE_SET_VISITED(*currentNode);
                astarContext.nodesExplored++;
                
                // Controlla limite nodi esplorati
                if (astarContext.nodesExplored >= MAX_NODES_TO_EXPLORE) {
                    Serial.println("[A*] Limite nodi raggiunto!");
                    
                    // In modalità esplorazione, usa il nodo più vicino alla meta
                    if (navigationMode == NAVIGATION_MODE_EXPLORATION) {
                        Node* bestNode = NULL;
                        int bestHCost = INT_MAX;
                        
                        for (int i = 0; i < openSetCount; i++) {
                            if (openSet[i]->hCost < bestHCost) {
                                bestNode = openSet[i];
                                bestHCost = openSet[i]->hCost;
                            }
                        }
                        
                        if (bestNode != NULL) {
                            astarContext.currentNode = bestNode;
                            astarContext.state = A_STAR_RECONSTRUCT;
                            Serial.println("[A*] Usando il nodo più vicino al goal");
                            return false;
                        }
                    }
                    
                    astarContext.state = A_STAR_DONE;
                    pathFound = false;
                    return true;
                }
                
                // Esplora i vicini
                for (int i = 0; i < 8; i++) {
                    int newX = currentNode->x + directions[i].dx;
                    int newY = currentNode->y + directions[i].dy;
                    
                    // Verifica che sia all'interno della griglia
                    if (newX < 0 || newX >= GRID_SIZE || newY < 0 || newY >= GRID_SIZE) {
                        continue;
                    }
                    
                    Node* neighbor = &grid[newY][newX];
                    
                    // Salta nodi visitati o con ostacoli
                    if (NODE_IS_VISITED(*neighbor) || NODE_IS_OBSTACLE(*neighbor)) {
                        continue;
                    }
                    
                    // Calcola il nuovo costo g
                    uint16_t newGCost = currentNode->gCost + directions[i].cost;
                    
                    // Se il nodo non è nella lista aperta o il nuovo percorso è migliore
                    if (!isInOpenSet(neighbor) || newGCost < neighbor->gCost) {
                        neighbor->gCost = newGCost;
                        neighbor->hCost = calculateHCost(newX, newY, astarContext.goalX, astarContext.goalY);
                        neighbor->fCost = neighbor->gCost + neighbor->hCost;
                        neighbor->parent = currentNode;
                        
                        // Se il nodo non è nella lista aperta, aggiungilo
                        if (!isInOpenSet(neighbor)) {
                            addToOpenSet(neighbor);
                        }
                    }
                }
                
                return false;
            }
            
        case A_STAR_RECONSTRUCT:
            {
                // Ricostruisci il percorso
                Node* current = astarContext.currentNode;
                
                // Ricostruisci percorso dalla meta alla partenza
                pathLength = 0;
                while (current != NULL) {
                    NODE_SET_PATH(*current);
                    
                    if (pathLength < GRID_SIZE * 2) {
                        path[pathLength++] = current;
                    }
                    
                    current = current->parent;
                    
                    // Cede il controllo ogni pochi nodi
                    if (pathLength % 5 == 0) {
                        yield();
                    }
                }
                
                // Inverti il percorso per averlo dalla partenza alla meta
                for (int i = 0; i < pathLength / 2; i++) {
                    Node* temp = path[i];
                    path[i] = path[pathLength - i - 1];
                    path[pathLength - i - 1] = temp;
                }
                
                // Semplifica il percorso
                simplifyPath();
                
                Serial.printf("[A*] Percorso trovato! Lunghezza: %d (esplorati: %d nodi)\r\n", 
                            pathLength, astarContext.nodesExplored);
                
                astarContext.state = A_STAR_DONE;
                pathFound = true;
                return true;
            }
            
        case A_STAR_DONE:
            return true;
            
        default:
            return true;
    }
}

/**
 * Versione cooperativa di findPath che non causa watchdog reset
 */
bool findPath(int startX, int startY, int goalX, int goalY) {
    // Avvia la ricerca
    startPathFinding(startX, startY, goalX, goalY);
    
    // Durante lo sviluppo/debug, possiamo eseguire tutto in una volta
    // In produzione, dovremmo chiamare questo da loop() finché non restituisce true
    unsigned long startTime = millis();
    
    // Esegui l'elaborazione A* per un massimo di 100ms per evitare watchdog timeout
    while (millis() - startTime < 100) {
        if (processPathFindingStep()) {
            // Processo completato
            return pathFound;
        }
        
        // Cedi il controllo brevemente
        yield();
    }
    
    // Se arriviamo qui, abbiamo superato il timeout, ma possiamo 
    // continuare in un ciclo successivo
    Serial.println("[A*] Timeout temporaneo, riprenderò nel prossimo ciclo");
    return false;
}

/**
 * Ricostruisce il percorso per la visualizzazione
 */
void reconstructPath() {
    if (pathLength == 0) {
        Serial.println("[A*] Nessun percorso da ricostruire!");
        return;
    }
    
    Serial.println("[A*] Ricostruzione del percorso:");
    
    // Stampa solo i punti principali per risparmiare spazio
    if (pathLength > 10) {
        // Stampa i primi 3 punti
        for (int i = 0; i < 3 && i < pathLength; i++) {
            Serial.printf("  Passo %d: (%d, %d)\r\n", i, path[i]->x, path[i]->y);
        }
        
        Serial.println("  ...");
        
        // Stampa gli ultimi 3 punti
        for (int i = max(3, pathLength - 3); i < pathLength; i++) {
            Serial.printf("  Passo %d: (%d, %d)\r\n", i, path[i]->x, path[i]->y);
        }
    } else {
        // Stampa tutto il percorso
        for (int i = 0; i < pathLength; i++) {
            Serial.printf("  Passo %d: (%d, %d)\r\n", i, path[i]->x, path[i]->y);
        }
    }
}

/**
 * Verifica se il percorso corrente è ancora valido
 */
bool isPathValid() {
    for (int i = 0; i < pathLength; i++) {
        if (NODE_IS_OBSTACLE(*path[i])) {
            Serial.printf("[A*] Percorso non valido: ostacolo in (%d, %d)\r\n", path[i]->x, path[i]->y);
            return false;
        }
    }
    return true;
}

/**
 * Imposta la modalità di navigazione
 */
void setNavigationMode(uint8_t mode) {
    navigationMode = mode;
    Serial.printf("[A*] Modalità di navigazione: %s\r\n", 
                 mode == NAVIGATION_MODE_DIRECT ? "Diretta" : "Esplorativa");
}

/**
 * Controlla se una destinazione è raggiungibile
 */
bool isGoalReachable(int startX, int startY, int goalX, int goalY) {
    // Calcola la distanza euclidea tra start e goal
    float distance = sqrt(pow(goalX - startX, 2) + pow(goalY - startY, 2)) * GRID_RESOLUTION;
    
    // Se la distanza è troppo grande, probabilmente non è raggiungibile direttamente
    if (distance > MAX_DISTANCE / 2) {
        return false;
    }
    
    // Prova a trovare un percorso diretto
    return findPath(startX, startY, goalX, goalY);
}

/**
 * Trova un punto intermedio verso la destinazione
 */
bool findIntermediatePoint(int startX, int startY, int goalX, int goalY, int& intermediateX, int& intermediateY) {
    // Calcola la direzione verso il goal
    int dx = goalX - startX;
    int dy = goalY - startY;
    float distance = sqrt(dx*dx + dy*dy);
    
    if (distance < 1.0f) {
        return false; // Goal troppo vicino
    }
    
    // Normalizza
    float norm_dx = dx / distance;
    float norm_dy = dy / distance;
    
    // Trova un punto a metà strada
    float targetDistance = min(MAX_NODES_TO_EXPLORE / 2.0f, distance / 2.0f);
    intermediateX = startX + round(norm_dx * targetDistance);
    intermediateY = startY + round(norm_dy * targetDistance);
    
    // Assicurati che sia all'interno della griglia
    intermediateX = constrain(intermediateX, 0, GRID_SIZE - 1);
    intermediateY = constrain(intermediateY, 0, GRID_SIZE - 1);
    
    // Verifica che non sia un ostacolo
    if (NODE_IS_OBSTACLE(grid[intermediateY][intermediateX])) {
        // Cerca un punto vicino libero
        for (int r = 1; r < 5; r++) {
            for (int i = 0; i < 8; i++) {
                int testX = intermediateX + directions[i].dx * r;
                int testY = intermediateY + directions[i].dy * r;
                
                if (testX >= 0 && testX < GRID_SIZE && testY >= 0 && testY < GRID_SIZE) {
                    if (!NODE_IS_OBSTACLE(grid[testY][testX])) {
                        intermediateX = testX;
                        intermediateY = testY;
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    return true;
}