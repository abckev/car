#ifndef A_STAR_H
#define A_STAR_H

#include "node.h"

// Definizione delle costanti per A*
#define MOVE_COST_STRAIGHT 10  // Costo per movimento orizzontale/verticale
#define MOVE_COST_DIAGONAL 14  // Costo per movimento diagonale (√2 * 10)

// Dichiarazione della matrice per la mappa dell'ambiente
extern Node grid[GRID_SIZE][GRID_SIZE];
extern Node* openSet[MAX_NODES_TO_EXPLORE];
extern int openSetCount;
extern Node* path[GRID_SIZE * 2];
extern int pathLength;
extern bool pathFound;
extern uint8_t navigationMode;

// Struttura per le direzioni di movimento
struct Direction {
    int dx;
    int dy;
    int cost;
};

// Direzioni di movimento possibili (8 direzioni: 4 cardinali + 4 diagonali)
extern const Direction directions[8];

// Funzioni di utilità per A*
uint16_t calculateHCost(int x1, int y1, int x2, int y2);
void addToOpenSet(Node* node);
Node* getLowestFCostNode();
void removeFromOpenSet(int index);
bool isInOpenSet(Node* node);
void clearPath();
void simplifyPath();

// Funzioni principali dell'algoritmo A*
void initializeGrid();
bool findPath(int startX, int startY, int goalX, int goalY);
void reconstructPath();
bool isPathValid();

// Funzioni avanzate per gestione della navigazione
void setNavigationMode(uint8_t mode);
bool isGoalReachable(int startX, int startY, int goalX, int goalY);
bool findIntermediatePoint(int startX, int startY, int goalX, int goalY, int& intermediateX, int& intermediateY);

void startPathFinding(int startX, int startY, int goalX, int goalY);
bool processPathFindingStep();

#endif // A_STAR_H