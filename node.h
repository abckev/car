#ifndef NODE_H
#define NODE_H

#include <stdint.h>


// Dimensione della griglia per A*
#define GRID_SIZE 50          // Numero di celle per lato della griglia
#define GRID_RESOLUTION 100   // mm per cella (10cm), permette di coprire 5mx5m con una griglia 50x50
#define GRID_CENTER (GRID_SIZE/2)

// Distanza massima in mm (5000mm = 5m)
#define MAX_DISTANCE 5000

// Parametri per ottimizzazione
#define MAX_NODES_TO_EXPLORE 200  // Limita il numero di nodi da esplorare
#define PATH_SIMPLIFICATION_THRESHOLD 3  // Numero minimo di nodi consecutivi per semplificare il percorso

// Flag per modalità di navigazione a lunga distanza
#define NAVIGATION_MODE_DIRECT 0     // Navigazione diretta quando possibile
#define NAVIGATION_MODE_EXPLORATION 1 // Navigazione esplorativa con aggiornamento dinamico

// Ottimizzazione della struttura del nodo (compatta per risparmiare memoria)
struct Node {
    int8_t x;           // Posizione X nella griglia (-128 a 127)
    int8_t y;           // Posizione Y nella griglia (-128 a 127)
    uint16_t gCost;     // Costo dal punto iniziale (16 bit è sufficiente)
    uint16_t hCost;     // Costo euristico verso il goal 
    uint16_t fCost;     // Costo totale
    uint8_t flags;      // Bit 0: obstacle, Bit 1: visited, Bit 2: path
    Node* parent;       // Puntatore al nodo padre
};

// Macro per accedere ai flag del nodo
#define NODE_IS_OBSTACLE(node) ((node).flags & 0x01)
#define NODE_SET_OBSTACLE(node) ((node).flags |= 0x01)
#define NODE_CLEAR_OBSTACLE(node) ((node).flags &= ~0x01)

#define NODE_IS_VISITED(node) ((node).flags & 0x02)
#define NODE_SET_VISITED(node) ((node).flags |= 0x02)
#define NODE_CLEAR_VISITED(node) ((node).flags &= ~0x02)

#define NODE_IS_PATH(node) ((node).flags & 0x04)
#define NODE_SET_PATH(node) ((node).flags |= 0x04)
#define NODE_CLEAR_PATH(node) ((node).flags &= ~0x04)

#endif // NODE_H