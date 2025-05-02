#include "esp32-hal-gpio.h"
#include "header.h"

// Variabili per tracciare il passo corrente di ciascun motore
int posSx = 0;
int posDx = 0;

// Array dei pin dei motori (definiti in header.h)
int motorSX[4] = {MOTORSTEP_SX_1, MOTORSTEP_SX_2, MOTORSTEP_SX_3, MOTORSTEP_SX_4};
int motorDX[4] = {MOTORSTEP_DX_1, MOTORSTEP_DX_2, MOTORSTEP_DX_3, MOTORSTEP_DX_4};

/*
  Inizializza i pin dei motori come output e li imposta a LOW.
*/
void initMotor() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorSX[i], OUTPUT);
    pinMode(motorDX[i], OUTPUT);
  }
  lowPin();
}

/*
  Imposta tutti i pin dei motori a LOW.
*/
void lowPin() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorSX[i], LOW);
    digitalWrite(motorDX[i], LOW);
  }
}

/*
  Converte il numero di passi in millimetri percorsi dalla ruota.
*/
float stepsToMM(int steps) {
  float circumference = PI * WHEEL_DIAMETER;
  return (steps / (float)GIRO) * circumference;
}

/*
  Esegue un singolo passo per un motore specificato.
  Parametri:
    - motor: array dei pin del motore.
    - pos: riferimento al contatore del passo corrente da aggiornare.
    - direction: direzione del passo (+1 o -1).
*/
void singleStep(int motor[], int &pos, int direction) {
  pos = (pos + direction + 4) % 4;
  digitalWrite(motor[pos], HIGH);
}

/*
  Muove il motore (o entrambi) per un numero di passi.
  Parametri:
    - m: 's' per motore sinistro, 'd' per destro, 'a' per entrambi (case-insensitive).
    - n: numero di passi.
*/
void goMotor(char m, int n) {
  Serial.print("Movimento motore ");
  Serial.print(m);
  Serial.print(" - passi: ");
  Serial.print(n);
  Serial.print(" - distanza: ");
  Serial.print(stepsToMM(n));
  Serial.println(" mm");

  for (int i = 0; i < n; i++) {
    if (m == 's' || m == 'S') {
      singleStep(motorSX, posSx, 1);
    }
    else if (m == 'd' || m == 'D') {
      singleStep(motorDX, posDx, 1);
    }
    else if (m == 'a' || m == 'A') {
      singleStep(motorSX, posSx, 1);
      singleStep(motorDX, posDx, 1);
    }
    delay(SPED_MOTOR);
    lowPin();
  }
}

/*
  Ruota il robot di un certo angolo in gradi, calcolando il numero di passi in base
  alla geometria del robot.
  Parametri:
    - angle: angolo in gradi (valore assoluto).
    - direction: 'R' o 'r' per rotazione a destra, 'L' o 'l' per rotazione a sinistra.
  
  Calcolo dei passi:
    d = (angle * PI / 180) * (ROBOT_TRACK / 2)
    passi = d * GIRO / (PI * WHEEL_DIAMETER)
          = (angle * GIRO * ROBOT_TRACK) / (360 * WHEEL_DIAMETER)
*/
void rotate(int angle, char direction) {
  int steps = (abs(angle) * GIRO * ROBOT_TRACK) / (360 * WHEEL_DIAMETER);
  Serial.print("Rotazione ");
  Serial.print((direction == 'R' || direction == 'r') ? "destra" : "sinistra");
  Serial.print(" di ");
  Serial.print(angle);
  Serial.print(" gradi - passi: ");
  Serial.println(steps);
  
  for (int i = 0; i < steps; i++) {
    if (direction == 'R' || direction == 'r') {
      // Per ruotare a destra: motore sinistro indietro, motore destro avanti.
      singleStep(motorSX, posSx, -1);
      singleStep(motorDX, posDx, 1);
    }
    else if (direction == 'L' || direction == 'l') {
      // Per ruotare a sinistra: motore sinistro avanti, motore destro indietro.
      singleStep(motorSX, posSx, 1);
      singleStep(motorDX, posDx, -1);
    }
    delay(SPED_MOTOR);
    lowPin();
  }
}
