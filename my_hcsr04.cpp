#include "header.h"
#include <HCSR04.h>

// Initialize sensor that uses digital pins 13 and 12.
float distanceSx = 0.0;
float distanceCe = 0.0;
float distanceDx = 0.0;

UltraSonicDistanceSensor distanceSensorSx(DIST_TRIG, DIST_SX_ECHO);
UltraSonicDistanceSensor distanceSensorCe(DIST_TRIG, DIST_CE_ECHO);
UltraSonicDistanceSensor distanceSensorDx(DIST_TRIG, DIST_DX_ECHO);

float getDistanceSx() { return distanceSx; }
float getDistanceCe() { return distanceCe; }
float getDistanceDx() { return distanceDx; }

void initHcsr04() {
    Serial.begin(115200);  // We initialize serial connection so that we could print values from sensor.
}

//serve delay() perchè altrimenti vanno in conflitto (usano lo stesso trigger)
void readHcsr04() {
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    delay(200);
    distanceSx = distanceSensorSx.measureDistanceCm();
    delay(200);
    distanceCe = distanceSensorCe.measureDistanceCm();
    delay(200);
    distanceDx = distanceSensorDx.measureDistanceCm();
    Serial.println("SX: " + String(distanceSx) + " CE: " + String(distanceCe) + " DX: " + String(distanceDx));
    delay(500);
}
