/* 
 * Pump/Relay Tests
 * Author: Daniel Stromberg
 * Date: 3/25/2024
*/

#include "Particle.h"
#include <Button.h>

const int buttonPin = D2;
const int pumpPin = D19;
const int pumpOnTime = 2000;

int currentMillis;
int lastMillis = -9999;


SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

Button moistureButton(buttonPin, true);

void setup() {
    Serial.begin(9600);
    pinMode(pumpPin, OUTPUT);
}

void loop() {
    currentMillis = millis();
    int timeSinceLastPump = currentMillis - lastMillis;

    if(moistureButton.isPressed()){
        digitalWrite(pumpPin, HIGH);
        lastMillis = currentMillis;
        Serial.printf("Button!\n");
    }

    if(timeSinceLastPump > pumpOnTime){
        digitalWrite(pumpPin, LOW);

    }
}
