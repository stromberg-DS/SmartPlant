/* 
 * Pump/Relay Tests
 * Author: Daniel Stromberg
 * Date: 3/25/2024
*/

#include "Particle.h"
#include <Button.h>

const int BUTTON_PIN = D2;
const int PUMP_PIN = D19;
const int PUMP_ON_TIME = 500;
const int SOIL_PIN = A1;
const int sensorInWater = 1744;     //when sensor is in pure pureWater
const int sensorInAir = 3485;       //sensor value in air
const int sensorInWetSoil = 1770;   //sensor value in VERY wet sensor
const int sensorInDrySoil = 3000;   //sensor value in dry soil
const int MIN_WATERING_INTERVAL = 5000;    //Minimum time between watering.

int currentMillis;
int lastMillis = -9999;


SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

Button moistureButton(BUTTON_PIN, true);

void setup() {
    Serial.begin(9600);
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(SOIL_PIN, INPUT);
}

void loop() {
    currentMillis = millis();
    int moistureLevel = analogRead(SOIL_PIN);
    int timeSinceLastPump = currentMillis - lastMillis;

    if(currentMillis %1000 == 0){
        Serial.printf("Moisture Level: %i\n", moistureLevel);
        Serial.printf("TimeSincePump: %i\n\n", timeSinceLastPump);
    }

    if(timeSinceLastPump > PUMP_ON_TIME){
        digitalWrite(PUMP_PIN, LOW);

    }

    if(moistureLevel >3000 && timeSinceLastPump > MIN_WATERING_INTERVAL){
        digitalWrite(PUMP_PIN, HIGH);
        lastMillis = currentMillis;
    }


}
