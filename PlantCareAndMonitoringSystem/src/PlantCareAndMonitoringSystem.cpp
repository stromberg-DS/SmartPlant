/* 
 * Plant Care & Monitoring System
 * Author: Daniel Stromberg
 * Date: 3/26/2024
*/

#include "Particle.h"
#include "math.h"
#include <Air_Quality_Sensor.h>
#include <Adafruit_MQTT/Adafruit_MQTT_SPARK.h>
#include <Adafruit_MQTT/Adafruit_MQTT.h>
#include "credentials.h"
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Sensor constants
const int DUST_PIN = 10;
const int AIR_QUALITY_PIN = A0;
const int SAMPLE_TIME_MS = 30000;
const int BME_ADDRESS = 0x76;
const int SOIL_PIN = A1;

//Display constants
const int OLED_RESET =-1;
const int OLED_ADDRESS = 0x3C;
const int DEGREE_SYMBOL = 0xF8;

const int PUMP_PIN = D19;
const int PUMP_ON_TIME = 500;
const int PUMP_TIMEOUT = 5000;

const int SENSOR_IN_WATER = 1744;     //when sensor is in pure pureWater
const int SENSOR_IN_AIR = 3485;       //sensor value in air
const int SENSOR_IN_WET_SOIL = 1770;   //sensor value in VERY wet sensor
const int SENSOR_IN_DRY_SOIL = 3000;   //sensor value in dry soil

int currentMillis;
int lastPumpTime = -9999;
bool isPumpOn = false;

int duration;
int totalLowTime = 0;
int lowPulseOccupancy = 0;
int lastLowTime = 0;
int lastSampleTime;
float ratio = 0;
float concentration = 0;
String warningText;

bool status;
float tempC;
float tempF;
float humidity;
int lastDisplayTime = 0;

AirQualitySensor sensor(AIR_QUALITY_PIN);
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish dustPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantinfo.dustsensor");
Adafruit_MQTT_Publish airQualityPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantinfo.airquality");
Adafruit_MQTT_Publish airQualityText = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airqualitywarningtext");

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;


SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

//functions
void updateDisplays();
float tempCtoF(float c);
void MQTT_connect();
bool MQTT_ping();
void getConc();


void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(SOIL_PIN, INPUT);

    status = bme.begin(BME_ADDRESS);
    if (status == false){
      Serial.printf("BME Failed to start at address 0x%02X", BME_ADDRESS);
    }

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.display();

  pinMode(DUST_PIN, INPUT);
  lastSampleTime = millis();
  new Thread("concThread", getConc);
}

void loop() {
    currentMillis = millis();
    // int quality = sensor.slope();
    int moistureLevel = analogRead(SOIL_PIN);
    int timeSinceLastPump = currentMillis - lastPumpTime;

    if (millis() - lastDisplayTime > 10000){
        tempF = tempCtoF(bme.readTemperature());
        humidity = bme.readHumidity();
        updateDisplays();
        lastDisplayTime = millis();
    }

    //pulseIN is slowing me down!!! I need threading for this function!!!!
    ///////////////////////////


    if(millis()-lastSampleTime > SAMPLE_TIME_MS){
        // if(totalLowTime == 0){
        // totalLowTime = lastLowTime;
        // } else{
        // lastLowTime = totalLowTime;
        // }

        Serial.printf("Dust Concentration: %0.2f\n", concentration);
        lastSampleTime = millis();

        // if(quality == 0){
        //     Serial.printf("Extreme VOCs!\n");
        //     warningText = "Extreme VOCs!";
        // }else if(quality == 1){
        //     Serial.printf("High VOCs\n");
        //     warningText = "High VOCs.";
        // } else if (quality == 2){
        //     Serial.printf("Low VOCs\n");
        //     warningText = "Low VOCs";
        // } else if(quality == 3){
        //     Serial.printf("Clean Air!\n");
        //     warningText = "Fresh Air!";
        // }
        Serial.printf("\n");
        if(mqtt.Update()){
            dustPub.publish(concentration);
            // airQualityPub.publish(quality);
            // airQualityText.publish(warningText);
        }
    }

    //PUMP RELAY IS VERY SLOW! NEED TO BE FIXED.
  
    if(timeSinceLastPump > PUMP_ON_TIME){
        Serial.printf("Time since pump: %i\n", timeSinceLastPump);
        digitalWrite(PUMP_PIN, LOW);
    }

    if(moistureLevel >3000 && timeSinceLastPump > PUMP_TIMEOUT){
        digitalWrite(PUMP_PIN, HIGH);
        lastPumpTime = currentMillis;
    }

    MQTT_connect();
    // MQTT_ping();

}

//Convert sensor temperature from celcius to Fahrenheit
float tempCtoF(float c){
    return ((c*9/5)+32);
}


//Update all local displays at once
void updateDisplays(){
    // Serial.printf("Temperature: %0.1fF\n\n", tempF);
    display.clearDisplay();
    display.setCursor(0,0);
    display.printf("Temp: %0.1f%cF\n\n", tempF, DEGREE_SYMBOL);
    display.printf("Humidity: %0.1f%%", humidity);
    display.display();
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
    int8_t ret;

    // Return if already connected.
    if (mqtt.connected()) {
    return;
    }

    Serial.print("Connecting to MQTT... ");

    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
        Serial.printf("Retrying MQTT connection in 5 seconds...\n");
        mqtt.disconnect();
        delay(5000);  // wait 5 seconds and try again
    }
    Serial.printf("MQTT Connected!\n");
}

//Keeps the connection open to Adafruit
bool MQTT_ping() {
    static unsigned int last;
    bool pingStatus;

    if ((millis()-last)>120000) {
        Serial.printf("Pinging MQTT \n");
        pingStatus = mqtt.ping();
        if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
        }
        last = millis();
    }
    return pingStatus;
}

void getConc(){
    const int sampleTime = 30000;
    unsigned int duration, startTime;
    startTime = 0;
    lowPulseOccupancy =0;
    while (true)
    {
        duration = pulseIn(DUST_PIN, LOW);
        lowPulseOccupancy = lowPulseOccupancy + duration;
        if ((millis()-startTime)>sampleTime){
            ratio = lowPulseOccupancy/(sampleTime*10);
            concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2) +520*ratio +0.62;
            startTime = millis();
            lowPulseOccupancy=0;
        }
    }
    
}