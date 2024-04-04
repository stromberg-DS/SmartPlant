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
#include "HX711.h"
#include "math.h"

//Sensor 
const int DUST_PIN = 10;
const int AIR_QUALITY_PIN = A0;
const int SAMPLE_TIME_MS = 30000;
const int BME_ADDRESS = 0x76;
const int SOIL_PIN = A1;

//Display 
const int OLED_RESET =-1;
const int OLED_ADDRESS = 0x3C;
const int DEGREE_SYMBOL = 0xF8;

//Pump 
const int PUMP_PIN = D19;
const int PUMP_ON_TIME = 500;
const int PUMP_TIMEOUT = 30000;
int lastPumpTime = -9999;
bool isPumpOn = false;
bool isWebButtonPressed;
bool isManualPumping = false;

//Moisture 
const int SENSOR_IN_WATER = 1744;       //when sensor is in pure pureWater
const int SENSOR_IN_AIR = 3485;         //sensor value in air
const int SENSOR_IN_WET_SOIL = 1770;    //sensor value in VERY wet soil
const int SENSOR_IN_DRY_SOIL = 3000;    //sensor value in dry soil

//Scale 
const float CAL_FACTOR = -478.79;
const int SAMPLES = 10;
const float EMPTY_WEIGHT = 130.0;   //No water, pump and cup only
const float LOW_WEIGHT = 230.0;     //Lowest water level before pump stops working
const float FULL_WEIGHT = 625.0;    //Full of water. 

//Timing
int currentMillis;
int lastDisplayTime = 0;

//Dust/VOC Sensors
int duration;
int totalLowTime = 0;
int lowPulseOccupancy = 0;
int lastLowTime = 0;
int lastSampleTime;
float ratio = 0;
float concentration = 0;
String warningText;

//BME
bool status;


TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish dustPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantinfo.dustsensor");
Adafruit_MQTT_Publish airQualityText = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airqualitywarningtext");
Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantinfo.humidity");
Adafruit_MQTT_Publish scalePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartscale");
Adafruit_MQTT_Publish moisturePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantinfo.soilmoisture");
Adafruit_MQTT_Subscribe waterButtonFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttononoff");

AirQualitySensor sensor(AIR_QUALITY_PIN);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;
HX711 myScale(D6, D7);

String DateTime, TimeOnly, Day, MonthDate, Year;

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

//functions
void updateDisplays(String _airQuality, float _tempF, float _humidity, float _moisturePercent);
float tempCtoF(float c);
void MQTT_connect();
bool MQTT_ping();
void getConc();
float mapFloat(float inVal, float minIn, float maxIn, float minOut, float maxOut);


void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);    

    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(SOIL_PIN, INPUT);

    Time.zone(-7);
    Time.beginDST();
    Particle.syncTime();

    status = bme.begin(BME_ADDRESS);
    if (status == false){
      Serial.printf("BME Failed to start at address 0x%02X", BME_ADDRESS);
    }

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.display();

    myScale.set_scale();
    delay(5000);        //let scale settle
    myScale.tare();
    myScale.set_scale(CAL_FACTOR);

    mqtt.subscribe(&waterButtonFeed);

    pinMode(DUST_PIN, INPUT);
    lastSampleTime = millis();
    new Thread("concThread", getConc);
}

void loop() {
    currentMillis = millis();
    float weight;
    int quality;
    float waterPercent;
    float tempC;
    float tempF;
    float humidity;
    int moistureLevel = analogRead(SOIL_PIN);
    int moisturePercent;
    int timeSinceLastPump = currentMillis - lastPumpTime;

    DateTime = Time.timeStr();
    MonthDate = DateTime.substring(4, 10);
    TimeOnly =  DateTime.substring(11, 19);
    Day = DateTime.substring(0,3);

    //update Sensors/display
    // if (millis() - lastDisplayTime > 3000){
        moisturePercent = map(moistureLevel, SENSOR_IN_DRY_SOIL, SENSOR_IN_WET_SOIL, 0, 100);
        quality = sensor.slope();
        weight = constrain(myScale.get_units(SAMPLES), 0, 5000);
        waterPercent = constrain(mapFloat(weight, LOW_WEIGHT, FULL_WEIGHT, 0, 100), 0, 110);
        tempF = tempCtoF(bme.readTemperature());
        humidity = bme.readHumidity();
        if(millis() - lastDisplayTime > 30000){
        updateDisplays(warningText, tempF, humidity, moisturePercent);
        // Serial.printf("Moisture: %i\n", moistureLevel);
        // Serial.printf("Moist Percent: %i\n", moisturePercent);
        lastDisplayTime = millis();
    }


    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))){
        if(subscription == &waterButtonFeed){
            //atof: ascii to float
            //char * turns binary into characters
            isWebButtonPressed = bool(atof((char *)waterButtonFeed.lastread));
        }
    }


    if(millis()-lastSampleTime > SAMPLE_TIME_MS){
        Serial.printf("Dust Concentration: %0.2f\n", concentration);
        lastSampleTime = millis();

        if(quality == 0){
            warningText = "Extreme VOCs!";
        }else if(quality == 1){
            warningText = "High VOCs.";
        } else if (quality == 2){
            warningText = "Low VOCs";
        } else if(quality == 3){
            warningText = "Fresh Air!";
        }

        if(mqtt.Update()){
            scalePub.publish(waterPercent); 
            dustPub.publish(concentration);
            moisturePub.publish(moisturePercent);
            tempPub.publish(tempF);
            humidityPub.publish(humidity);
            airQualityText.publish(warningText);
            Serial.printf("Water Percent: %f\n", waterPercent);
            Serial.printf("Moist Percent: %i\n\n", moisturePercent);
        }
    }
  
    if(timeSinceLastPump > PUMP_ON_TIME){
        digitalWrite(PUMP_PIN, LOW);
    }

    if((moistureLevel >SENSOR_IN_DRY_SOIL && timeSinceLastPump > PUMP_TIMEOUT) || isWebButtonPressed){
        digitalWrite(PUMP_PIN, HIGH);
        lastPumpTime = currentMillis;
    }

    MQTT_connect();
    MQTT_ping();

}

//Convert sensor temperature from celcius to Fahrenheit
float tempCtoF(float c){
    return ((c*9/5)+32);
}

float mapFloat(float inVal, float minIn, float maxIn, float minOut, float maxOut){
    return (inVal-minIn)*(maxOut-minOut)/(maxIn-minIn) + minOut;
}

//Update all local displays at once
void updateDisplays(String _airQuality, float _tempF, float _humidity, float _moisturePercent){
    // Serial.printf("Temperature: %0.1fF\n\n", tempF);
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.printf("%0.0f%cF %0.0f%%\n", _tempF, DEGREE_SYMBOL, _humidity);
    display.setTextSize(1);
    display.printf("\nSoil: %0.0f%%\n", _moisturePercent);
    display.printf("Dust: %0.0f\n%s\n\n", concentration, _airQuality.c_str());
    display.printf("%s, %s %s\n", Day.c_str(), MonthDate.c_str(), TimeOnly.c_str());
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
            if(totalLowTime == 0){
                totalLowTime = lastLowTime;
            } else{
                lastLowTime = totalLowTime;
            }

            ratio = lowPulseOccupancy/(sampleTime*10);
            concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2) +520*ratio +0.62;
            startTime = millis();
            lowPulseOccupancy=0;
        }
    }
    
}