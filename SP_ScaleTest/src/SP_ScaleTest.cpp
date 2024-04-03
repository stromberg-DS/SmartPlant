/* 
 * SP Scale Test
 * Author: Daniel Stromberg
 * Date: 03/31/2024
*/

#include "Particle.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include "HX711.h"

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

HX711 myScale(D6,D7);
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish scalePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartscale");

const float CAL_FACTOR = -478.79;
const int SAMPLES = 10;

float weight;
int offset;
int lastPublishTime = -99999;

//weights
const float EMPTY_WEIGHT = 130.0; //Completely empy cup with pump
const float LOW_WEIGHT = 230; //Lowest before pump stops working
const float FULL_WEIGHT = 625.0; //Completely full with pump

//functions
void MQTT_connect();
bool MQTT_ping();

void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);

    WiFi.on();
    WiFi.connect();
    while(WiFi.connecting()) {
    Serial.printf(".");
    }
    Serial.printf("\n\n");

    myScale.set_scale();
    delay(5000);
    myScale.tare();
    myScale.set_scale(CAL_FACTOR);
}

void loop() {
  static int timeSincePublish;
  timeSincePublish = millis() - lastPublishTime;
  weight = constrain(myScale.get_units(SAMPLES),0, 5000);

  if(timeSincePublish > 6000){
    if(mqtt.Update()){
      scalePub.publish(weight);
      Serial.printf("Weight: %0.2f\n", weight);
    }
    lastPublishTime = millis();
  }
    
  MQTT_connect();
  MQTT_ping();

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