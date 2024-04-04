/* 
 * Threaded Dust Sensor Test
 * Author: Daniel Stromberg
 * Date: 4/3/24
 */

#include "Particle.h"
#include "math.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"

const int dustPin = 10;
const int sampleTime = 30000;
int duration;
int totalLowTime = 0;
int lastLowTime = 0;
int startTime;
float concentration = 0;
int lastTime = 0;
int updateTime = 5000;
int lowpulseoccupancy;
float ratio = 0;

TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish dustPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plantinfo.dustsensor");

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

//functions
void MQTT_connect();
bool MQTT_ping();
void getConc();

void setup() {
    Serial.begin(9600);
    new Thread("concThread", getConc);
}

void loop() {
    if((millis()-lastTime)>updateTime){
      Serial.printf("Time: %0.2f, Conc: %0.2f, Ratio: %0.2f\n", millis()/1000.0, concentration, ratio);
      lastTime = millis();
    }
}

void getConc(){
  unsigned int duration, startTime;
  startTime = 0;
  lowpulseoccupancy = 0;
  while(true){
    duration = pulseIn(dustPin, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;\
    if ((millis()-startTime) > sampleTime){

      ratio = lowpulseoccupancy/(sampleTime*10.0);
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
      startTime = millis();
      lowpulseoccupancy = 0;
    }
  }
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