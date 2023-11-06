/*      This program  reads the angles and heading from the accelerometer, gyroscope
        and compass on a BerryIMU connected to an Arduino.


       The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported


       Feel free to do whatever you like with this code.
       Distributed as-is; no warranty is given.

       https://ozzmaker.com/berryimu/


*/
#include <WiFi.h>
#include <PubSubClient.h>
#include "IMU.h"
#include <Wire.h>
#define DT  0.02          // Loop time
#define G_GAIN 0.070    // [deg/s/LSB]

const char* ssid = "G.M.W.W.";
const char* password = "margotrobbie123";

byte buff[6];
int accRaw[3];
float AccYangle = 0.0;
float AccXangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;
float oldValueX = 0.0;
float oldValueY = 0.0;
float oldValueZ = 0.0;

unsigned long startTime;

// Replace your MQTT Broker IP address here:
const char* mqtt_server = "192.168.99.113";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Wire.begin();
  //Wire.setClock(100000);

  Serial.begin(115200);  // start serial for output
  
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  detectIMU();
  scan();
  enableIMU();


}
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connect_mqttServer() {
  // Loop until we're reconnected
  while (!client.connected()) {

        //first check if connected to wifi
        if(WiFi.status() != WL_CONNECTED){
          //if not connected, then first connect to wifi
          setup_wifi();
        }

        //now attemt to connect to MQTT server
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32_client")) { // Change the name of client here if multiple ESP32 are connected
          //attempt successful
          Serial.println("connected");
          // Subscribe to topics here
          client.subscribe("rpi/broadcast");
          //client.subscribe("rpi/xyz"); //subscribe more topics here
          
        } 
        else {
          //attempt not successful
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" trying again in 2 seconds");

          delay(2000);
        }
  }
  
}

//this function will be executed whenever there is data available on subscribed topics
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Check if a message is received on the topic "rpi/broadcast"
  if (String(topic) == "rpi/broadcast") {
      if(messageTemp == "10"){
        Serial.println("Action: blink LED");
      }
  }

  //Similarly add more if statements to check for other subscribed topics 
}


void loop() {
  if (!client.connected()) {
    connect_mqttServer();
  }

  long now = millis();
  //We only care about accell values for now, the IMU gyro calcs are trash
  //Read the measurements from  sensors
  readACC(buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
  if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
  if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;
  
  //trying to LPF
  accRaw[0] = .4 * accRaw[0] + oldValueX*(1-.4);
  accRaw[1] = .4 * accRaw[1] + oldValueY*(1-.4);
  accRaw[2] = .4 * accRaw[2] + oldValueZ*(1-.4);
  
  oldValueX = accRaw[0];
  oldValueY = accRaw[1];
  oldValueZ = accRaw[2];
  
  //Convert Accelerometer values to degrees
  AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
  AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;


  //If IMU is up the correct way, use these lines
        AccXangle -= (float)180.0;
  if (AccYangle > 90)
          AccYangle -= (float)270;
  else
    AccYangle += (float)90;
 
  Serial.print("#Raw Values 0 \t");
  Serial.print(accRaw[0]);
  Serial.print("\t### Raw 1\t");
  Serial.print(accRaw[1]);  
  Serial.print("\t### Raw 2\t");
  Serial.print(accRaw[2]);          
  Serial.print("#AccX\t");
  Serial.print(AccXangle);
  Serial.print("\t###  AccY  ");
  Serial.print(AccYangle);

  String imuData = String(AccXangle) + "," + String(AccYangle);
  if (now - lastMsg > 100) {
    lastMsg = now;
    client.publish("esp32/sensor1", imuData.c_str()); //topic name (to which this ESP32 publishes its data). 88 is the dummy value.
    
  }

}
