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

const char* mqtt_server = "test.mosquitto.org";
const char* mqttClientId = "ESP32Client";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Wire.begin();
  //Wire.setClock(100000);
         // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  delay(500);
  setup_wifi();
  delay(2000);

  client.setServer(mqtt_server, 1883);
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


void loop() {
  if (!client.connected()) {
    if (client.connect(mqttClientId)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print(client.state());
      Serial.println("Connection failed. Retrying...");
      delay(2000);
      return;
    }
  }
 startTime = millis();
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
  if (client.connected()) {
    Serial.print("Hello");
    client.publish("esp32/IMU", imuData.c_str(), false);
  }
  //Each loop should be at least 20ms.
  while(millis() - startTime < (DT*1000))
        {
            delay(1);
        }
  Serial.println( millis()- startTime);
 


}
