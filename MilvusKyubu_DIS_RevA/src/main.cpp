#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Nextion.h>

//#define nextion Serial1

#define HOSTNAME "Milvus Kyubu DIS"
#define FIRMWARE "D1MINI_OTA_MQ_DIS_RevA_v11"


//#define DEBUG


// DISPLAY variables


int levelY = 0;
int levelX = 0;

String pitch;
String roll;
String dim;

int pitchMin = 50;
int pitchMax = 65;
int rollMin = 35;
int rollMax = 65;


int touchRes[16];
int currentPageId;

int pageNumber;
int componentNumber;
int pressEvent;
int numericalData;


// WIFI & MQTT

//const char* ssid = "TheMatrix";
//const char* password = "tamghI,.ocl17";

const char* ssid = "milvus-net";
const char* password = "superbrandls";

unsigned long lastPub;
unsigned long now;
unsigned long pubInt = 200;
char payload[8];
char jsonarray[64];

float EMA_a = 0.15;      //initialization of EMA alpha
float pitchEMA = 0;          //initialization of EMA S
float rollEMA = 0;          //initialization of EMA S



#define SERVER            "192.168.1.50"
#define SERVERPORT        1883
#define MQTT_USERNAME     "mqttmaster"
#define MQTT_PW           "mqttmaster"
#define USERNAME          "mqttmaster"
#define TOPIC             "dis1/"
#define T_STATE           "state"
#define T_COMMAND         "command"
#define T_YPR             "ypr"
#define T_FIRMWARE        "fw"
#define T_LOCALIP         "ip"



#define GREEN 5806
#define RED 53890


// EEPROM storage

int eeAdd = 0;


WiFiClient netclient;
PubSubClient mqttClient(netclient);

//SoftwareSerial nextion(DIS_RX, DIS_TX);  // RX, TX
Nextion display(Serial1, 115200);

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void updateDisplay() {

  
  Serial.print(".");
  dim = "dim=" + dim;

  char __dim[sizeof(dim)];
  dim.toCharArray(__dim, sizeof(__dim));

  display.sendCommand(__dim);
  dim="";


  levelX = constrain(roll.toFloat()*10, -50, 50)+50;
  levelY = constrain(pitch.toFloat()*10, -50, 50)+50;


  display.setComponentText("v20", pitch);
  display.setComponentText("v21", roll);

  display.setComponentValue("h0", levelX);
  display.setComponentValue("h1", levelY);

  if (levelX <= rollMax && levelX >= rollMin){
            display.sendCommand("h0.bco=5806");
  }
  else {
            display.sendCommand("h0.bco=53890");
  }
  if (levelY <= pitchMax && levelY >= pitchMin){
            display.sendCommand("h1.bco=5806");
  }
  else {
            display.sendCommand("h1.bco=53890");
  }

            // display.sendCommand("h1.bco=53890"); //senso_orange
            // display.sendCommand("h1.bco=5338"); //senso_blue
            // display.sendCommand("h1.bco=63488"); //red
            // display.sendCommand("h1.bco=5806"); //green
            // display.sendCommand("h1.bco=30174"); //default_blue


}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(HOSTNAME, MQTT_USERNAME, MQTT_PW)) {
      Serial.println("connected");

      mqttClient.publish(TOPIC T_STATE, "ONLINE");

      IPAddress myIp = WiFi.localIP();
      sprintf(payload, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
      mqttClient.publish(TOPIC T_LOCALIP, payload);

      mqttClient.publish(TOPIC T_FIRMWARE, FIRMWARE);

      // subscribe to topics
      mqttClient.subscribe(TOPIC T_COMMAND);
      mqttClient.subscribe(TOPIC T_YPR);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte * data,  int length) {

  String strTopic = String((char*)topic);
  String strPayload;

  /*
  if (strTopic == "dis1/pubint") {

    for (int i = 0; i < length; i++) {
      strPayload += (char)data[i];
    }
    pubInt = strPayload.toInt();
    Serial.println(pubInt);

  }
  */


  if (strTopic == "dis1/command") {

      const size_t bufferSize2 = JSON_OBJECT_SIZE(1) + 10;
      DynamicJsonBuffer jsonBuffer2(bufferSize2);
      
      for (int i = 0; i < length; i++) {
        strPayload += (char)data[i];
      }

      JsonObject& root = jsonBuffer2.parseObject(strPayload);

      const char* dimC = root["dim"]; // "23"
      dim = String (dimC);

      Serial.println(strPayload);

      updateDisplay();

  }

  if (strTopic == "dis1/ypr") {

      const size_t bufferSize = JSON_OBJECT_SIZE(3) + 30;
      DynamicJsonBuffer jsonBuffer(bufferSize);

      for (int i = 0; i < length; i++) {
        strPayload += (char)data[i];
      }

      JsonObject& root = jsonBuffer.parseObject(strPayload);

      //const char* yawC = root["yaw"]; // "23"
      const char* pitchC = root["pitch"]; // "23"
      const char* rollC = root["roll"]; // "-5"

      //pitch = String (pitchC);
      //roll = String (rollC);
      
      int pitchInt = atoi(pitchC);
      int rollInt = atoi(rollC);
      pitchEMA = (EMA_a*pitchInt) + ((1-EMA_a)*pitchEMA);
      rollEMA  = (EMA_a*rollInt) + ((1-EMA_a)*rollEMA);

      char pitch_dis[6];
      char roll_dis[6];
      /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
      dtostrf(pitchEMA, 4, 1, pitch_dis);
      dtostrf(rollEMA, 4, 1, roll_dis);

      pitch =  pitch_dis;
      roll = roll_dis;

      updateDisplay();

    }
}

void readDisplay () {

  while (Serial.available() > 0) {
    for (int i = 1 ; i < 16; i++) {
      touchRes[i] =  Serial.read();
      //Serial.print(touchRes[i]);
      //delay(20);
    }
    touchRes[15] = '\0';

    pageNumber = touchRes[2];
    currentPageId = touchRes[2]; 
    componentNumber = touchRes[3];
    pressEvent = touchRes[4];
    numericalData = touchRes[9];


    for (int i = 1 ; i < 16; i++) {
      touchRes[i] = ' '  ;
      //Serial.print(touchRes[i]);
      //delay(20);
    }

    #ifdef DEBUG

    Serial.print("pageNumber: ");
    Serial.println(pageNumber);
    Serial.print("componentNumber: ");
    Serial.println(componentNumber);
    Serial.print("pressEvent: ");
    Serial.println(pressEvent);
    Serial.print("numericalData: ");
    Serial.println(numericalData);
    Serial.println("--------------");
    Serial.println(currentPageId);
    Serial.println("--------------");

    #endif

  }

  if (pressEvent == 0)  {
      display.pageId();
      //Serial.println();
  }



}

void setup() {

  Serial1.begin(115200);                   // Display
  Serial.begin(115200);                    // Console Debug

  Serial.println();
  Serial.println("Booting...");

  WiFi.hostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }


  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(HOSTNAME); // Hostname defaults to esp8266-[ChipID]

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });


  ArduinoOTA.begin();


  Serial.println("Online!");
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE);
  printWifiStatus();

  display.init();
  display.sendCommand("page title");
  display.sendCommand("dim=80");
  display.sendCommand("bkcmd=0");

  //display.sendCommand("bauds=115200");

  mqttClient.setServer(SERVER, SERVERPORT);
  mqttClient.setCallback(callback);

}

void loop() {

  ArduinoOTA.handle();

  // start code here -----------------------------------------------------------
  if (!mqttClient.connected()) {
    reconnect();
  }

  readDisplay();
  mqttClient.loop();

  // end code here -----------------------------------------------------------

}
