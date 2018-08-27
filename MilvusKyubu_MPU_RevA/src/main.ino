#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <ArduinoJson.h>

#define HOSTNAME "Milvus Kyubu MPU"
#define FIRMWARE "D1MINI_OTA_MQ_MPU_RevA_v11"

//#define DEBUG

#define BLUELED D3

// MISC variables


// WIFI & MQTT

const char* ssid = "milvus-net";
const char* password = "superbrandls";

unsigned long lastPub;
unsigned long pubInt = 100;
char payload[8];
char jsonarray[64];
char jyaw[8];
char jpitch[8];
char jroll[8];

#define SERVER            "192.168.1.50"
#define SERVERPORT        1883
#define MQTT_USERNAME     "mqttmaster"
#define MQTT_PW           "mqttmaster"
#define USERNAME          "mqttmaster"
#define TOPIC             "mpu/"
#define T_STATE           "state"
#define T_COMMAND         "command"
#define T_FIRMWARE        "fw"
#define T_PUBINT          "pubint"
#define T_LOCALIP         "ip"
//#define T_YAW             "yaw"
//#define T_PITCH           "pitch"
//#define T_ROLL            "roll"
#define T_YPR             "ypr"

// EEPROM storage

int eeAdd = 0;

struct dataStore {
  float cyaw;
  float cpitch;
  float croll;
  int pubInt;
};

dataStore myDataStore;

// MPU variables

MPU6050 mpu;

//#define MPU6050_ACCEL_OFFSET_X -771
//#define MPU6050_ACCEL_OFFSET_Y -3
//#define MPU6050_ACCEL_OFFSET_Z 1277
//#define MPU6050_GYRO_OFFSET_X  31
//#define MPU6050_GYRO_OFFSET_Y  31
//#define MPU6050_GYRO_OFFSET_Z  31

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw = 0;
float pitch = 0;
float roll = 0;
float dyaw = 0;
float dpitch = 0;
float droll = 0;
float cyaw = 0;
float cpitch = 0;
float croll = 0;

WiFiClient netclient;
PubSubClient mqttClient(netclient);

void levelMPU () {

  cyaw = yaw*(-1);
  cpitch = pitch*(-1);
  croll = roll*(-1);

  //storeData();

}

void resetMPU () {

  cyaw = 0;
  cpitch = 0;
  croll = 0;

  //storeData();

}

void storeData () {

  myDataStore = {cyaw, cpitch, croll, pubInt};
  EEPROM.begin(512);
  EEPROM.put(eeAdd, myDataStore);
  EEPROM.end();

}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial1.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(HOSTNAME, MQTT_USERNAME, MQTT_PW)) {
      Serial1.println("connected");

      mqttClient.publish(TOPIC T_STATE, "ONLINE");

      IPAddress myIp = WiFi.localIP();
      sprintf(payload, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
      mqttClient.publish(TOPIC T_LOCALIP, payload);

      mqttClient.publish(TOPIC T_FIRMWARE, FIRMWARE);

      // subscribe to topics
      mqttClient.subscribe(TOPIC T_COMMAND);
      mqttClient.subscribe(TOPIC T_PUBINT);

    } else {
      Serial1.print("failed, rc=");
      Serial1.print(mqttClient.state());
      Serial1.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte * data, unsigned int length) {

  String strTopic = String((char*)topic);
  String strPayload;

  if (strTopic == "mpu/pubint") {

    for (int i = 0; i < length; i++) {
      strPayload += (char)data[i];
    }
    pubInt = strPayload.toInt();
    //Serial1.println(pubInt);

  }

  if (strTopic == "mpu/command") {

    if (data[0] == 'l' && data[1] == 'e' && data[2] == 'v')  {
      levelMPU();
      mqttClient.publish(TOPIC T_STATE, "LEVEL SET");
    }

    if (data[0] == 'r' && data[1] == 'e' && data[2] == 's')  {
      resetMPU();
      mqttClient.publish(TOPIC T_STATE, "LEVEL RESET");
    }

    if (data[0] == 's' && data[1] == 'a' && data[2] == 'v')  {
      storeData();
      mqttClient.publish(TOPIC T_STATE, "DATA SAVED");
    }

    if (data[0] == 's' && data[1] == 'p')  {
      for (int i = 2; i < length; i++) {
        strPayload += (char)data[i];
      }
      cpitch = strPayload.toFloat() - pitch;
      //storeData();
      mqttClient.publish(TOPIC T_STATE, "PITCH SET");
    }

    if (data[0] == 's' && data[1] == 'r')  {
      for (int i = 2; i < length; i++) {
        strPayload += (char)data[i];
      }
      croll = strPayload.toFloat() - roll;
      //storeData();
      mqttClient.publish(TOPIC T_STATE, "ROLL SET");
    }

    if (data[0] == 'm' && data[1] == 'p' && data[2] == 'u')  {
      for (int i = 2; i < length; i++) {
        strPayload += (char)data[i];
      }

      mqttClient.publish(TOPIC T_STATE, "RESTART MPU");
      ESP.restart();

    }

    if (data[0] == 'r' && data[1] == 'e' && data[2] == 'c')  {
      for (int i = 2; i < length; i++) {
        strPayload += (char)data[i];
      }

      mqttClient.publish(TOPIC T_STATE, "RECONNECT MQTT");
      reconnect();

    }

  }

}

void setup() {

  Serial.begin(115200);                    // Console Debug

  pinMode(BLUELED, OUTPUT);
  digitalWrite(BLUELED, HIGH);

  Serial.println("Booting");

  WiFi.hostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial1.println("Connection Failed! Rebooting...");
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
    Serial1.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial1.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial1.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial1.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial1.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial1.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial1.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial1.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial1.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.println("Online!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE);

  mqttClient.setServer(SERVER, SERVERPORT);
  mqttClient.setCallback(callback);


  // join I2C bus (I2Cdev library doesn't do this automatically)
  //#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000L);
    //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  //  Fastwire::setup(400, true);
  //#endif

  // initialize and test mpu

  Serial.print("init mpu...");
  mpu.initialize();
  //delay(1000);
  Serial.println(mpu.testConnection() ? F("ok") : F("failed"));
  Serial.print("init DMP...");
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {

      Serial.print("enabling DMP...");
      mpu.setDMPEnabled(true);

      Serial.println("ok");
      dmpReady = true;
      digitalWrite(BLUELED, LOW);
      packetSize = mpu.dmpGetFIFOPacketSize();

  } else {

      Serial.print("failed...code ");
      Serial.println(devStatus);
  }

  // read calibration value from EEPROM
  EEPROM.begin(512);
  EEPROM.get(eeAdd, myDataStore );
  EEPROM.end();

  cyaw = myDataStore.cyaw;
  cpitch = myDataStore.cpitch;
  croll = myDataStore.croll;

  if (myDataStore.pubInt < 1000) {
    pubInt  = myDataStore.pubInt;
  }

  #ifdef DEBUG

  Serial.println("EEPROM data: ");
  Serial.println("-------------------------");
  Serial.print("cyaw: ");
  Serial.println(myDataStore.cyaw);
  Serial.print("cpitch: ");
  Serial.println(myDataStore.cpitch);
  Serial.print("croll: ");
  Serial.println(myDataStore.croll);
  Serial.print("pubInt: ");
  Serial.println(myDataStore.pubInt);

  #endif

}

void loop() {
  ArduinoOTA.handle();

  // start code here -----------------------------------------------------------




//// START NON DMP HERE

while (fifoCount < packetSize) {

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if (!mqttClient.connected()) {
    reconnect();
  }

  if (mqttClient.connected() && millis() - lastPub > pubInt) {


    #ifdef DEBUG

    Serial.print("dy:  ");
    Serial.println(dyaw);
    Serial.print("dp:  ");
    Serial.println(dpitch);
    Serial.print("dr:  ");
    Serial.println(droll);

    Serial.println("--------------------------------------");


    Serial.print("cy:  ");
    Serial.println(cyaw);
    Serial.print("cp:  ");
    Serial.println(cpitch);
    Serial.print("cr:  ");
    Serial.println(croll);

    Serial.println("--------------------------------------");
    Serial.println("--------------------------------------");

    #endif


    //dtostrf(dyaw, 4, 1, payload);
    //mqttClient.publish(TOPIC T_YAW, payload);
    dtostrf(dyaw, 4, 1, jyaw);
    root["yaw"] = jyaw;

    //dtostrf(dpitch, 4, 1, payload);
    //mqttClient.publish(TOPIC T_PITCH, payload);
    dtostrf(dpitch, 4, 1, jpitch);
    root["pitch"] = jpitch;

    //dtostrf(droll, 4, 1, payload);
    //mqttClient.publish(TOPIC T_ROLL, payload);
    dtostrf(droll, 4, 1, jroll);
    root["roll"] = jroll;

    // publisg ypr in json

    root.printTo(jsonarray);
    mqttClient.publish(TOPIC T_YPR, jsonarray);


    lastPub = millis();

  }

  fifoCount = mpu.getFIFOCount();
  //Serial.print(F("+"));

}
  //// STOP NON DMP HERE-----------

    if (fifoCount == 1024) {
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("FIFO overflow!"));

              ESP.restart();

    }

    else {

     if (fifoCount % packetSize != 0) {
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("FIFO reset! "));

            ESP.restart();

     }
     else{

        while (fifoCount >= packetSize) {
           mpu.getFIFOBytes(fifoBuffer,packetSize);
           mpu.resetFIFO();
           fifoCount = mpu.getFIFOCount();
           //fifoCount -= packetSize;
        }

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        yaw = ypr[0] * 180/M_PI;
        pitch = ypr[1] * 180/M_PI;
        roll = ypr[2] * 180/M_PI;

        dyaw = yaw + cyaw;
        dpitch = pitch + cpitch;
        droll = roll + croll;

        //Serial.print(F("."));

      }

    }


  mqttClient.loop();

    // end code here -----------------------------------------------------------

}
