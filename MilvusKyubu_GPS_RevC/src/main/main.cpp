#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
//#include <WiFiManager.h>

#define HOSTNAME "Milvus Kyubu GPS"
#define FIRMWARE "FEATHER_OTA_MQ_GPS_RevC_v16"

//#define DEBUG

const char* ssid = "milvus-net";
const char* password = "superbrandls";


unsigned long lastPub;
unsigned long pubInt = 1000;
char payload[8];
char jsonarray[128];

#define GPS_RX 12 // GPS TX,
#define GPS_TX 14 // GPS RX,

#define SERVER            "192.168.1.50"
#define SERVERPORT        1883
#define MQTT_USERNAME     "mqttmaster"
#define MQTT_PW           "mqttmaster"
#define USERNAME          "mqttmaster"
#define TOPIC             "gps/"
#define T_STATE           "state"
#define T_COMMAND         "command"
#define T_FIRMWARE        "fw"
#define T_PUBINT          "pubint"
#define T_LOCALIP         "ip"
#define T_LAT             "lat"
#define T_LNG             "lng"
#define T_LNG             "lng"
#define T_SPEED           "speed"
#define T_DEG             "deg"
#define T_DATE            "date"
#define T_TIME            "time"
#define T_SAT             "sat"
#define T_ALT             "alt"
#define T_DISW1           "disW1"
#define T_DISW2           "disW2"
#define T_CW1             "cW1"
#define T_CW2             "cW2"
#define T_OSM             "osm"
#define T_BAT             "bat"


// GPS variables

int gpsChars = 0;
int gpsCharsNew = 0;
static const uint32_t GPSBaud = 9600;
int batLevel =0;

char lng[12];
char lat[12];
char alt[6];
char deg[6];
char speed[6];
char time_sat[10];
char date[12];
char datetime[24];
char disW1[6];
char disW2[6];
char hdop[6];

double courseToW1;
double courseToW2;

double prevLng;
double prevLat;

double w1lng;
double w1lat;
double w2lng;
double w2lat;
double disToW1 = 0;
int w1State = 0;
double disToW2 = 0;
int w2State = 0;


WiFiClient netclient;
PubSubClient mqttClient(netclient);
TinyGPSPlus gps;
SoftwareSerial ssGPS(GPS_TX, GPS_RX); // Create a SoftwareSerial



void battery_level() {

  // read the battery level from the ESP8266 analog in pin.
  // analog read level is 10 bit 0-1023 (0V-1V).
  // our 1M & 220K voltage divider takes the max
  // lipo value of 4.2V and drops it to 0.758V max.
  // this means our min analog read value should be 580 (3.14V)
  // and the max analog read value should be 774 (4.2V).
  batLevel = analogRead(A0);
  // convert battery level to percent
  batLevel = map(batLevel, 560, 730, 0, 100);
  //Serial.print("Battery level: "); Serial.print(level); Serial.println("%");

}

void updateWaypoints() {

      if (w1State == 0)  {     // if waypoint ist not set, distance is always 0
            disToW1 = 0;
      }

      if (w2State == 0)  {     // if waypoint ist not set, distance is always 0
            disToW2 = 0;
      }

      if (w1State == 1 && gps.speed.kmph() >= 1)  {     // if waypoint ist set, calc distance continousl
      //if (w1State == 1 && gps.speed.kmph() != 0)  {     // if waypoint ist set, calc distance continously

        disToW1 = disToW1 + TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), prevLat, prevLng);
        courseToW1 = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), w1lat, w1lng);

      }

      if (w2State == 1 && gps.speed.kmph() >= 1)  {     // if waypoint ist set, calc distance continously

        disToW2 = disToW2 + TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), prevLat, prevLng);
        courseToW2 = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), w2lat, w2lng);

      }

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
      mqttClient.subscribe(TOPIC T_PUBINT);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte * data, unsigned int length) {
  // handle message arrived {
  /*
  Serial.println("");
  Serial.print(topic);
  Serial.print(": ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println();
  */

  String strTopic = String((char*)topic);
  String strPayload;

  if (strTopic == "gps/pubint") {

    for (int i = 0; i < length; i++) {
      strPayload += (char)data[i];
    }
    pubInt = strPayload.toInt();
    //Serial.println(pubInt);

  }

  if (strTopic == "gps/command") {

    if (data[0] == 'W' && data[1] == '1' && data[2] == '1')  {

      w1lng = gps.location.lng();
      w1lat = gps.location.lat();
      w1State = 1 ; // waypoint ist set now...
      mqttClient.publish(TOPIC T_STATE, "W1 set");

    }

    if (data[0] == 'W' && data[1] == '1' && data[2] == '0')  {

      w1State = 0 ; // waypoint ist reset now and follows actual position...
      mqttClient.publish(TOPIC T_STATE, "W1 reset");
    }

    if (data[0] == 'W' && data[1] == '2' && data[2] == '1')  {

      w2lng = gps.location.lng();
      w2lat = gps.location.lat();
      w2State = 1 ; // waypoint ist set now...
      mqttClient.publish(TOPIC T_STATE, "W2 set");

    }

    if (data[0] == 'W' && data[1] == '2' && data[2] == '0')  {

      w2State = 0 ; // waypoint ist reset now and follows actual position...
      mqttClient.publish(TOPIC T_STATE, "W2 reset");
    }



  }
}

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do
    {
      while (ssGPS.available())
        gps.encode(ssGPS.read());
    } while (millis() - start < ms);
}

void setup() {

  ssGPS.begin(GPSBaud);                       // GPS
  Serial.begin(115200);                    // Console Debug

  //pinMode(1, FUNCTION_3);
  //pinMode(1, OUTPUT);
  //pinMode(0, INPUT);

  Serial.println("Booting");

  //WiFiManager wifiManager;
  //wifiManager.resetSettings();
  //wifiManager.autoConnect(HOSTNAME);

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
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Firmware: ");
  Serial.println(FIRMWARE);

  mqttClient.setServer(SERVER, SERVERPORT);
  mqttClient.setCallback(callback);

}

void loop() {
  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    reconnect();
  }

  // start code here -----------------------------------------------------------

  smartDelay(100);
  gpsCharsNew = gps.charsProcessed() - gpsChars;

  if (gpsCharsNew < 10){
      Serial.println("no new GPS data received...");
      gpsChars = gps.charsProcessed();
  }

  if (gps.location.age() < 5000){    // we have a fix and new data


      if (mqttClient.connected() && millis() - lastPub > pubInt) {

        battery_level();
        updateWaypoints();
        prevLng = gps.location.lng();
        prevLat = gps.location.lat();

        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();

        sprintf(time_sat, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        sprintf(date, "%02d.%02d.%04d", gps.date.day(), gps.date.month(), gps.date.year());
        sprintf(datetime, "%04d-%02d-%02dT%02d:%02d:%02dZ", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

        dtostrf(gps.location.lng(), 1, 6, lng);
        dtostrf(gps.location.lat(), 1, 6, lat);
        dtostrf(gps.altitude.meters(), 1, 0, alt);
        dtostrf(gps.course.deg(), 1, 0, deg);
        dtostrf(gps.speed.kmph(), 1, 0, speed);
        dtostrf(disToW1, 1, 0, disW1);
        dtostrf(disToW2, 1, 0, disW2);
        dtostrf(gps.hdop.value(), 1, 0, hdop);

        sprintf (payload, "%i", gps.satellites.value());

        mqttClient.publish(TOPIC T_SAT, payload);
        mqttClient.publish(TOPIC T_LAT, lat);
        mqttClient.publish(TOPIC T_LNG, lng);
        mqttClient.publish(TOPIC T_DEG, deg);
        mqttClient.publish(TOPIC T_SPEED, speed);
        mqttClient.publish(TOPIC T_ALT, alt);
        mqttClient.publish(TOPIC T_TIME, time_sat);
        mqttClient.publish(TOPIC T_DATE, date);
        mqttClient.publish(TOPIC T_DISW1, disW1);
        mqttClient.publish(TOPIC T_DISW2, disW2);
        mqttClient.publish(TOPIC T_CW1, TinyGPSPlus::cardinal(courseToW1));
        mqttClient.publish(TOPIC T_CW2, TinyGPSPlus::cardinal(courseToW2));

        sprintf (payload, "%i", batLevel);
        mqttClient.publish(TOPIC T_BAT, payload);

        root["lat"] = lat;
        root["lon"] = lng;
        root["timestamp"] = datetime;
        root["hdop"] = hdop;
        root["altitude"] = alt;
        root["speed"] = speed;

        root.printTo(jsonarray);
        mqttClient.publish(TOPIC T_OSM, jsonarray);

        lastPub = millis();

        // print values to console for debugging
        #ifdef DEBUG

            Serial.print("Lat:  ");
            Serial.print(gps.location.lat(), 6);
            Serial.print("    Lon:  ");
            Serial.println(gps.location.lng(), 6);
            Serial.print("Date: ");
            Serial.print(gps.date.month(), DEC);
            Serial.print("/");
            Serial.print(gps.date.day(), DEC);
            Serial.print("/");
            Serial.print(gps.date.year(), DEC);
            Serial.print("     Time: ");
            Serial.print(gps.time.hour(), DEC);
            Serial.print(":");
            Serial.print(gps.time.minute(), DEC);
            Serial.print(":");
            Serial.println(gps.time.second(), DEC);

            Serial.print("A-G:  ");
            Serial.print(gps.altitude.meters());
            Serial.print("       A-B:  ");

            Serial.print("Sat/Prec: ");
            Serial.print(gps.satellites.value());
            Serial.print("/");
            Serial.print(gps.hdop.value());
            Serial.print("   Temp: ");

            Serial.print("Speed:  ");
            Serial.print(gps.speed.kmph());
            Serial.print("      Course:  ");
            Serial.println(gps.course.deg());

            Serial.println("----------------------------------");

        #endif

      }

  }

  mqttClient.loop();

}
