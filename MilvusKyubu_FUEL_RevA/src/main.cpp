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

#define HOSTNAME "Milvus Kyubu FUEL"
#define FIRMWARE "D1MINI_OTA_MQ_FUEL_RevA_v10"


// Variables

#define BT1 D6

int fuelPin = A0;    // select the input pin for the potentiometer
int rawFuelValue = 0;
int rawFuelMin = 1000;
int rawFuelMax = 0;
int fuelPercent = 0;
int tankCapacity = 80;
int fuelWarning = 20;
int backlight = 80;         // not used...

float EMA_a = 0.05;      //initialization of EMA alpha
int rawFuelEMA = 0;          //initialization of EMA S


String disFuelRaw;
String disFuelPercent;
String disFuelLiter;

bool stateB1;

bool b1Active = false;
bool b1longActive = false;

long buttonTimer = 0;
long longButton = 3000;
bool calMode = false;



// WIFI & MQTT

//const char* ssid = "TheMatrix";
//const char* password = "tamghI,.ocl17";

const char* ssid = "milvus-net";
const char* password = "superbrandls";


#define GREEN 5806
#define RED 53890

// EEPROM storage

int eeAdd = 0;

struct dataStore {
  int rawFuelMin;
  int rawFuelMax;
  int tankCapacity;
};

dataStore myDataStore;


WiFiClient netclient;

//SoftwareSerial nextion(DIS_RX, DIS_TX);  // RX, TX
Nextion display(Serial1, 9600);

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

void storeData () {

  myDataStore = {rawFuelMin, rawFuelMax, tankCapacity};
  EEPROM.begin(512);
  EEPROM.put(eeAdd, myDataStore);
  EEPROM.end();

}

void checkModeButton () {

    stateB1 = digitalRead(BT1);

    if (stateB1 == LOW) {

		if (b1Active == false) {
			b1Active = true;
			buttonTimer = millis();
		}

		if ((millis() - buttonTimer > longButton) && (b1longActive == false)) {
			b1longActive = true;
            if (calMode == true){
                calMode = false;
                storeData ();
            } else {
                calMode = true;
                rawFuelMin = 1000;
                rawFuelMax = 0;
                tankCapacity = 80;
            }
		}
	} else {

		if (b1Active == true) {
			if (b1longActive == true) {
				b1longActive = false;
			} else {

            //short press action
                if (calMode == true){
                    if (tankCapacity <= 130){
                        tankCapacity++;
                    }
                    else {
                        tankCapacity = 80;;
                    }
                }

                // short press section
			}
			b1Active = false;
		}
	}
};



void setup() {

  Serial1.begin(9600);                   // Display
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

  pinMode(BT1, INPUT_PULLUP);

  EEPROM.begin(512);
  EEPROM.get(eeAdd, myDataStore );
  EEPROM.end();

  rawFuelMin = myDataStore.rawFuelMin;
  rawFuelMax = myDataStore.rawFuelMax;
  tankCapacity = myDataStore.tankCapacity;


  display.init();
  //display.sendCommand("cal");
  display.sendCommand("dim=80");
  //display.sendCommand("bkcmd=0");




  //display.sendCommand("bauds=115200");

  rawFuelEMA = analogRead(fuelPin);  //set EMA S for t=1

}

void loop() {

    ArduinoOTA.handle();

    // start code here -----------------------------------------------------------

    rawFuelValue = analogRead(fuelPin);
    rawFuelEMA = (EMA_a*rawFuelValue) + ((1-EMA_a)*rawFuelEMA);

    checkModeButton();

    if (calMode){

        display.setComponentText("d0", disFuelRaw);
        display.setComponentText("t0", "Calibration");
        display.sendCommand("t0.bco=53890");

        if (rawFuelEMA >= rawFuelMax){
            rawFuelMax = rawFuelEMA;
        }
        if (rawFuelEMA <= rawFuelMin){
            rawFuelMin = rawFuelEMA;
        }

    }

    else {

        if (fuelPercent >= 0 && fuelPercent <= fuelWarning){
            display.setComponentText("d0", "Low Fuel");
            display.sendCommand("d0.pco=53890");

        }
        else{
        display.setComponentText("d0", "");
            display.sendCommand("d0.pco=65535");
        }

        display.setComponentText("t0", "Aux Fuel Tank");
        display.sendCommand("t0.bco=0");


    }


    fuelPercent = map(rawFuelEMA, rawFuelMin, rawFuelMax, 0, 100);

    if (fuelPercent < 0){
        fuelPercent = 0;
    }
    if (fuelPercent >  100){
        fuelPercent = 100;
    }


    disFuelRaw = String(rawFuelEMA, DEC);
    disFuelPercent = String(fuelPercent, DEC);
    disFuelLiter = String(fuelPercent*tankCapacity/100, DEC);

    if (fuelPercent >= 0 && fuelPercent <= fuelWarning){
        //display.setComponentText("d0", "Low Fuel");
        //display.sendCommand("d0.pco=53890");
        display.sendCommand("b1.bco=53890");
        display.sendCommand("b1.pco=53890");
    }
    else{
        display.sendCommand("b1.bco=48631");
        display.sendCommand("b1.pco=1024");
    }

    display.setComponentValue("b1", fuelPercent);
    display.setComponentText("d1", disFuelPercent);
    display.setComponentText("d2", disFuelLiter);



  // end code here -----------------------------------------------------------

}
