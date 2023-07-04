


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Preferences.h>
#include "ThingSpeak.h"
#include <WiFi.h>


#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino


//BluetoothSerial ESP_BT; //Object for Bluetooth

WiFiClient  client;


#define VERSION 1   //INT Type Only

//ESP32 PIN
#define LED_PIN 22
#define LED_ERROR_PIN 23
#define LED_BLE_PIN 19
#define MIST1_PIN 12
#define MIST2_PIN 13
#define ADC_PIN 36
#define PUMP_PIN 17
#define BLE_PIN 32
#define ENABLE_SENSORS_PIN 5
#define ONE_WIRE_TEMP 27
#define DHT_SENSOR_PIN  18
#define pH_PIN 35
#define TANK_HIGH_PIN 4
#define TANK_LOW_PIN 2
#define CP_PIN 33
#define CN_PIN 25
#define EC_PIN 26

//CONSTANT STATUT MISTER
#define MIST_INIT 0
#define MIST_OK 1
#define MIST_ERROR 2

#define MIST_MIN_FREQ 90000
#define MIST_MAX_FREQ 130000

#define ADC_BUFFER_SIZE 100
#define OFFSET_ADC 2

#define LED_CHANNEL_MIST_1 0
#define LED_CHANNEL_MIST_2 1
#define LED_CHANNEL_MIST_TEST 3
#define LED_CHANNEL_RESOLUTION 1
#define LED_CHANNEL_DUTYCYCLE 1

#define LED_CHANNEL_BLINK 4

// setting PWM properties





//TEMPERATURE SENSORS INIT
#define TEMPERATURE_PRECISION 10
OneWire oneWire(ONE_WIRE_TEMP);
DallasTemperature tempSensors(&oneWire);

#define DHT_SENSOR_TYPE DHT22
DHT dhtSensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);


uint8_t tankThermometer[8];
uint8_t rootThermometer[8];

//Temp constants between root and tank temp sensors
#define ROOT_CODE 0
#define TANK_CODE 1



//Threshold for capacitor pin of esp32 one for start ble & one for water sensor
#define VALUE_THRESHOLD 50
#define VALUE_THRESHOLD_WATER_LEVEL 50
#define TITLE_PREF "Pref"



//BLE
#define SERVICE_UUID           "367fa97d-b107-49e0-9503-3409d2c247c9"
#define CHARACTERISTIC_UUID_RX "367fa97d-b108-49e0-9503-3409d2c247c9"
#define CHARACTERISTIC_UUID_TX "367fa97d-b109-49e0-9503-3409d2c247c9"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool bleActivated = false;
bool isBLEInitialize = false;



// TIMER CONST
#define TIME_RESTART_ESP   86400000 // in milliseconds
#define TIME_CHECK_WATER_LEVEL   120000 // in milliseconds
#define WIFI_TIMEOUT 30000 // in milliseconds
const int TIME_ON_PUMP =  20000; // in milliseconds
const int TIME_OFF_PUMP = 1800000; // in milliseconds



#define TIME_CHECK_BUTTON_BLE 1000 // in milliseconds
#define BLE_ACTIVE_TIMEOUT 120000 // in milliseconds



//MISTER VARIABLES
int statutMist1 = MIST_INIT;
int statutMist2 = MIST_INIT;
int freqMist1 = 0;
int freqMist2 = 0;


boolean enableMist = 0;  // Default OFF
int frequencyMisterDefault = 108000; // Default 108 kHz
int frequencyMister1 = 108000; // Default 108 kHz                                
int frequencyMister2 = 108000; // Default 108 kHz
int timeOnMister = 600000; // Default 10 min
int timeOffMister = 600000; // Default 10 min

int errorPump = 0;


// pH VARIABLES
boolean enablepH = 0; // Default OFF
int pHCalib4 = -1; // Default
int pHCalib9 = -1; // Default
float temperatureInCelsiusCompensation = 0;
#define pH_BUFFER_SIZE 500
#define PH4_DEFAULT 500
#define PH9_DEFAULT 1000


// EC VARIABLES
boolean enableEC = 0; // Default OFF
String ECCalib = ""; // Default Empty
#define EC_BUFFER_SIZE 1000
#define EC_DEFAULT "15517/6650/0.93|-3E-05|/:6650/3770/2.67|-0.0003|/:"


// Temperature VARIABLES
boolean isCelsius = 1; // Default Celsius
boolean enableTankTemp = 0; // Default OFF
boolean enableRootTemp = 0; // Default OFF
boolean enableAmbiantTemp = 0; // Default OFF
float offSetTankTemp = 0;
float offSetRootTemp = 0;
float offSetAmbiantTemp = 0;
int timeCheckSensors = 600000; // Default 10 min
#define TIME_CHECK_SENSORS_BLE 3000 // 3s in milliseconds




boolean isECpHTempCompensation  = false; // Default FALSE

//ThingSpeak VARIABLES
String ssid = "" ;
String pw = "";
long myChannelNumber = 0;
String myWriteAPIKey = "";
bool enableThingSpeak = false;  // Default false
int intervalTimeThingSpeak = 15;



//UltraTower VARIABLES
String nameDevice = "Ultra";




//TIMER variable
unsigned long startTimeBle = 0;
unsigned long startTimeMist = 0;
unsigned long startTimePump = 0;
unsigned long startTimeCheckWaterLevel = 0;
unsigned long startTimeSensors = 0;
unsigned long startTime = 0;
int intervalTimeMister = 4000;






boolean isMist = 1;
boolean isPump = 1;
int sequenceMist = -1;
int cycleMist = 0;

const int capacity = JSON_OBJECT_SIZE(40);
StaticJsonDocument<capacity> jsonBuffer;





void setup() {


  Serial.begin(115200);

  delay(100); // give me time to bring up serial monitor

   pinMode(BLE_PIN, INPUT);
   pinMode(TANK_HIGH_PIN, INPUT);
   pinMode(TANK_LOW_PIN, INPUT);


  //INIT PIN
  pinMode(MIST1_PIN, INPUT);
  pinMode(MIST2_PIN, INPUT);

  digitalWrite(PUMP_PIN, LOW);
  pinMode(PUMP_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);


  digitalWrite(LED_ERROR_PIN, HIGH);
  pinMode(LED_ERROR_PIN, OUTPUT);

  digitalWrite(LED_BLE_PIN, HIGH);
  pinMode(LED_BLE_PIN, OUTPUT);

  digitalWrite(ENABLE_SENSORS_PIN, LOW);
  pinMode(ENABLE_SENSORS_PIN, OUTPUT);

  /*Preferences preferences;
    preferences.begin(TITLE_PREF, false);
    preferences.clear();
    preferences.end();*/


  getPreferences();

  startTimeMist = millis() - timeOffMister;
  startTimePump = millis();
  startTimeSensors = millis();
  startTime = millis();

  timeCheckSensors = intervalTimeThingSpeak * 60000;



  isPump = false;
  isMist = false;

  Serial.println("BOOT PROCEDURE OK");

  delay(500);
  
}

void loop() {


  //BLE Button Activation
  if (touchRead(BLE_PIN) < VALUE_THRESHOLD && !bleActivated) {
    checkIfStartBle();
  }
  pinMode(BLE_PIN, INPUT);



  if (bleActivated) {
    if (deviceConnected) {

    } else {
      if (millis() - startTimeBle > BLE_ACTIVE_TIMEOUT) {
        stopLED(LED_BLE_PIN);
        pServer->getAdvertising()->stop();
        bleActivated = false;
        timeCheckSensors = intervalTimeThingSpeak * 60000;
        Serial.println("BLE OFF");
      }
    }
  }






  //MISTER ROUTINE
  if (enableMist) {
    if (isMist) {
      if (millis() - startTimeMist > timeOnMister) {
        setMisterOff();
        startTimeMist = millis();
        isMist = false;
        if (deviceConnected) {
          sendStatutNotif("MOFF");
        }
      }

      
      if (millis() - startTimeCheckWaterLevel > TIME_CHECK_WATER_LEVEL) {
        if (checkIfPinIsInWater(TANK_LOW_PIN)) {
          startTimeCheckWaterLevel = millis();
        } else {
          startPump();
        }

      }
    } else {

      if (millis() - startTimeMist > timeOffMister) {

        if (checkIfPinIsInWater(TANK_LOW_PIN)) {
          setMisterOn();
          isMist = true;
          startTimeMist = millis();

          if (deviceConnected) {
            sendStatutNotif("MON");
          }

        } else {
          startPump();
        }


      }
    }
  }





  //SENSORS ROUTINE
  if (deviceConnected || enableThingSpeak) {
    if (millis() - startTimeSensors > timeCheckSensors) {
      checkSensors();
    }
  }




  if (millis() - startTime > TIME_RESTART_ESP ) {
    Serial.println("RESTART EVERY 24 HOURS");
    ESP.restart();
  }


}


//**********************************************************************************************************************************************************
//*******************************************************       MISTERS FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************




void setMisterOn() {

  Serial.println("MISTER ON");

  if (statutMist1 == MIST_INIT ) {
    initFreqMister();

  }

  digitalWrite(MIST1_PIN, LOW);
  pinMode(MIST1_PIN, OUTPUT);
  digitalWrite(MIST2_PIN, LOW);
  pinMode(MIST2_PIN, OUTPUT);

  if (statutMist1 == MIST_OK ) {
    ledcAttachPin(MIST1_PIN, LED_CHANNEL_MIST_1);
    ledcWrite(LED_CHANNEL_MIST_1, LED_CHANNEL_DUTYCYCLE);
  }

  if (statutMist2 == MIST_OK ) {
    ledcAttachPin(MIST2_PIN, LED_CHANNEL_MIST_2);
    ledcWrite(LED_CHANNEL_MIST_2, LED_CHANNEL_DUTYCYCLE);
  }

}

void setMisterOff() {

  ledcDetachPin(MIST1_PIN);
  pinMode(MIST1_PIN, INPUT);
  ledcDetachPin(MIST2_PIN);
  pinMode(MIST2_PIN, INPUT);

}


void searchForBestFrequency (int mistPin) {

  int32_t buf[ADC_BUFFER_SIZE];
  int minFreq = frequencyMisterDefault - 3000;
  int maxFreq = frequencyMisterDefault + 3000;
  int freqMaxADC = 0;
  int minADC = 0;
  int maxADC = 0;
  int32_t sample = 0;

 // analogSetPinAttenuation(ADC_PIN, ADC_0db);


  for (int i = minFreq; i < maxFreq ; i += 250)
  {

    ledcSetup(LED_CHANNEL_MIST_TEST, i , LED_CHANNEL_RESOLUTION);
    ledcAttachPin(mistPin, LED_CHANNEL_MIST_TEST);
    ledcWrite(LED_CHANNEL_MIST_TEST, LED_CHANNEL_DUTYCYCLE);

    Serial.println("************************************");
    Serial.print("Freq : ");
    Serial.println(i);

    delay(500);



    for (int j = 0; j < ADC_BUFFER_SIZE; j++)
    {
      buf[j] = analogRead(ADC_PIN);
      delay(4);
    }

    sample = getAverage(buf , ADC_BUFFER_SIZE);


    Serial.print("ADC : ");
    Serial.println(sample);

    if (i == minFreq)
    {
      minADC = sample;
      maxADC = sample;
      freqMaxADC = minFreq;
    } else
    {
      if (sample + OFFSET_ADC < minADC) {
        minADC = sample;
      }

      if (sample - OFFSET_ADC > maxADC ) {
        maxADC = sample;
        freqMaxADC = i;

      }
    }
  }



  Serial.println("************************************");
  Serial.print("MAX Freq = ");
  Serial.println(freqMaxADC);
  Serial.print("MIN ADC = ");
  Serial.println(minADC);
  Serial.print("MAX ADC = ");
  Serial.println(maxADC);

  if (maxADC <= 80 && maxADC - minADC <= 70) {

    if (maxADC <= 80) {
      Serial.println("ERROR INIT MIST : DISK NOT DETECTED");
    } else {
      Serial.println("INIT MIST : PEAK DETECTED");
    }

    blinkLED(LED_ERROR_PIN);
    if (mistPin == MIST1_PIN) {
      statutMist1 = MIST_ERROR;
    } else {
      statutMist2 = MIST_ERROR;
    }

  } else {
    Serial.println("INIT MIST : PEAK DETECTED");

    if (mistPin == MIST1_PIN) {
      freqMist1 = freqMaxADC;
      statutMist1 = MIST_OK;
      ledcSetup(LED_CHANNEL_MIST_1, freqMist1 , LED_CHANNEL_RESOLUTION);
    } else {
      freqMist2 = freqMaxADC;
      statutMist2 = MIST_OK;
      ledcSetup(LED_CHANNEL_MIST_2, freqMist2 , LED_CHANNEL_RESOLUTION);
    }
  }

}



void initFreqMister() {


  blinkLED(LED_PIN);


  ledcSetup(LED_CHANNEL_MIST_TEST, frequencyMisterDefault , LED_CHANNEL_RESOLUTION);
  ledcAttachPin(MIST1_PIN, LED_CHANNEL_MIST_TEST);
  ledcAttachPin(MIST2_PIN, LED_CHANNEL_MIST_TEST);
  ledcWrite(LED_CHANNEL_MIST_TEST, LED_CHANNEL_DUTYCYCLE);

  delay(30000);


  setMisterOff();

  searchForBestFrequency(MIST1_PIN);

  setMisterOff();

  searchForBestFrequency(MIST2_PIN);
  setMisterOff();


  stopLED(LED_PIN);

}



//**********************************************************************************************************************************************************
//*******************************************************     LED FUNCTION        *******************************************************************
//**********************************************************************************************************************************************************


void blinkLED(int pinLED) {

  pinMode(pinLED, OUTPUT);
  ledcSetup(LED_CHANNEL_BLINK, 1 , 16);
  ledcAttachPin(pinLED, LED_CHANNEL_BLINK);
  ledcWrite(LED_CHANNEL_BLINK, 65535 / 2);

}

void stopLED(int pinLED) {

  ledcDetachPin(pinLED);
  digitalWrite(pinLED, HIGH);
  pinMode(pinLED, OUTPUT);

}






//**********************************************************************************************************************************************************
//*******************************************************      PUMP FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************

bool checkIfPinIsInWater(int pin) {
 
  int i = 0;

  Serial.println("");
  Serial.println("**********************************************");
  Serial.println("CHECK PIN IN WATER");

  setMisterOff();
  delay(1000);

   for (int16_t j= 0; j< 100; j++){
     Serial.print(touchRead(pin));
     Serial.print("/");

   }
    Serial.println("");



 unsigned long startTime = millis();
  while (1) {
    if (millis() - startTime > 1000) {
      Serial.println("WATER LEVEL : Pin is not in water");
      pinMode(pin, INPUT);
      return false;
      break;
    }

    int touch = touchRead(pin);

    Serial.print(touch);
    if (touch < VALUE_THRESHOLD_WATER_LEVEL) {
      i++;
    } else {
      i = 0;
    }


    if (i >= 10) {
      Serial.println("WATER LEVEL : PIN OK");
      if (isMist) {
        setMisterOn();
      }
      pinMode(pin, INPUT);

      return true;
      break;
    }

    delay(5);
  }
}

void startPump() {
 
  

  isPump = true;
  Serial.println("START PUMP");
  pinMode(TANK_HIGH_PIN, INPUT);
  pinMode(TANK_LOW_PIN, INPUT);


  setMisterOff();
  delay(500);


  unsigned long startPumpFunction =  millis();
  digitalWrite(LED_PIN, LOW);
  digitalWrite(PUMP_PIN, HIGH);

  delay(4000);

  int i = 0;
  int touchReadValue = 100;
  startTimePump = millis();

  while (isPump) {
    if (millis() - startTimePump > TIME_ON_PUMP) {
      Serial.println("TIME OUT PUMP : ERROR");
      isPump = false;
      break;
    }

    touchReadValue = touchRead(TANK_HIGH_PIN);
    Serial.print(touchReadValue);
    Serial.print("/");

    if (touchReadValue < VALUE_THRESHOLD_WATER_LEVEL) {
      i++;
    } else {
      i = 0;
    }

    if (i >= 10) {
      errorPump = 0;
      Serial.println("WATER LEVEL : HIGH TANK REACH");
      break;
    }

    delay(5);
  }



  if (!isPump) {
    errorPump++;
    isMist = false;
    if (errorPump >= 2) {
      Serial.println("ERROR PUMP OR WATER SENSORS : THE SYSTEM MUST BE RESTARTED");
      enableMist = false;
      stopLED(LED_ERROR_PIN);
      digitalWrite(LED_ERROR_PIN, LOW);
      pinMode(LED_ERROR_PIN, OUTPUT);

    }
  }

  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
  pinMode(TANK_HIGH_PIN, INPUT);

  unsigned long endPumpFunction =  millis();
  
  delay(2000);

  if (isMist) {
    setMisterOn();
  }

   if (enableThingSpeak) {
     if(initThingSpeak()){
      long timePumpFunction = (endPumpFunction - startPumpFunction)/1000;
      ThingSpeak.setField(6, timePumpFunction);
      sendFieldsToThingSpeak(); 
      
     }else{
      turnOffWiFi();
     }
   }

  startTimePump = millis();
  startTimeCheckWaterLevel  = millis();
}






void checkIfStartBle() {
  unsigned long startTimeButton = millis();
  int i = 0;


  while (1) {
    if (millis() - startTimeButton > TIME_CHECK_BUTTON_BLE) { 
      Serial.println("TIME OUT BLE NOT ACTIVATED");
      break;
    }


    if (touchRead(BLE_PIN) < VALUE_THRESHOLD) {
      i++;
    } else {
      i = 0;
    }

    if (i >= 5) {

      initBLE();


      break;
    }

    delay(10);
  }



}



void sendStatutNotif(String str) {
  if (bleActivated) {
    if (deviceConnected) {
      sendNotifDataOverBLE(str);
    }
  }
}



void sendNotifDataOverBLE(String str) { //20 bytes in your notification data payload

  int sizeStr = str.length() + 1;
  char buf[sizeStr];
  str.toCharArray(buf, sizeStr);

  pTxCharacteristic->setValue((uint8_t*)&buf[0], sizeStr);
  pTxCharacteristic->notify();
  delay(10); //Delay between two packets
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      timeCheckSensors = TIME_CHECK_SENSORS_BLE;
      activateSensors();
      Serial.println("DEVICE CONNECTED");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      startTimeBle = millis();
      Serial.println("DEVICE DISCONNECTED");
      pServer->startAdvertising();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() == 0) {
        return;
      }

      // Decode data
      int keyIndex = 0;
      for (int index = 0; index < value.length(); index ++) {
        value[index] = (char) value[index] ;

      }
      jsonBuffer.clear();

      DeserializationError err = deserializeJson(jsonBuffer, value);

      if (err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.c_str());
        return;
      }

      JsonObject object = jsonBuffer.as<JsonObject>();
      const char* cmd = object["CMD"];
      Serial.println(cmd);

      if (strcmp(cmd, "MFREQ") == 0 ) {
        //frequencyMister = object["DATA"];
        // ledcSetup(ledChannel, frequencyMister, resolution);
        sendNotifDataOverBLE("MFREQ");


      } else if (strcmp(cmd, "MENA") == 0) {
        enableMist = object["DATA"];
        sendNotifDataOverBLE("MENA");
        if (!enableMist) {
          setMisterOff();
        }

      } else if (strcmp(cmd, "MISON") == 0) { //MIST is ON
        String strTosend = "MISON:" +  String(isMist);
        sendNotifDataOverBLE(strTosend);;



      } else if (strcmp(cmd, "MCYCL") == 0) {
        timeOnMister = object["DATA_ON"];
        timeOffMister = object["DATA_OFF"];
        sendNotifDataOverBLE("MCYCL");

      } else if (strcmp(cmd, "TENA") == 0) {
        enableAmbiantTemp = object["DATA_AMB"];
        enableRootTemp = object["DATA_ROOT"];
        enableTankTemp = object["DATA_TANK"];
        sendNotifDataOverBLE("TENA");

      } else if (strcmp(cmd, "TOFFS") == 0) {
        offSetAmbiantTemp = object["DATA_AMB"];
        offSetRootTemp = object["DATA_ROOT"];
        offSetTankTemp = object["DATA_TANK"];
        sendNotifDataOverBLE("TOFFS");

      } else if (strcmp(cmd, "TFUSE") == 0) {
        activateSensors();
        String strToReturn = OneWireScanner(object["DATA"]);
        String strTosend = "TFUSE:" + strToReturn;
        sendNotifDataOverBLE(strTosend);

      } else if (strcmp(cmd, "THING") == 0) {
        enableThingSpeak = object["DATA_ENA"];
        intervalTimeThingSpeak = object["TIME"];
        ssid = String(object["SSID"].as<char*>());
        myWriteAPIKey = String(object["APIKEY"].as<char*>());
        myChannelNumber = object["CHANNEL"];

        String newPW = String(object["PW"].as<char*>());
        if (newPW != "") {
          pw = newPW;
        }

        String strTosend = "THING:" + String(initThingSpeak());
        sendNotifDataOverBLE(strTosend);
        turnOffWiFi();

      } else if (strcmp(cmd, "PHENA") == 0) {
        enablepH = object["DATA"];
        sendNotifDataOverBLE("PHENA");


      } else if (strcmp(cmd, "PHCAL") == 0) {
        activateSensors();
        delay(500);
        getRawPhSensor();
        delay(100);
        String strToReturn = String(getRawPhSensor());
        String strTosend = "PHCAL:" + strToReturn;
        sendNotifDataOverBLE(strTosend);

      } else if (strcmp(cmd, "PHSET") == 0) {
        pHCalib4 = object["DATA4"];
        pHCalib9 = object["DATA9"];
        sendNotifDataOverBLE("PHSET");

      } else if (strcmp(cmd, "ECENA") == 0) {
        enableEC = object["DATA"];
        sendNotifDataOverBLE("ECENA");


      } else if (strcmp(cmd, "ECSET") == 0) {
        ECCalib = String(object["DATA"].as<char*>());

        sendNotifDataOverBLE("ECSET");

      }






      object.clear();
      savePreferences();

    }

    void onRead(BLECharacteristic *pCharacteristic) {
      Serial.println("BLE onRead request");
      String wifiCredentials;




      jsonBuffer["MENA"].set(enableMist);
      jsonBuffer["MFREQ"].set(frequencyMister1);
      jsonBuffer["MON"].set(timeOnMister);
      jsonBuffer["MOFF"].set(timeOffMister);

      jsonBuffer["PHENA"].set(enablepH);
      if (pHCalib4 != -1) {
        jsonBuffer["PHCAL"].set(true);
      } else {
        jsonBuffer["PHCAL"].set(false);
      }




      jsonBuffer["TCELSIUS"].set(isCelsius);
      jsonBuffer["TTANKENA"].set(enableTankTemp);
      jsonBuffer["TTANKOFFSET"].set(offSetTankTemp);
      jsonBuffer["TROOTENA"].set(enableRootTemp);
      jsonBuffer["TROOTOFFSET"].set(offSetRootTemp);
      jsonBuffer["TAMBENA"].set(enableAmbiantTemp);
      jsonBuffer["TAMBOFFSET"].set(offSetAmbiantTemp);

      jsonBuffer["THINGENA"].set(enableThingSpeak);
      jsonBuffer["SSID"].set(ssid);
      jsonBuffer["THINGAPIKEY"].set(myWriteAPIKey);
      jsonBuffer["THINGCHANNEL"].set(myChannelNumber);
      jsonBuffer["THINGTIME"].set(intervalTimeThingSpeak);


      jsonBuffer["ECENA"].set(enableEC);
      if (ECCalib != "") {
        jsonBuffer["ECCAL"].set(true);
      } else {
        jsonBuffer["ECCAL"].set(false);
      }





      jsonBuffer["V"].set(VERSION);





      // Convert JSON object into a string
      serializeJson(jsonBuffer, wifiCredentials);


      // encode the data
      int keyIndex = 0;
      for (int index = 0; index < wifiCredentials.length(); index ++) {
        wifiCredentials[index] = (char) wifiCredentials[index];
      }
      pCharacteristic->setValue((uint8_t*)&wifiCredentials[0], wifiCredentials.length());
      jsonBuffer.clear();

      delay(100);
    }
};


/**
   initBLE
   Initialize BLE service and characteristic
   Start BLE server and service advertising
*/
void initBLE() {
  Serial.println("Start BLE");

  if (!isBLEInitialize) {

    // Initialize BLE and set output power
    BLEDevice::init(nameDevice.c_str());

    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);


    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
                                            );

    pRxCharacteristic->setCallbacks(new MyCallbacks());


    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_TX,
                          BLECharacteristic::PROPERTY_NOTIFY |
                          BLECharacteristic::PROPERTY_INDICATE
                        );

    pTxCharacteristic->addDescriptor(new BLE2902());


    // Start the service
    pService->start();
    isBLEInitialize = true;

  }



  // Start advertising
  pServer->getAdvertising()->start();

  Serial.println("BLE READY");

  startTimeBle = millis();
  //startTimeLED = millis();

  blinkLED(LED_BLE_PIN);

  bleActivated = true;

  delay(2);
}









//**********************************************************************************************************************************************************
//*******************************************************       PREFERENCES FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************

void getPreferences() {
  Preferences preferences;
  preferences.begin(TITLE_PREF, false);

  enableMist =  preferences.getBool("ENAMIST", 0);
  //frequencyMister =  preferences.getInt("FREQMIST", 99000);
  //frequencyMister =  99000;
  timeOnMister =  preferences.getInt("CYCLON", 600000);
  timeOffMister = preferences.getInt("CYCLOFF", 600000);



  enablepH = preferences.getBool("ENAPH", 0);

  pHCalib4 = preferences.getInt("PHCALIB4", -1);
  pHCalib9 = preferences.getInt("PHCALIB9", -1);

  isCelsius = preferences.getBool("ISCELSIUS", 1);
  enableTankTemp = preferences.getBool("ENATANK", 0);
  enableRootTemp = preferences.getBool("ENAROOT", 0);
  enableAmbiantTemp = preferences.getBool("ENAAMB", 0);
  offSetRootTemp = preferences.getFloat("OFFSETROOT", 0.0);
  offSetTankTemp = preferences.getFloat("OFFSETANK", 0.0);
  offSetAmbiantTemp = preferences.getFloat("OFFSEAMB", 0.0);


  preferences.getBytes("ADRESSTANK", tankThermometer, 8);
  preferences.getBytes("ADRESSROOT", rootThermometer, 8);


  enableThingSpeak = preferences.getBool("ENATHING", 0);
  intervalTimeThingSpeak = preferences.getInt("TIMETHING", 15);
  ssid = preferences.getString("SSID", "");
  pw = preferences.getString("PW", "");
  myWriteAPIKey = preferences.getString("APIKEY", "");
  myChannelNumber = preferences.getLong("CHANNEL", 0);

  enableEC = preferences.getBool("ENAEC", 0);
  ECCalib = preferences.getString("ECCALIB", "");


  preferences.end();
}



void savePreferences() {


  Preferences preferences;
  preferences.begin(TITLE_PREF, false);


  preferences.putBool("ENAMIST", enableMist);
  //   preferences.putInt("FREQMIST", frequencyMister);
  preferences.putInt("CYCLON", timeOnMister);
  preferences.putInt("CYCLOFF", timeOffMister);

  preferences.putBool("ENAPH", enablepH);
  preferences.putInt("PHCALIB4", pHCalib4);
  preferences.putInt("PHCALIB9", pHCalib9);

  preferences.putBool("ISCELSIUS", isCelsius);
  preferences.putBool("ENATANK", enableTankTemp);
  preferences.putBool("ENAROOT", enableRootTemp);
  preferences.putBool("ENAAMB", enableAmbiantTemp);
  preferences.putFloat("OFFSETROOT", offSetRootTemp);
  preferences.putFloat("OFFSETANK", offSetTankTemp);
  preferences.putFloat("OFFSEAMB", offSetAmbiantTemp);

  preferences.putBytes("ADRESSTANK", tankThermometer, 8);
  preferences.putBytes("ADRESSROOT", rootThermometer, 8);


  preferences.putBool("ENATHING", enableThingSpeak);
  preferences.putString("SSID", ssid);
  preferences.putString("PW", pw);
  preferences.putLong("CHANNEL", myChannelNumber);
  preferences.putString("APIKEY", myWriteAPIKey);
  preferences.putInt("TIMETHING",  intervalTimeThingSpeak);


  preferences.putBool("ENAEC", enableEC);
  preferences.putString("ECCALIB", ECCalib);










  preferences.end();


}

//**********************************************************************************************************************************************************
//*******************************************************       SENSORS FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************



void activateSensors() {
  digitalWrite(ENABLE_SENSORS_PIN, HIGH); //3.3V for temperature sensors and pH sensor
}

void switchOffSensors() {
  digitalWrite(ENABLE_SENSORS_PIN, LOW); //0V for temperature sensors and pH sensor                                   
}


bool sendToThingSpeak = false;
void checkSensors() {
  Serial.println("");
  Serial.println("********************************************");
  Serial.println("ROUTINE SENSORS");
  activateSensors();
  delay(200);

  temperatureInCelsiusCompensation = -200.0;

  sendToThingSpeak = false;

  if (enableThingSpeak && !bleActivated) {
    sendToThingSpeak = initThingSpeak();

    if (!sendToThingSpeak) {
      turnOffWiFi();
    }
  }


  //Request Tank and Root temperature if enable
  if (enableTankTemp || enableRootTemp) {
    requestRootTankTemp();
  }

  //Request ambiant temperature if enable
  if (enableAmbiantTemp) {
    requestDHT22Temp();
  }

  //Request pH if enable
  if (enablepH) {
    requestpH();
  }

  //Request EC if enable
  if (enableEC) {
    requestEC();
  }





  if (sendToThingSpeak) {
    sendFieldsToThingSpeak(); //Send to thingspeak if enable
  }



  switchOffSensors();
  startTimeSensors = millis();
}


int32_t getAverage(int32_t buf[] , int sizeBuf) {
  quickSort(buf, sizeBuf);

  int32_t avg = 0;
  int16_t nbToStart = sizeBuf / 4;
  int16_t nbToEnd = sizeBuf - nbToStart;
  for (int16_t i = nbToStart; i < nbToEnd; i++) {
    avg += buf[i];
  }

  return avg /  (sizeBuf - 2 * nbToStart);

}

void quickSort(int32_t *ar, int32_t n) {
  if (n < 2)
    return;
  int32_t p = ar[n / 2];
  int32_t *l = ar;
  int32_t *r = ar + n - 1;
  while (l <= r) {
    if (*l < p) {
      l++;
    }
    else if (*r > p) {
      r--;
    }
    else {
      int t = *l;
      *l = *r;
      *r = t;
      l++;
      r--;
    }
  }
  quickSort(ar, r - ar + 1);
  quickSort(l, ar + n - l);
}



//**********************************************************************************************************************************************************
//*******************************************************  TEMPERATURE SENSORS FUNCTIONS *******************************************************************
//**********************************************************************************************************************************************************

void requestRootTankTemp() {
  tempSensors.begin();
  tempSensors.requestTemperatures();
  float temp = -200;

  if (enableTankTemp) {
    if (tempSensors.isConnected(tankThermometer)) {

      temp = tempSensors.getTempC(tankThermometer);
      temp = temp + offSetTankTemp;
      temperatureInCelsiusCompensation = temp;
      Serial.print("Tank Temperature: ");
      Serial.println(temp);

      if (deviceConnected) {
        String strTosend = "TTANK:" ;
        strTosend.concat(temp);
        sendNotifDataOverBLE(strTosend);
      }

      if (sendToThingSpeak) {
        ThingSpeak.setField(1, temp);
      }
    } else {
      Serial.println("TANK Temperature not connected ");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (tankThermometer[i] < 0x10) Serial.print("0");
        Serial.print(tankThermometer[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println("}");
    }
  }

  temp = -200;

  if (enableRootTemp) {
    if (tempSensors.isConnected(rootThermometer)) {

      temp = tempSensors.getTempC(rootThermometer);
      temp = temp + offSetRootTemp;
      Serial.print("ROOT Temperature: ");
      Serial.println(temp);

      if (deviceConnected) {
        String strTosend = "TROOT:" ;
        strTosend.concat(temp);
        sendNotifDataOverBLE(strTosend);
      }

      if (sendToThingSpeak) {
        ThingSpeak.setField(3, temp);
      }
    }
  }
}

void requestDHT22Temp() {
  dhtSensor.begin();
  delay(2000);
  float temp = dhtSensor.readTemperature(false, true);
  temp = temp + offSetAmbiantTemp;

  Serial.print("Ambiant Temperature: ");
  Serial.println(temp);

  if (deviceConnected) {
    String strTosend = "TAMBI:" ;
    strTosend.concat(temp);
    sendNotifDataOverBLE(strTosend);
  }

  if (sendToThingSpeak) {
    ThingSpeak.setField(2, temp);
  }

}










//For the first use the tower must scan the address of the sensors.
String OneWireScanner(int code) {
  Serial.println("Temperature sensor scanner");
  uint8_t address[8];


  // tempSensors.requestTemperatures();
  tempSensors.begin();

  int count = tempSensors.getDeviceCount();
  Serial.print("Number of device : ");
  Serial.println(count);

  if (count == 0) {
    tempSensors.setOneWire(&oneWire);
    tempSensors.begin();
    delay(100);
    count = tempSensors.getDeviceCount();
  }

  if (count == 0) { // No sensors connected
    return "0";
  }

  if (count == 1) { // No sensors connected
    if (tempSensors.getAddress(address , 0)) {
      if (code == TANK_CODE) {
        Serial.print("ADRESS OF TANK TEMP :");
      } else {
        Serial.print("ADRESS OF ROOT TEMP :");
      }
      Serial.print("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println("}");



      if (code == TANK_CODE) {
        memcpy(tankThermometer, address, sizeof(address));


        //If the address is identical to the other thermometer then the other one is deleted

        if (memcmp(rootThermometer, tankThermometer, sizeof(tankThermometer)) == 0) {
          memset (rootThermometer, 0, sizeof(rootThermometer));
          return "4";
        }




      } else {
        memcpy(rootThermometer, address, sizeof(address));

        //If the address is identical to the other thermometer then the other one is deleted

        if (memcmp(tankThermometer, rootThermometer, sizeof(rootThermometer)) == 0) {
          memset (tankThermometer, 0, sizeof(tankThermometer));
          // memcpy(tankThermometer, 0, sizeof(address));
          return "4";
        }


      }

      return "1";
    } else {
      return "3";
    }


  }

  if (count > 1) { // too many sensors connected
    Serial.println("Too many sensors connected");
    return "2";
  }

  Serial.println("Stop One wire scanner");

  return "3";
}


//**********************************************************************************************************************************************************
//*******************************************************         pH FUNCTIONS           *******************************************************************
//**********************************************************************************************************************************************************


void requestpH() {
  float value = 0;

  int calibpH4 = pHCalib4;
  int calibpH9 = pHCalib9;

  if (calibpH4 == -1) {
    calibpH4 = PH4_DEFAULT;
  }

  if (calibpH9 == -1) {
    calibpH9 = PH9_DEFAULT;
  }


  int32_t valraw = getRawPhSensor();

  value = map(valraw, calibpH4, calibpH9, 4.00, 9.18);

  Serial.print("pH Value: ");
  Serial.println(value);

  if (enableTankTemp && temperatureInCelsiusCompensation != -200.0) {
    float errorpH = getErrorpHTemperature(value);
    value = value + errorpH;
    Serial.print("pH error in relation to temperature: ");
    Serial.println(errorpH);
    Serial.print("pH compensated by temperature: ");
    Serial.println(value);
  }



  if (deviceConnected) {
    String strTosend = "PHVAL:" ;
    strTosend.concat(value);
    sendNotifDataOverBLE(strTosend);
  }

  if (sendToThingSpeak) {
    ThingSpeak.setField(4, value);
  }
}





int32_t getRawPhSensor() {
  int32_t valueSensor = 0;
  int32_t buf[pH_BUFFER_SIZE];

  analogSetPinAttenuation(pH_PIN, ADC_2_5db);


  for (int i = 0; i < pH_BUFFER_SIZE; i++) //Get 10 sample value from the sensor for smooth the value
  {
    buf[i] = analogRead(pH_PIN);


  }

  return getAverage(buf , pH_BUFFER_SIZE);
}


float getErrorpHTemperature(float pH) {
  return (pH - 7) * ((temperatureInCelsiusCompensation - 25) / 10) * 0.03;
}








//**********************************************************************************************************************************************************
//*******************************************************      THINGSPEAK FUNCTIONS      *******************************************************************
//**********************************************************************************************************************************************************


bool initThingSpeak() {
  Serial.println("Init WIFI and ThingSpeak");


  // CONNECTION AU WIFI
  unsigned long startAttemptTime = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pw.c_str());


  while (WiFi.status() != WL_CONNECTED) { //Changement de && par || TODO peut être remplacé par un if
    Serial.print("*");
    if (millis() - startAttemptTime > WIFI_TIMEOUT) {
      break;
    }
    delay(50);
  }
  Serial.println("");


  // Make sure that we're actually connected, otherwise return false
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  return ThingSpeak.begin(client);  // Initialize ThingSpeak
}


void sendFieldsToThingSpeak() {
  Serial.println("");
  Serial.println("**************************************************");
  Serial.println("Send to ThingSpeak");


  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey.c_str());
  if (x == 200) {
    Serial.println("Channel update successful.");
  }
  else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  turnOffWiFi();

}

void turnOffWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

//**********************************************************************************************************************************************************
//*******************************************************         EC FUNCTIONS           *******************************************************************
//**********************************************************************************************************************************************************


void requestEC() {

  String str = "";

  if (ECCalib == "") {
    str = EC_DEFAULT;
  } else {
    str = ECCalib;
  }


  uint32_t valueSensor = getRawECSensor();
  float value = 0.0;

  String arrayMain[6]; // MAX 6 values of calibrations
  String arraySecond[3]; // MAX 3 values of calibrations
  String arrayEquation[2]; // MAX 2 values of equation


  int sizeArrayMain = split(arrayMain, str , ':');

  for (int i = 0 ; i <= sizeArrayMain ; i++) {

    int sizeArraySecond = split(arraySecond, arrayMain[i] , '/');

    if (arraySecond[0].toInt() >= valueSensor && valueSensor > arraySecond[1].toInt()) {

      split(arrayEquation, arraySecond[2] , '|');

      Serial.println(arrayEquation[0]);
      Serial.println(arrayEquation[1]);

      value = arrayEquation[0].toFloat() + (arrayEquation[1].toFloat() * valueSensor);
      break;
    }

  }

  Serial.print("EC Value: ");
  Serial.println(value);

  if (deviceConnected) {
    String strTosend = "ECVAL:" ;
    strTosend.concat(value);
    strTosend.concat(":");
    strTosend.concat(valueSensor);
    sendNotifDataOverBLE(strTosend);
  }

  if (sendToThingSpeak) {
    ThingSpeak.setField(5, value);
  }


}





volatile uint32_t endCycle = 0;
void IRAM_ATTR isrCountEndCycle() {
  endCycle = ESP.getCycleCount();
}



/*hw_timer_t *timerEC = NULL;
  void IRAM_ATTR onTimer(){
     endCycle = -1;
  }*/


// Time constants in MICROSECOND !!!!
#define CHARGEDELAY 25                        // Time in microseconds it takes for the cap to charge; at least 5x RC.
// 22 nF & 330R resistor RC = 7.25 us, times 5 = 36.3 us.
#define DISCHARGEDELAY 10                     // Discharge the cap before flipping polarity. Better for the pins. 2xRC.

#define EC_TIMEOUT 2000                       // Timeout for the EC measurement in microseconds.
// 2 ms half cycle --> 250 Hz.


uint32_t getRawECSensor() {

  int32_t buf[EC_BUFFER_SIZE];
  uint32_t dischargeCycles = 0;               // The number of clock cycles it took for the capacitor to discharge.
  uint32_t totalCycles = 0;                   // The cumulative number of clock cycles over all measurements.
  uint32_t startCycle;                        // The clock cycle count at which the measurement starts.
  uint32_t startTime;                         // The micros() count at which the measurement starts (for timeout).
  boolean timeoutStop = false;
  bool timeout = false;

  for (uint16_t i = 0; i < EC_BUFFER_SIZE ; i++) { // take 2^ECSAMPLES measurements of the EC.

    ///////////////////////////////////////////////////////////////////////
    // Stage 1: charge the cap, positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: output, low.
    // EC: input.
    digitalWrite(CP_PIN, HIGH);
    digitalWrite(CN_PIN, LOW);
    pinMode (EC_PIN, INPUT);
    pinMode (CP_PIN, OUTPUT);
    pinMode (CN_PIN, OUTPUT);
    delayMicroseconds(CHARGEDELAY);           // allow the cap to charge fully.

    ///////////////////////////////////////////////////////////////////////
    // Stage 2: measure positive discharge cycle by measuring the number of clock cycles it takes
    // for pin CAPPOS to change from HIGH to LOW.
    // CAPPOS: input.
    // CAPNEG: output, low (unchanged).
    // EC: output, low.
    endCycle = 0;
    pinMode (CP_PIN, INPUT);
    startTime = micros();


    attachInterrupt(CP_PIN, isrCountEndCycle, FALLING);



    startCycle = ESP.getCycleCount();
    GPIO.out_w1tc = ((uint32_t)1 << EC_PIN);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);


    while (endCycle == 0) {                   // endCyle gets set in the ISR, when an interrupt is received.
      if (micros() - startTime > EC_TIMEOUT) { // Time out - in case sensor not connected or not in water.
        timeout = true;
        break;
      }
    }


    detachInterrupt(CP_PIN);

    if (timeout) break;

    dischargeCycles = endCycle - startCycle;
    buf[i] = dischargeCycles;



    ///////////////////////////////////////////////////////////////////////
    // Stage 3: fully discharge the cap, prepare for negative cycle.
    // Necessary to keep total voltage within the allowed range (without these discharge cycles the voltage would jump to about +1.4*Vcc and -0.4*Vcc)
    // CAPPOS: output, low.
    // CAPNEG: output, low (unchanged).
    // EC: input.
    digitalWrite(CP_PIN, LOW);
    digitalWrite(CN_PIN, LOW);
    pinMode (EC_PIN, INPUT);
    pinMode (CP_PIN, OUTPUT);
    pinMode (CN_PIN, OUTPUT);
    delayMicroseconds(DISCHARGEDELAY);

    ///////////////////////////////////////////////////////////////////////
    // Stage 4: charge the cap, negative cycle.
    // CAPPOS: output, low (unchanged).
    // CAPNEG: output, high.
    // EC: input (unchanged).
    digitalWrite (CN_PIN, HIGH);
    delayMicroseconds (CHARGEDELAY);

    ///////////////////////////////////////////////////////////////////////
    // Stage 5: negative discharge cycle, compensation.
    // CAPPOS: input.
    // CAPNEG: output, high (unchanged).
    // EC: output, high.
    digitalWrite (EC_PIN, HIGH);
    pinMode (CP_PIN, INPUT);
    pinMode (EC_PIN, OUTPUT);



    uint32_t dischargeCyclesCompensationStart = ESP.getCycleCount();
    uint32_t dischargeCyclesCompensation = 0;
    while (dischargeCyclesCompensation <= dischargeCycles) {
      dischargeCyclesCompensation =  ESP.getCycleCount() - dischargeCyclesCompensationStart;
    }


    ///////////////////////////////////////////////////////////////////////
    // Stage 6: fully discharge the cap, prepare for positive cycle.
    // CAPPOS: output, high.
    // CAPNEG: ouput, high (unchanged).
    // EC: input.
    digitalWrite(CP_PIN, HIGH);
    pinMode(CP_PIN, OUTPUT);
    pinMode(EC_PIN, INPUT);
    delayMicroseconds(DISCHARGEDELAY);
  }

  if (timeout) {
    dischargeCycles = 0;
  }
  else {
    dischargeCycles = getAverage(buf , pH_BUFFER_SIZE);
  }


  // Disconnect the sensor.
  pinMode(CP_PIN, INPUT);
  pinMode(CN_PIN, INPUT);
  pinMode(EC_PIN, INPUT);

  return dischargeCycles;

}






// string: string to parse
// ar : String Array to feed
// d: delimiter
// returns number of items parsed
int split(String *ar, String string, char d)
{
  String data = "";
  int bufferIndex = 0;

  for (int i = 0; i < string.length(); ++i)
  {
    char c = string[i];

    if (c != d)
    {
      data += c;
    }
    else
    {
      data += '\0';
      ar[bufferIndex] = data;
      bufferIndex++;
      data = "";
    }
  }

  return bufferIndex - 1;
}
