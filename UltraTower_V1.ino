


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


WiFiClient  client;


#define VERSION 1   //INT Type Only

//ESP32 PIN
#define LED_PIN 22
#define MIST1_PIN 18
#define MIST2_PIN 5
#define PUMP_PIN 17
#define BLE_PIN 4
#define ENABLE_SENSORS_PIN 26
#define ONE_WIRE_TEMP 27
#define DHT_SENSOR_PIN  14 
#define pH_PIN 13
#define TANK_HIGH_PIN 2
#define TANK_LOW_PIN 15
#define CP_PIN 32
#define CN_PIN 33
#define EC_PIN 25



//TEMPERATURE SENSORS INIT
#define TEMPERATURE_PRECISION 10
OneWire oneWire(ONE_WIRE_TEMP); 
DallasTemperature tempSensors(&oneWire);

#define DHT_SENSOR_TYPE DHT22
DHT dhtSensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);


uint8_t tankThermometer[8];
uint8_t rootThermometer[8];

#define ROOT_CODE 0
#define TANK_CODE 1
//DallasTemperature tempSensors();



#define VALUE_THRESHOLD 50
#define VALUE_THRESHOLD_WATER_LEVEL 55
#define TITLE_PREF "Pref"



//BLE 
#define SERVICE_UUID           "367fa97d-b107-49e0-9503-3409d2c247c9" 
#define CHARACTERISTIC_UUID_RX "367fa97d-b108-49e0-9503-3409d2c247c9"
#define CHARACTERISTIC_UUID_TX "367fa97d-b109-49e0-9503-3409d2c247c9"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool bleActivated = false;



// TIMER CONST
#define TIME_RESTART_ESP   86400000 
#define TIME_CHECK_WATER_LEVEL   120000 
#define WIFI_TIMEOUT 30000 // 1 min in milliseconds
const int TIME_ON_PUMP =  240000; //4 min
const int TIME_ON_PUMP_INIT = 10000; //10 secondes      
const int TIME_OFF_PUMP = 1800000; //30 min
const int TIME_ON_MISTERS = 1000;  //10 min
const int TIME_OFF_NONE_MISTERS = 1000; //5 min 
#define TIME_CHECK_BUTTON_BLE 1000 //1 seconde
#define BLE_ACTIVE_TIMEOUT 120000 // 20min in milliseconds



// Misters VARIABLES
boolean enableMist = 0;  // Default OFF
int frequencyMister = 108000; // Default 108 kHz
int timeOnMister = 600000; // Default 10 min
int timeOffMister = 600000; // Default 10 min


// pH VARIABLES
boolean enablepH = 0; // Default OFF
int pHCalib4 = -1; // Default 
int pHCalib9 = -1; // Default 
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
int timeCheckSensors = 20000; // Default 10 min

boolean isECpHTempCompensation  = false; // Default FALSE

//ThingSpeak VARIABLES
String ssid = "" ;
String pw = "";
long myChannelNumber = 0;
String myWriteAPIKey = "";
bool enableThingSpeak = false;  // Default false 



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


// setting PWM properties
const int ledChannel = 0;
const int resolution = 1;
const int dutyCycle = 1;



boolean isMist = 1;
boolean isPump = 1;
int sequenceMist = -1;
int cycleMist = 0;

const int capacity = JSON_OBJECT_SIZE(40);
StaticJsonDocument<capacity> jsonBuffer;

void setup() {


  pinMode(MIST1_PIN, INPUT);
  pinMode(MIST2_PIN, INPUT);

  Serial.begin(115200);
  delay(100); // give me time to bring up serial monitor

  
  // configure LED PWM functionalitites
  

 
  

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);


  digitalWrite(ENABLE_SENSORS_PIN, LOW);
  pinMode(ENABLE_SENSORS_PIN, OUTPUT);

 /*Preferences preferences;
         preferences.begin(TITLE_PREF, false);
         preferences.clear();
         preferences.end();*/


  delay(500);

  Serial.println("BOOT PROCEDURE OK");

  //startPump();

  
  
 

  getPreferences();

  startTimeMist = millis() - timeOffMister;
  startTimePump = millis();
  startTimeSensors = millis();
  startTime = millis();


  ledcSetup(ledChannel, frequencyMister, resolution);  // canal = 0, frequence = 108000 Hz, resolution = 1 bit   (pas besoin de plus de bit un rapport de 50% ON/OFF est suffisant pour faire vibrer le disque
  
  isPump = false;
  isMist = false;

}

void loop() {


  //BLE Button Activation
  if(touchRead(BLE_PIN) < VALUE_THRESHOLD && !bleActivated){
    checkIfStartBle();  
  }
 

  if(bleActivated){
    if(deviceConnected){
      
    }else{
      if (millis() - startTimeBle > BLE_ACTIVE_TIMEOUT) {
       // pServer->getAdvertising()->stop();
       // bleActivated = false;
        Serial.println("BLE OFF");
        Serial.println("RESTART ESP32");
        delay(100);
        ESP.restart();
        
      }
    }
  }



  


  //MISTER ROUTINE
  if(enableMist){
    if(isMist){
       if (millis() - startTimeMist > timeOnMister) {setMisterOff();}

       if (millis() - startTimeCheckWaterLevel > TIME_CHECK_WATER_LEVEL) {
        if(checkIfPinIsInWater(TANK_LOW_PIN)){
          startTimeCheckWaterLevel = millis();
        }else{
          startPump();
        }

        }
    }else{

      
        if (millis() - startTimeMist > timeOffMister) {
          if(checkIfPinIsInWater(TANK_LOW_PIN)){
            setMisterOn();
          }else{
            startPump();
          }
          
          
      }
    }
  }

  //PUMP ROUTINE
  if(isPump){
    if(touchRead(TANK_HIGH_PIN) < VALUE_THRESHOLD_WATER_LEVEL){
       if(checkIfPinIsInWater(TANK_HIGH_PIN)){
        Serial.println("WATER LEVEL : FULL STOP PUMP");
        digitalWrite(PUMP_PIN, LOW);
        digitalWrite(LED_PIN, HIGH);
        isPump = false;
      }
    }
   

    if(millis() - startTimePump > TIME_ON_PUMP){
         Serial.println("TIME OUT PUMP : ERROR STOP PUMP");
         isPump = false;
         digitalWrite(PUMP_PIN, LOW);
         digitalWrite(LED_PIN, HIGH);
     
    }

      pinMode(TANK_HIGH_PIN, INPUT);
 }  


  //SENSORS ROUTINE
  if(bleActivated || enableThingSpeak){
    if (millis() - startTimeSensors > timeCheckSensors) {checkSensors();}
  }
  
  



  
  if (millis() - startTimePump > TIME_OFF_PUMP && !isMist) {
    startPump();
  }



  if (millis() - startTime > TIME_RESTART_ESP ) {
    Serial.println("RESTART EVERY 24 HOURS");
    ESP.restart();
  }

  
}


//**********************************************************************************************************************************************************
//*******************************************************       MISTERS FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************

void setMisterOn(){
     isMist = true;
  
    digitalWrite(MIST1_PIN, LOW);
    pinMode(MIST1_PIN, OUTPUT);
    digitalWrite(MIST2_PIN, LOW);
    pinMode(MIST2_PIN, OUTPUT);

    
    ledcAttachPin(MIST1_PIN, ledChannel);
    ledcAttachPin(MIST2_PIN, ledChannel);
    ledcWrite(ledChannel, dutyCycle);

     startTimeMist = millis();
     sendStatutNotif("MON");
}

void setMisterOff(){
    isMist = false;
    ledcDetachPin(MIST1_PIN);
    pinMode(MIST1_PIN, INPUT);
    ledcDetachPin(MIST2_PIN);
    pinMode(MIST2_PIN, INPUT);
    startTimeMist = millis();
    sendStatutNotif("MOFF");
}


//**********************************************************************************************************************************************************
//*******************************************************      PUMP FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************

bool checkIfPinIsInWater(int pin){
  unsigned long startTime = millis();
  int i = 0;
  
   while (1){
           if(millis() - startTime > 100){ //Attendre 2 s de connection sinon echec
             Serial.println("WATER LEVEL : Pin is not in water");
             return false;
             break;
           }    
   
    
           if(touchRead(pin) < VALUE_THRESHOLD_WATER_LEVEL){
            i++;
           }else{
            i = 0;
           }
           Serial.print(i);
    
            if(i >= 5){
              Serial.println("WATER LEVEL : PIN OK");
              return true;
              break;
            }
    
       }

    pinMode(pin, INPUT);
}

void startPump(){

  isPump = true;
  Serial.println("START PUMP");
  
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);

 /*  for (uint16_t i=0; i < 5; i++){
     digitalWrite(PUMP_PIN, LOW);
     delay(100);
     digitalWrite(PUMP_PIN, HIGH);
     delay(100);
    
   }*/
 // digitalWrite(PUMP_PIN, HIGH);

  int i = 0;
  int touchReadValue = 0;
  startTimePump = millis();

  while (isPump){
       if(millis() - startTimePump > TIME_ON_PUMP_INIT){
         Serial.println("TIME OUT PUMP : ERROR");
         isPump = false;
         break;
       }

       touchReadValue = touchRead(TANK_LOW_PIN);

       if(touchReadValue < VALUE_THRESHOLD_WATER_LEVEL){
        i++;
       }else{
        i = 0;
       }

        if(i >= 5){
          Serial.println("WATER LEVEL : BOTTOM TANK REACH");
          break;
        }

       delay(100);

   }


    
 if(!isPump){
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
 }  
 startTimeCheckWaterLevel = millis();
}






void checkIfStartBle() {
  unsigned long startTimeButton = millis();
  int i = 0;
  

 
   while (1){
           if(millis() - startTimeButton > TIME_CHECK_BUTTON_BLE){ //Attendre 2 s de connection sinon echec
             Serial.println("TIME OUT BLE NOT ACTIVATED");
              break;
           }    
   
    
           if(touchRead(BLE_PIN) < VALUE_THRESHOLD){
            i++;
           }else{
            i = 0;
           }
    
            if(i >= 5){
              initBLE();
              break;
            }
    
           delay(10);
       }
  
}



void sendStatutNotif(String str){
  if(bleActivated){
    if(deviceConnected){
       sendNotifDataOverBLE(str);
    }
  }
}



void sendNotifDataOverBLE(String str){  //20 bytes in your notification data payload

      int sizeStr = str.length()+1;
      char buf[sizeStr];  
      str.toCharArray(buf,sizeStr);
      
      pTxCharacteristic->setValue((uint8_t*)&buf[0], sizeStr);
      pTxCharacteristic->notify();
      delay(10); //Delay between two packets
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      activateSensors();
      Serial.println("DEVICE CONNECTED");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      startTimeBle = millis();
      Serial.println("DEVICE DISCONNECTED");
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

      if(err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.c_str());
        return;
        }

       JsonObject object = jsonBuffer.as<JsonObject>();
       const char* cmd = object["CMD"];
       Serial.println(cmd);

       if (strcmp(cmd, "MFREQ") == 0 ){
             frequencyMister = object["DATA"];
             ledcSetup(ledChannel, frequencyMister, resolution);
             sendNotifDataOverBLE("MFREQ");
         
        
       }else if (strcmp(cmd, "MENA") == 0){
            enableMist = object["DATA"];
            sendNotifDataOverBLE("MENA");
            if(!enableMist){setMisterOff();}
   
       }else if (strcmp(cmd, "MCYCL") == 0){
            timeOnMister = object["DATA_ON"];
            timeOffMister = object["DATA_OFF"];
            sendNotifDataOverBLE("MCYCL");
   
       }else if (strcmp(cmd, "TENA") == 0){
            enableAmbiantTemp = object["DATA_AMB"];
            enableRootTemp = object["DATA_ROOT"];
            enableTankTemp = object["DATA_TANK"];
            sendNotifDataOverBLE("TENA");
   
       }else if (strcmp(cmd, "TOFFS") == 0){
            offSetAmbiantTemp = object["DATA_AMB"];
            offSetRootTemp = object["DATA_ROOT"];
            offSetTankTemp = object["DATA_TANK"];
            sendNotifDataOverBLE("TOFFS");
   
       }else if (strcmp(cmd, "TFUSE") == 0){
            activateSensors();
            String strToReturn = OneWireScanner(object["DATA"]);
            String strTosend = "TFUSE:" + strToReturn;
            sendNotifDataOverBLE(strTosend);
   
       }else if (strcmp(cmd, "THING") == 0){
            enableThingSpeak = object["DATA_ENA"];
            ssid = String(object["SSID"].as<char*>()); 
            myWriteAPIKey =String(object["APIKEY"].as<char*>()); 
            myChannelNumber = object["CHANNEL"]; 

            String newPW = String(object["PW"].as<char*>()); 
            if(newPW != ""){ pw = newPW; }      
            sendNotifDataOverBLE("THING");
            
       }else if (strcmp(cmd, "PHENA") == 0){
            enablepH = object["DATA"];
            sendNotifDataOverBLE("PHENA");
           
   
       }else if (strcmp(cmd, "PHCAL") == 0){
            activateSensors();
            delay(100);
            getRawPhSensor();
            String strToReturn = String(getRawPhSensor());
            String strTosend = "PHCAL:" + strToReturn;
            sendNotifDataOverBLE(strTosend);
          
       }else if (strcmp(cmd, "PHSET") == 0){
            pHCalib4 = object["DATA4"];
            pHCalib9 = object["DATA9"]; 
            sendNotifDataOverBLE("PHSET");
          
       }else if (strcmp(cmd, "ECENA") == 0){
            enableEC = object["DATA"];
            sendNotifDataOverBLE("ECENA");
           
   
       }else if (strcmp(cmd, "ECCAL") == 0){
           
            String strToReturn = String(getRawECSensor());
            String strTosend = "ECCAL:" + strToReturn;
            sendNotifDataOverBLE(strTosend);
          
       }else if (strcmp(cmd, "ECSET") == 0){
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
      jsonBuffer["MFREQ"].set(frequencyMister);
      jsonBuffer["MON"].set(timeOnMister);
      jsonBuffer["MOFF"].set(timeOffMister);

      jsonBuffer["PHENA"].set(enablepH);
      if(pHCalib4 !=-1){
        jsonBuffer["PHCAL"].set(true);
      }else{
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

      
      jsonBuffer["ECENA"].set(enableEC);
      if(ECCalib != ""){
        jsonBuffer["ECCAL"].set(true);
      }else{
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
                    BLECharacteristic::PROPERTY_NOTIFY|
                    BLECharacteristic::PROPERTY_INDICATE
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());
 

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  Serial.println("BLE READY");

  startTimeBle = millis();
  //startTimeLED = millis();

  bleActivated = true;

  delay(2);
}









//**********************************************************************************************************************************************************
//*******************************************************       PREFERENCES FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************

void getPreferences(){
  Preferences preferences;
  preferences.begin(TITLE_PREF, false);

  enableMist =  preferences.getBool("ENAMIST", 0);
  frequencyMister =  preferences.getInt("FREQMIST", 106000);
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
  ssid = preferences.getString("SSID", "");
  pw = preferences.getString("PW", "");
  myWriteAPIKey = preferences.getString("APIKEY", "");
  myChannelNumber = preferences.getLong("CHANNEL", 0);

  enableEC = preferences.getBool("ENAEC", 0);
  ECCalib = preferences.getString("ECCALIB", "");
   
  
  preferences.end();
}



void savePreferences(){

  
  Preferences preferences;
  preferences.begin(TITLE_PREF, false);


   preferences.putBool("ENAMIST", enableMist);
   preferences.putInt("FREQMIST", frequencyMister);
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


   preferences.putBool("ENAEC", enableEC);
   preferences.putString("ECCALIB", ECCalib);
   






  

  
  preferences.end();
 
  
}

//**********************************************************************************************************************************************************
//*******************************************************       SENSORS FUNCTIONS        *******************************************************************
//**********************************************************************************************************************************************************



void activateSensors(){
  digitalWrite(ENABLE_SENSORS_PIN, HIGH); //3.3V for temperature sensors and pH sensor
}

void switchOffSensors(){
  digitalWrite(ENABLE_SENSORS_PIN, LOW); //0V for temperature sensors and pH sensor                                   //TODO ajouter EC
}


bool sendToThingSpeak = false;
void checkSensors(){
  Serial.println("");
  Serial.println("********************************************");
  Serial.println("ROUTINE SENSORS");
  activateSensors();
  delay(200);

  sendToThingSpeak = false;

  if(enableThingSpeak && !bleActivated){
    
    sendToThingSpeak = initThingSpeak();
    
    if(!sendToThingSpeak){turnOffWiFi();}
    
  }

 
  //Request Tank and Root temperature if enable
  if(enableTankTemp) { requestRootTankTemp(); }

  //Request ambiant temperature if enable
  if(enableAmbiantTemp){ requestDHT22Temp(); }

  //Request pH if enable
  if(enablepH){ requestpH(); }

  //Request EC if enable
  if(enableEC){ requestEC(); }

  



  if(sendToThingSpeak){sendFieldsToThingSpeak();}  //Send to thingspeak if enable

  

  switchOffSensors();
  startTimeSensors = millis();
}


int32_t getAverage(int32_t buf[] , int sizeBuf){
  quickSort(buf, sizeBuf);

  int32_t avg = 0;
  int16_t nbToStart = sizeBuf/4;
  int16_t nbToEnd = sizeBuf - nbToStart;
  for (int16_t i= nbToStart; i< nbToEnd; i++){
      avg += buf[i];
   }
 
  return avg /  (sizeBuf - 2*nbToStart);
  
}

void quickSort(int32_t *ar, int32_t n){
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

void requestRootTankTemp(){
  tempSensors.requestTemperatures();
  tempSensors.begin();
  float temp = 0;

  if(enableTankTemp){
    if(tempSensors.isConnected(tankThermometer)){
      
      temp = tempSensors.getTempC(tankThermometer);
      temp = temp + offSetTankTemp;
      Serial.print("Tank Temperature: ");
      Serial.println(temp);

     if(deviceConnected){
      String strTosend = "TTANK:" ;
      strTosend.concat(temp);
      sendNotifDataOverBLE(strTosend);
     }
      
      if(sendToThingSpeak){ThingSpeak.setField(1, temp);}
    }else{
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
}

void requestDHT22Temp(){
  dhtSensor.begin();
  delay(2000);
  float temp = dhtSensor.readTemperature(false, true);
  temp = temp + offSetAmbiantTemp;

  Serial.print("Ambiant Temperature: ");
  Serial.println(temp);

   if(deviceConnected){
    String strTosend = "TAMBI:" ;
    strTosend.concat(temp);
    sendNotifDataOverBLE(strTosend);
   }

   if(sendToThingSpeak){ThingSpeak.setField(2, temp);}

}










//For the first use the tower must scan the address of the sensors.
String OneWireScanner(int code){
  Serial.println("Temperature sensor scanner");
  uint8_t address[8];

 
  tempSensors.requestTemperatures();
  tempSensors.begin();
  
  int count = tempSensors.getDeviceCount();
  Serial.print("Number of device : ");
  Serial.println(count);

  if(count == 0){ // No sensors connected
    return "0";
  }

  if(count == 1){ // No sensors connected
    if (tempSensors.getAddress(address , 0)){
      if(code == TANK_CODE){
        Serial.print("ADRESS OF TANK TEMP :");
      }else{
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

      if(code == TANK_CODE){
        memcpy(tankThermometer, address, sizeof(address));
       
       
      }else{
        memcpy(rootThermometer, address, sizeof(address));
      }

      return "1";
    }else{
      return "3";
    }
  
    
  }

   if(count > 1){ // too many sensors connected
    Serial.println("Too many sensors connected");
    return "2";
  }

  Serial.println("Stop One wire scanner");

  return "3";
}


//**********************************************************************************************************************************************************
//*******************************************************         pH FUNCTIONS           *******************************************************************
//**********************************************************************************************************************************************************


void requestpH(){
  float value = 0;

  int calibpH4 = pHCalib4;
  int calibpH9 = pHCalib9;

  if(calibpH4 == -1){
    calibpH4 = PH4_DEFAULT;
  }

  if(calibpH9 == -1){
    calibpH9 = PH9_DEFAULT;
  }
  
  

  value = map(getRawPhSensor(), calibpH4, calibpH9, 4.00, 9.18);

  Serial.print("pH Value: ");
  Serial.println(value);
  
  if(deviceConnected){
    String strTosend = "PHVAL:" ;
    strTosend.concat(value);
    sendNotifDataOverBLE(strTosend);
   }
     
  if(sendToThingSpeak){ThingSpeak.setField(4, value);}
}





int32_t getRawPhSensor(){
  int32_t valueSensor = 0;
  int32_t buf[pH_BUFFER_SIZE];

  analogSetPinAttenuation(pH_PIN, ADC_2_5db);


  for(int i=0;i<pH_BUFFER_SIZE;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(pH_PIN);
    
  }

  return getAverage(buf , pH_BUFFER_SIZE);
}








//**********************************************************************************************************************************************************
//*******************************************************      THINGSPEAK FUNCTIONS      *******************************************************************
//**********************************************************************************************************************************************************


bool initThingSpeak(){
  Serial.println("Init WIFI and ThingSpeak");
  
  
  // CONNECTION AU WIFI
  unsigned long startAttemptTime = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pw.c_str());


   while (WiFi.status() != WL_CONNECTED){ //Changement de && par || TODO peut être remplacé par un if
     Serial.println("*");
    if(millis() - startAttemptTime > WIFI_TIMEOUT){
      break;
    }
    delay(20);
  }


  // Make sure that we're actually connected, otherwise return false
  if(WiFi.status() != WL_CONNECTED){
    return false;
  }

  return ThingSpeak.begin(client);  // Initialize ThingSpeak
}


void sendFieldsToThingSpeak(){
  Serial.println("");
  Serial.println("**************************************************");
  Serial.println("Send to ThingSpeak");


   // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey.c_str());
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  turnOffWiFi();
 
}

void turnOffWiFi(){
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

//**********************************************************************************************************************************************************
//*******************************************************         EC FUNCTIONS           *******************************************************************
//**********************************************************************************************************************************************************


void requestEC(){

  String str = "";

  if(ECCalib == ""){
    str = EC_DEFAULT;
  }else{
    str = ECCalib;
  }
  


  
  int valueSensor = getRawECSensor();
  float value = 0.0;

  String arrayMain[6]; // MAX 6 values of calibrations
  String arraySecond[3]; // MAX 3 values of calibrations
  String arrayEquation[2]; // MAX 2 values of equation

  
  int sizeArrayMain = split(arrayMain, str , ':');

  for(int i = 0 ; i<= sizeArrayMain ; i++){
    
    int sizeArraySecond = split(arraySecond, arrayMain[i] , '/');

    if(arraySecond[0].toInt() >= valueSensor && valueSensor > arraySecond[1].toInt()){
   
      split(arrayEquation, arraySecond[2] , '|');

      Serial.println(arrayEquation[0]);
      Serial.println(arrayEquation[1]);
      
      value = arrayEquation[0].toFloat() + (arrayEquation[1].toFloat() *valueSensor);
      break;
    }
   
  }

  Serial.print("EC Value: ");
  Serial.println(value);

   if(deviceConnected){
    String strTosend = "ECVAL:" ;
    strTosend.concat(value);
    sendNotifDataOverBLE(strTosend);
   }
     
  if(sendToThingSpeak){ThingSpeak.setField(5, value);}

  
}








volatile uint32_t endCycle = 0; 
void IRAM_ATTR isrCountEndCycle() {
  //__asm__ __volatile__("esync; rsr %0,ccount":"=a" (endCycle));
    endCycle = ESP.getCycleCount();
    // trigger = true;
}

hw_timer_t *timerEC = NULL;
void IRAM_ATTR onTimer(){
     endCycle = -1;
}


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
  for (uint16_t i=0; i < EC_BUFFER_SIZE ; i++) { // take 2^ECSAMPLES measurements of the EC.

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
    pinMode (CP_PIN,INPUT);

    timerEC = timerBegin(2, 80, true);
    timerAttachInterrupt(timerEC, &onTimer, true);
    timerAlarmWrite(timerEC, EC_TIMEOUT, false);
    timerAlarmEnable(timerEC);
    attachInterrupt(CP_PIN, isrCountEndCycle, FALLING); 
    
    startCycle = ESP.getCycleCount();
    GPIO.out_w1tc = ((uint32_t)1 << EC_PIN);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
  
    while (endCycle == 0) {           // endCyle gets set in the ISR, when an interrupt is received.
    }
    
    timerDetachInterrupt(timerEC);
    timerEC = NULL;
    detachInterrupt(CP_PIN);
    
    if(endCycle == -1){
      timeoutStop = true;
      break;
      }
    
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
    pinMode (CP_PIN,INPUT);
    pinMode (EC_PIN, OUTPUT); 



    uint32_t dischargeCyclesCompensationStart = ESP.getCycleCount();
    uint32_t dischargeCyclesCompensation = 0;
    while (dischargeCyclesCompensation <= dischargeCycles) {   
     dischargeCyclesCompensation =  ESP.getCycleCount() - dischargeCyclesCompensationStart;
    }
   // delayMicroseconds (dischargeCycles / 250);
  

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
  
  if (timeoutStop) {
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

  return bufferIndex-1;
}
