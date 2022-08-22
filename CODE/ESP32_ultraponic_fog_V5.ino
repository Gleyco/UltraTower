
#define LED_PIN 22
#define MIST1_PIN 19
#define MIST2_PIN 23
#define MIST3_PIN 18
#define PUMP_PIN 5

// TIMER CONST
const int TIME_RESTART_ESP =  86400000; 
const int TIME_ON_PUMP =  40000;    
const int TIME_OFF_PUMP = 6000000; 
const int TIME_ON_EACH_MISTERS = 5000;  
const int TIME_OFF_NONE_MISTERS = 20000;  
const int NUM_REPEAT_CYCLE_MISTERS = 1;



//TIMER variable
unsigned long startTimeMist = 0;
unsigned long startTimePump = 0;
unsigned long startTime = 0;
int intervalTimeMister = 4000;


// setting PWM properties
const int freq = 108000;
const int ledChannel = 0;
const int resolution = 8;
const int dutyCycle = 128;



boolean isMist = 1;
boolean isPump = 1;
int sequenceMist = -1;
int cycleMist = 0;

void setup() {

  Serial.begin(115200);
  delay(100); // give me time to bring up serial monitor

  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);  // canal = 0, frequence = 110000 Hz, resolution = 1 bit   (pas besoin de plus de bit un rapport de 50% ON/OFF est suffisant pour faire vibrer le disque

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MIST1_PIN, ledChannel);
  ledcAttachPin(MIST2_PIN, ledChannel);
  ledcAttachPin(MIST3_PIN, ledChannel);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);



  delay(500);

  Serial.println("BOOT PROCEDURE OK");

  startPump();

  
  startTimeMist = millis();
  startTimePump = millis();
  startTime = millis();
  

}

void loop() {


  //MISTER ROUTINE
  
  if (millis() - startTimeMist > intervalTimeMister) {
    isMist = true;
    ledcDetachPin(MIST1_PIN);
    digitalWrite(MIST1_PIN, LOW);
    pinMode(MIST1_PIN, OUTPUT);
    ledcDetachPin(MIST2_PIN);
    digitalWrite(MIST2_PIN, LOW);
    pinMode(MIST2_PIN, OUTPUT);
    ledcDetachPin(MIST3_PIN);
    digitalWrite(MIST3_PIN, LOW);
    pinMode(MIST3_PIN, OUTPUT);
    sequenceMist ++;
    

    if (sequenceMist > 3) {
      intervalTimeMister = TIME_ON_EACH_MISTERS;
      sequenceMist = 0;
    }
    
    if (sequenceMist == 0) {
    //  Serial.println("MIST 0");
      
      ledcAttachPin(MIST1_PIN, ledChannel);
    }else if(sequenceMist == 1) {
     // Serial.println("MIST 1");
      
      ledcAttachPin(MIST2_PIN, ledChannel);
    }else if(sequenceMist == 2){
    //  Serial.println("MIST 2");
      ledcAttachPin(MIST3_PIN, ledChannel);

      cycleMist++;

      if(cycleMist > NUM_REPEAT_CYCLE_MISTERS){
        cycleMist = 0;
      }else{
        sequenceMist = -1;
      }

      
    }else {
      Serial.println("BREAK MIST");
      intervalTimeMister = TIME_OFF_NONE_MISTERS;
      isMist = false;
    }

    ledcWrite(ledChannel, dutyCycle);
    startTimeMist = millis();

  }


  
  if (millis() - startTimePump > TIME_OFF_PUMP && !isMist) {

    startPump();
  }



  if (millis() - startTime > TIME_RESTART_ESP ) {
    Serial.println("RESTART EVERY 24 HOURS");
    ESP.restart();
  }

  
}


void startPump(){

  isPump = true;
  startTimePump = millis();
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);

  Serial.println(touchRead(T3));

  int i = 0;
  int touchReadValue = 0;

  while (isPump){
       if(millis() - startTimePump > TIME_ON_PUMP){ //Attendre 2 s de connection sinon echec
         Serial.println("TIME ON PUMP END");
          break;
       }

       touchReadValue = touchRead(T3);


       if(touchReadValue < 50){
        i++;
       }else{
        i = 0;
       }

       Serial.println(touchReadValue);
       Serial.print("Threehold touch : ");
       Serial.println(i);

        if(i >= 5){
          isPump = false;
          Serial.println("It's full");
        }

       delay(100);

   }


 digitalWrite(PUMP_PIN, LOW);
 digitalWrite(LED_PIN, HIGH);
    
 startTimePump = millis();

    
}
