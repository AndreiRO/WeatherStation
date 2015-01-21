#include <HIH4030.h>
#include <Wire.h>
#include <SFE_BMP180.h>

#define DEBUG 0

#define HIH4030_PIN A1
#define HIH4030_SUPPLY_VOLTAGE  5.0
#define ARDUINO_VCC 5.0
#define SLOPE 0.03068
#define OFFSET 0.958


SFE_BMP180 bmpSenzor = SFE_BMP180();
HIH4030 hihSenzor(HIH4030_PIN, HIH4030_SUPPLY_VOLTAGE, ARDUINO_VCC);

const unsigned long delayTime = 1000L;
const unsigned long COLLECT_TIME = 4000L;/*1000L*2 _ **60L*60L*6L*/


const int WORKING_PIN = 3;
short int WORKING_PIN_STATE = LOW;

const int COLLECTING_PIN = 2;
short int COLLECTING_PIN_STATE = LOW;

const int LIGHT_PIN = 0;

const int ERROR_PIN = 5;

unsigned long currentTime = 0;
unsigned long currentPassed = 0;


void setup() {
  pinMode(WORKING_PIN, OUTPUT);
  pinMode(COLLECTING_PIN, OUTPUT);
  pinMode(ERROR_PIN, OUTPUT);
  
  Serial.begin(9600);
  
  hihSenzor.calibrate(SLOPE, OFFSET);
  
  if (bmpSenzor.begin()) {
    Serial.println("Weather station init success\n");
  } else {
    digitalWrite(ERROR_PIN, HIGH);
    Serial.println("Weather station init fail\n");
    while(1); // Pause forever.
  }
  
}


void debug() {
  Serial.print("W: ");
  Serial.print(WORKING_PIN_STATE);
  Serial.print(" C: ");
  Serial.print(COLLECTING_PIN_STATE);
  Serial.print(" T: ");
  Serial.print( currentTime);
  Serial.print(" Col:");
  Serial.print(COLLECT_TIME);
  Serial.print(" P: ");
  Serial.print(currentPassed);
  Serial.println("\n");
}

void loop() {
  showWorking();
  collectData();
  delay(delayTime);
 
  if(DEBUG) {
   debug();
  }
 
}

void toggle(int pin, short& state) {
 
   if(state == LOW) {
     state = HIGH; 
   } else {
     state = LOW; 
   }
  
   digitalWrite(WORKING_PIN, WORKING_PIN_STATE);
}

void showWorking() {
   toggle(WORKING_PIN, WORKING_PIN_STATE); 
}

void collectData() {
  
  if(COLLECTING_PIN_STATE == HIGH) {
    COLLECTING_PIN_STATE = LOW;
    digitalWrite(COLLECTING_PIN, LOW);
  }
  
  if(readyToCollectData()) {
     // collect pressure and temperature
     double temp;
     double press;
     double humid;
     double light;
     
     char status;
     getBMPData(press, temp);
     humid = hihSenzor.getSensorRH();
     light = getLight();
    
     sendData(temp, press, humid, light);

     COLLECTING_PIN_STATE = HIGH;
     digitalWrite(COLLECTING_PIN, HIGH);

    }
 }


short readyToCollectData() {
    if(currentTime < COLLECT_TIME) {
        currentTime += delayTime;
        return 0;
    } else {
      currentTime = 0;
      return 1;
    }
}


void sendData(double temperature, double pressure, double humidity, double light) {
   Serial.print("Temperature: ");
   Serial.print(temperature);
   Serial.print(" Pressure: ");
   Serial.print(pressure);
   Serial.print(" Humidity: ");
   Serial.print(humidity);
   Serial.print(" Light: ");
   Serial.print(light);
   Serial.println("\n");
}


void getBMPData(double& pressure, double& temp) {
  char status;
  double T,P,p0,a;

  status = bmpSenzor.startTemperature();
  if (status != 0) {

    delay(status);
  
    status = bmpSenzor.getTemperature(temp);
    if (status != 0) {

      status = bmpSenzor.startPressure(3);
      if (status != 0) {
        delay(status);

        status = bmpSenzor.getPressure(pressure,temp);
        if (status == 0) {
           Serial.println("error retrieving pressure measurement\n");
        }
        
      } else {
        Serial.println("error starting pressure measurement\n");
        digitalWrite(ERROR_PIN, HIGH);
      }
      
    } else { 
      Serial.println("error retrieving temperature measurement\n");
      digitalWrite(ERROR_PIN, HIGH);
    }
  } else {
    Serial.println("error starting temperature measurement\n");
    digitalWrite(ERROR_PIN, HIGH);
  }
  
}

double getLight() {
   int senzorValue = analogRead(LIGHT_PIN);
   double light    = (double)senzorValue / 1023. * 100;
   
   return light;
}
