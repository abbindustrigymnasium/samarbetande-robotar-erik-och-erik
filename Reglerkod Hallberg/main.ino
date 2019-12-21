#include "EspMQTTClient.h"
#define hal_giv 12

byte Pwm_a = 5;
byte Dir_a1 = 0;
int hal_value;
bool drive = false;
int interval;
long startMillis;
float pwmSpeed = 0;
void onConnectionEstablished();
float desiredSpeed = 0;

EspMQTTClient client(
  "ABB_Indgym_Guest",                           // Wifi ssid
  "Welcome2abb",                              // Wifi password
  "maqiatto.com",                            // MQTT broker ip
  1883,                                     // MQTT broker port
  "erik.spector@abbindustrigymnasium.se", // MQTT username
  "123456789",                              // MQTT password
  "Erik",                               // Client name
  onConnectionEstablished,              // Connection established callback
  true,                                // Enable web updater
  true                                // Enable debug messages
);
float Kp = 0.01;
float Ki = 1 ;
void setup() {
  pinMode(Pwm_a, OUTPUT);
  pinMode(Dir_a1, OUTPUT);
  pinMode(12, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(hal_giv), HtoL, FALLING);
  startMillis = millis();

}
float wantValue;
void onConnectionEstablished() {
  client.subscribe("erik.spector@abbindustrigymnasium.se/getVelocity", [] (const String & payload) { // Subscribar på hastigheten 
    desiredSpeed = payload.toInt();
  });
  client.subscribe("erik.spector@abbindustrigymnasium.se/p", [] (const String & payload) {
    Kp = payload.toInt();
  });
  client.subscribe("erik.spector@abbindustrigymnasium.se/i", [] (const String & payload) {
    Ki = payload.toInt();
  });

}


void startMotor(int pwm) { // funcktion för att köra motorn, tar in ett pwm värde. 
  analogWrite(Pwm_a, pwm);
  digitalWrite(Dir_a1, LOW);

}


float signals;
float previousSignals;
long deltaT = 0;
float motor() { // räknar ut hastigheten på bilen. 
  deltaT = deltaTime(); 
  float deltaS = deltaSignals();
  float rSpeed = ((deltaS / 96) * 3.7 * PI) / deltaT*1000; // hastigheten räknas ut genom: antal hjulvarv * omkretsen / deltaT. Då får vi cm/ms för att få det i cm/s, multiplicerar jag med 1000. 
  Serial.println(rSpeed);
  return rSpeed;
}

float deltaTime() { // funktion som returnar tiden sen senaste mättningen. 
  float nowMillis = millis();
  float deltaMillis = nowMillis - startMillis;
  startMillis = nowMillis;
  return deltaMillis;
}

float deltaSignals() { // returnar hur antalet signaler från hallgivaren sedan senaste mättnigen. 
  int currentSignals = signals;
  signals = 0;
  return currentSignals;
}

float integratedError = 0;

void loop() {
  client.loop();
  
    startMotor(pwmSpeed);
    float currentSpeed = motor();
    float error = desiredSpeed - currentSpeed; // räknar ut felet
  
  
    integratedError += error * deltaT/1000; // integrerandetermen 
    pwmSpeed = error * Kp + integratedError * Ki; // proportionellatermen + integrerande multiplicerat med respektive konstant. 
    if (pwmSpeed > 1023){ // gör så att pwm inte går över 1023 vilket är max
      pwmSpeed = 1023;
    }
    Serial.println(pwmSpeed);
    Serial.println(integratedError);
    
    client.publish("erik.spector@abbindustrigymnasium.se/currentVelocity", "hallberg " + String(currentSpeed)); // skickar den hasrigheten till mqqt

 
}



ICACHE_RAM_ATTR void HtoL() { // funktionen som räknar singnaler från hallgivaren. 
  signals++;
}
