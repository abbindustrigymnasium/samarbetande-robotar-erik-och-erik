#include "EspMQTTClient.h"
#define hal_giv 13

 

byte Pwm_a = 5;
byte Dir_a1 = 1;
int hal_value;
long currentMillis;
bool connectedToMQTT = false;

int interval;
int previousMillis;
long startMillis;
float varvo;
float pwmSpeed = 0;
void onConnectionEstablished();
float desiredSpeed = 30.0;

 

EspMQTTClient client(
  "ABB_Indgym",                           // Wifi ssid
  "7Laddaremygglustbil",                              // Wifi password
  "maqiatto.com",                            // MQTT broker ip
  1883,                                     // MQTT broker port
  "erik.spector@abbindustrigymnasium.se", // MQTT username
  "123456789",                              // MQTT password
  "Spector",                               // Client name
  onConnectionEstablished,              // Connection established callback
  true,                                // Enable web updater
  true                                // Enable debug messages
);
float Kp = 2;
float Ki = 0.5;

float integratedError = 0;
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
  connectedToMQTT = true;
  client.subscribe("erik.spector@abbindustrigymnasium.se/i", [] (const String & payload) {
    integratedError = 0;
    Ki = payload.toInt();
  });
  client.subscribe("erik.spector@abbindustrigymnasium.se/p", [] (const String & payload) {
    Kp = payload.toInt();
  });
  client.subscribe("erik.spector@abbindustrigymnasium.se/getVelocity", [] (const String & payload) {
    desiredSpeed = payload.toInt();
  });
};

 


void startMotor(int pwm) {
  analogWrite(Pwm_a, pwm);
  digitalWrite(Dir_a1, LOW);
}

long deltaT = 0;
float calculateVelocity() {
  deltaT = deltaTime();
  float deltaS = deltaSignals();
  float rSpeed = ((deltaS / 48) * 3.7 * PI) / deltaT * 1000;
  Serial.println(rSpeed);
  return rSpeed;
}

int signals = 0;
 

long deltaTime() {
  long nowMillis = millis();
  long deltaMillis = nowMillis - startMillis;
  startMillis = nowMillis;
  return deltaMillis;
}

 

float deltaSignals() {
  int currentSignals = signals;
  signals = 0;
  return currentSignals;
}

void loop() {
  client.loop();
  if (connectedToMQTT){
    startMotor(pwmSpeed);
    float currentSpeed = calculateVelocity();
    float error = desiredSpeed - currentSpeed;
    Serial.println(Kp);
  
    integratedError += error * deltaT;
    pwmSpeed = error * Kp + integratedError * Ki;
    delay(100);
    client.publish("erik.spector@abbindustrigymnasium.se/currentVelocity", "spector " + String(currentSpeed));
  }
}

 

 

ICACHE_RAM_ATTR void HtoL() {
  signals++;
}
