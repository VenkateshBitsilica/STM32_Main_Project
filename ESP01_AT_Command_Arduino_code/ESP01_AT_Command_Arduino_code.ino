#include <SoftwareSerial.h>

const byte rxPin = 2; 
const byte txPin = 3; 

SoftwareSerial ESP8266 (rxPin, txPin);

void setup() 
{
  Serial.begin(115200);
  ESP8266.begin(115200);  
  delay(1000);         
}

void loop() 
{
  delay(1000);
  Serial.println("Sending an AT command...");
  ESP8266.println("AT");
  delay(30);
  while (ESP8266.available())
  {
     Serial.println("...");
     String inData = ESP8266.readStringUntil('\n');
     Serial.println("ESP8266 Responded for AT Commands: " + inData);
  }  
}