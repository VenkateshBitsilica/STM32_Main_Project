#include <SoftwareSerial.h>

SoftwareSerial sim800(2, 3);  // RX, TX

void setup() {
  Serial.begin(9600);
  sim800.begin(9600);
  Serial.println("Initializing SIM800L...");
  delay(2000); // Let the module stabilize

  sendCommand("AT");           // Check communication
  sendCommand("ATE0");         // Disable echo
  sendCommand("AT+CFUN=1");    // Full functionality mode
  sendCommand("AT+CPIN?");     // SIM status, should return "READY"
  sendCommand("AT+CSQ");       // Signal strength
  sendCommand("AT+CREG?");     // Network registration

  waitForNetwork();            // Wait until registered

  sendCommand("AT+CMGF=1");         // Set SMS mode to text
  sendCommand("AT+CSCS=\"GSM\"");   // Set character set to GSM

  delay(1000);
  Serial.println("Sending SMS...");

  sim800.print("AT+CMGS=\"+919642593997\"\r"); // Replace with your number
  delay(1000);
  sim800.print("Hello from SIM800L!");         // Message content
  delay(500);
  sim800.write(26); // CTRL+Z to send SMS

  Serial.println("SMS Sent!");
}

void loop() {
  // Nothing to do
}

void sendCommand(const char* cmd) {
  sim800.println(cmd);
  Serial.print("Command Sent: ");
  Serial.println(cmd);
  delay(1000);
  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}

void waitForNetwork() {
  int tries = 0;
  while (tries < 10) {
    sim800.println("AT+CREG?");
    delay(1000);
    if (sim800.find("+CREG: 0,1") || sim800.find("+CREG: 1,0") || sim800.find("+CREG: 0,5")) {
      Serial.println("Registered to Network");
      return;
    }
    tries++;
  }
  Serial.println("Not registered to Network");
}
