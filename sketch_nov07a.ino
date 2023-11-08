#include <ESP8266WiFi.h>
#include <WiFiClient.h>

const char* ssid = "your-SSID"; // Change to your Wi-Fi network name
const char* password = "your-PASSWORD"; // Change to your Wi-Fi password
const int serverPort = 8888; // Port to listen on

const int motor1A = 10;
const int motor1B = 9;
const int motor1E = 8; // Enable pin for Motor 1
const int motor2A = 7;
const int motor2B = 6;
const int motor2E = 5; // Enable pin for Motor 2

WiFiServer server(serverPort);

void setup() {
  Serial.begin(115200);
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor1E, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motor2E, OUTPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  server.begin();
  Serial.println("Server started.");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char command = client.read();
        executeCommand(command);
      }
    }
  }

  if (Serial.available() > 0) {
    char command = Serial.read();
    client.print(command);
  }

  // Read distance data from Arduino Mega (replace with your actual distance reading logic)
  int distance = analogRead(A0); // Use the appropriate pin connected to the sensor
  client.print(distance);

  delay(1000);
}

void executeCommand(char command) {
  switch (command) {
    case 'F':
      // Move forward
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
      digitalWrite(motor1E, HIGH); // Enable Motor 1
      digitalWrite(motor2A, HIGH);
      digitalWrite(motor2B, LOW);
      digitalWrite(motor2E, HIGH); // Enable Motor 2
      break;
    case 'B':
      // Move backward
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, HIGH);
      digitalWrite(motor1E, HIGH); // Enable Motor 1
      digitalWrite(motor2A, LOW);
      digitalWrite(motor2B, HIGH);
      digitalWrite(motor2E, HIGH); // Enable Motor 2
      break;
        case 'L':
      // Turn left
      // Implement left turn logic by turning off one motor while keeping the other running
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, LOW);
      digitalWrite(motor1E, HIGH);
      digitalWrite(motor2A, HIGH);
      digitalWrite(motor2B, LOW);
      digitalWrite(motor2E, HIGH);
      break;
    case 'R':
      // Turn right
      // Implement right turn logic by turning off one motor while keeping the other running
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
      digitalWrite(motor1E, HIGH);
      digitalWrite(motor2A, LOW);
      digitalWrite(motor2B, LOW);
      digitalWrite(motor2E, HIGH);
      break;
    // Add more commands as needed
  }
}
