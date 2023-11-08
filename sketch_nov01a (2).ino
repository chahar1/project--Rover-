// Include necessary libraries
#include <NewPing.h>

// Define motor control pins
const int motor1A =10;
const int motor1B=9;
const int motor1E = 8;// Enable pin for Motor 1
const int motor2A =7;
const int motor2B =6;
const int motor2E = 5; // Enable pin for Motor 2

// HC-SR04 pin configuration
#define TRIG_PIN 52
#define ECHO_PIN 53

NewPing sonar(TRIG_PIN, ECHO_PIN);

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor1E, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motor2E, OUTPUT);

  // Set up serial communication
  Serial.begin(112500);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    executeCommand(command);
  }

  int distance = readDistance();
  Serial.println(distance);
  delay(100);
}

void executeCommand(char command) {
  // Implement motor control logic based on the received command
  switch (command) {
    case 'F':
      // Move both motors forward
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
      digitalWrite(motor1E, HIGH);
      digitalWrite(motor2A, HIGH);
      digitalWrite(motor2B, LOW);
      digitalWrite(motor2E, HIGH);
      break;
    case 'B':
      // Move both motors backward
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, HIGH);
      digitalWrite(motor1E, HIGH);
      digitalWrite(motor2A, LOW);
      digitalWrite(motor2B, HIGH);
      digitalWrite(motor2E, HIGH);
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

int readDistance() {
  // Read distance from HC-SR04 sensor
  unsigned int uS = sonar.ping();
  return uS / US_ROUNDTRIP_CM;
}
