#include <NewPing.h>

// Motor Driver L298N
const int motor1Pin1 = 9;
const int motor1Pin2 = 8;
const int motor2Pin1 = 7;
const int motor2Pin2 = 6;
const int enablePin1 = 10;
const int enablePin2 = 5;

// HC-SR04 Ultrasonic Sensor
const int trigPin = 52;
const int echoPin = 53;

NewPing sonar(trigPin, echoPin);

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Initialize the serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Measure distance using HC-SR04
  int distance = sonar.ping_cm();

  // Print the distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check for obstacles
  if (distance < 20) { // Change this value depending on your needs
    // If an obstacle is detected, turn right
    turnRight();
  } else {
    // If no obstacle is detected, move forward
    moveForward();
  }
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enablePin1, 255); // Adjust the speed as needed
  analogWrite(enablePin2, 255);
}

void turnRight() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enablePin1, 255); // Adjust the speed as needed
  analogWrite(enablePin2, 255);
}
