#include <Arduino_FreeRTOS.h>

#include <Arduino_OV767X.h>
#include <Adafruit_OV7670.h>
#include <image_ops.h>
#include <ov7670.h>
#include <SPIBrute.h>

#include <Adafruit_SSD1306.h>

memaling()

// Define motor control pins
const int enablePin1 = 10; // Enable pin for Motor 1
const int in1Pin = 9;    // Input 1 pin for Motor 1
const int in2Pin = 8;    // Input 2 pin for Motor 1
const int enablePin2 = 5; // Enable pin for Motor 2
const int in3Pin = 7;    // Input 1 pin for Motor 2
const int in4Pin = 6;    // Input 2 pin for Motor 2

//extra camera
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

const int vsyncPin = 9;
const int hrefPin = 3;
const int pclkPin = 2;

volatile bool frameFlag = false;
volatile bool lineFlag = false;

#define FRAME_WIDTH 120
#define FRAME_HEIGHT 80
#define PIXEL_BYTES 1

uint8_t frameBuffer[FRAME_WIDTH * FRAME_HEIGHT * PIXEL_BYTES];


void setup() {
  // Set the motor control pins as outputs
  pinMode(enablePin1, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  // Initialize the serial communication
  Serial.begin(9600);
  //camera module
  Serial.begin(115200);
  display.begin(SSD1306_I2C_ADDRESS, 20, 21); // Adjust I2C pins accordingly.
  display.display();
  display.clearDisplay();

  pinMode(vsyncPin, INPUT);
  pinMode(hrefPin, INPUT);
  pinMode(pclkPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(vsyncPin), onFrameStart, RISING);
  attachInterrupt(digitalPinToInterrupt(hrefPin), onLineStart, RISING);
}

void loop() {
  // Run the motors forward for 10 seconds
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  analogWrite(enablePin1, 255); // Motor 1 at full speed
  analogWrite(enablePin2, 255); // Motor 2 at full speed

  delay(10000); // 10 seconds

  // Stop the motors
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(enablePin1, 0); // Motor 1 stopped
  analogWrite(enablePin2, 0); // Motor 2 stopped

  delay(1000); // Delay for 1 second (optional)

  // Run the motors backward for 10 seconds
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enablePin1, 255); // Motor 1 at full speed
  analogWrite(enablePin2, 255); // Motor 2 at full speed

  delay(10000); // 10 seconds

  // Stop the motors
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
  analogWrite(enablePin1, 0); // Motor 1 stopped
  analogWrite(enablePin2, 0); // Motor 2 stopped

  delay(1000); // Delay for 1 second (optional)
  //camera module
  if (frameFlag) {
    frameFlag = false;
    displayFrame();
  }
}
//extra camera code
void onFrameStart() {
  noInterrupts();
  if (lineFlag) {
    lineFlag = false;
    frameFlag = true;
  }
  interrupts();
}

void onLineStart() {
  lineFlag = true;
}

void displayFrame() {
  display.clearDisplay();
  for (int x = 0; x < FRAME_WIDTH; x++) {
    for (int y = 0; y < FRAME_HEIGHT; y++) {
      uint16_t color = (frameBuffer[y * FRAME_WIDTH * PIXEL_BYTES + x * PIXEL_BYTES] << 8) | frameBuffer[y * FRAME_WIDTH * PIXEL_BYTES + x * PIXEL_BYTES + 1];
      display.drawPixel(x, y, color);
    }
  }
  display.display();
}

void captureFrame() {
  for (int y = 0; y < FRAME_HEIGHT; y++) {
    for (int x = 0; x < FRAME_WIDTH; x += 2) {
      while (digitalRead(pclkPin) == LOW) {}
      frameBuffer[y * FRAME_WIDTH * PIXEL_BYTES + x] = PIND;
      while (digitalRead(pclkPin) == HIGH) {}
      frameBuffer[y * FRAME_WIDTH * PIXEL_BYTES + x + 1] = PIND;
      while (digitalRead(pclkPin) == LOW) {}
      frameBuffer[y * FRAME_WIDTH * PIXEL_BYTES + x + 2] = PIND;
      while (digitalRead(pclkPin) == HIGH) {}
      frameBuffer[y * FRAME_WIDTH * PIXEL_BYTES + x + 3] = PIND;
    }
    while (digitalRead(hrefPin) == LOW) {}
    while (digitalRead(hrefPin) == HIGH) {}
  }
}
