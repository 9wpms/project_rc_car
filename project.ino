#include <ESP32Servo.h>
#include "ESC.h"

// Pin Definitions
#define LED_PIN 2
#define IR_LEFT_PIN 34
#define IR_RIGHT_PIN 21
#define ESC_LEFT_PIN 13
#define ESC_RIGHT_PIN 12

// Speed and Threshold Constants
#define SPEED_MIN 1000
#define SPEED_MAX 1500
#define SPEED_SAFE 1220
const float safety_distance = 150.0;

// Calibration Constants for IR Sensor
const float A2 = 295992.240;
const float B2 = -1.334;

// ESC and Servo Initialization
ESC myESC_L(ESC_LEFT_PIN, SPEED_MIN, SPEED_MAX, 500);
ESC myESC_R(ESC_RIGHT_PIN, SPEED_MIN, SPEED_MAX, 500);
Servo steering_right, steering_left;

// Global Variables
double sensorLeft = 0, sensorRight = 0;
float disLeft = 40.0, disRight = 40.0;
unsigned long previousSensorTime = 0, previousMotorTime = 0, previousDebugTime = 0;
const unsigned long sensorInterval = 100, motorInterval = 100, debugInterval = 500;

void setup() {
  Serial.begin(115200);

  // Initialize Steering Servos
  steering_left.attach(16);
  steering_right.attach(17);
  steering_left.write(90);  // Center position
  steering_right.write(90);

  // Set up LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Set up IR Sensors
  pinMode(IR_LEFT_PIN, INPUT);

  // Initialize ESCs
  myESC_L.arm();
  Serial.println("Left ESC armed.");
  delay(1000);
  myESC_R.arm();
  Serial.println("Right ESC armed.");
  delay(2000);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read and Process Sensor Data
  if (currentMillis - previousSensorTime >= sensorInterval) {
    previousSensorTime = currentMillis;

    // Uncomment if using right sensor
    // sensorRight = (analogRead(IR_RIGHT_PIN) + sensorRight) / 2;

    // Smooth the sensor values
    sensorLeft = (analogRead(IR_LEFT_PIN) + sensorLeft) / 2;

    // Uncomment if using right sensor
    // if (sensorRight < 0) {
    //   sensorRight = 0;
    //   disRight = 40.0;
    // } else {
    //   disRight = A1 * pow(sensorRight, B1);
    // }

    // Calculate distances
    if (sensorLeft < 0) {
      sensorLeft = 0;
      disLeft = 40.0;
    } else {
      disLeft = A2 * pow(sensorLeft, B2);
    }
  }

  // Motor Control Based on Distance
  if (currentMillis - previousMotorTime >= motorInterval) {
    previousMotorTime = currentMillis;

    if (disLeft > safety_distance) {
      myESC_L.speed(SPEED_SAFE);
      myESC_R.speed(SPEED_SAFE);
      steering_left.write(90);
      steering_right.write(90);
    } else if (disLeft <= safety_distance) {
      myESC_L.speed(SPEED_MIN);
      myESC_R.speed(SPEED_MIN);
      steering_left.write(150);
      steering_right.write(150);
    }
  }

  // Debugging Output
  if (currentMillis - previousDebugTime >= debugInterval) {
    previousDebugTime = currentMillis;

    Serial.print("Sensor Left Value: ");
    Serial.print(sensorLeft);
    Serial.print(", Distance Left: ");
    Serial.println(disLeft);

    // Uncomment if using right sensor
    // Serial.print("Sensor Right Value: ");
    // Serial.print(sensorRight);
    // Serial.print(", Distance Right: ");
    // Serial.println(disRight);
  }
}
