#include <ESP32Servo.h>
#include "ESC.h"
#include <PID_v1.h>

#define LED_PIN 2
#define IR_LEFT_PIN 34

#define ESC_LEFT_PIN 13
#define ESC_RIGHT_PIN 12

#define SPEED_MIN 1000
#define SPEED_MAX 1220 // Limit speed to 1220

const float A2 = 295992.240;
const float B2 = -1.334;

ESC myESC_L(ESC_LEFT_PIN, SPEED_MIN, SPEED_MAX, 500);
ESC myESC_R(ESC_RIGHT_PIN, SPEED_MIN, SPEED_MAX, 500);

Servo steering_right, steering_left;

// PID Variables
double input, output, setpoint = 150.0; // Setpoint at 150 cm
PID myPID(&input, &output, &setpoint, 2.0, 1.0, 1.0, DIRECT); // Tuned values: Kp, Ki, Kd

// Time tracking
unsigned long previousMillis = 0;
const unsigned long sensorInterval = 100;

// Safety variables
const float safety_distance = 150.0;
double sensorLeft = 0;
float disLeft = 40.0;

void setup() {
  Serial.begin(115200);

  // Attach servos
  steering_left.attach(16);
  steering_right.attach(17);
  steering_left.write(90);
  steering_right.write(90);

  // Set up LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Set up IR sensor pin
  pinMode(IR_LEFT_PIN, INPUT);

  // Initialize ESCs
  myESC_L.arm();
  Serial.println("Left ESC armed.");
  delay(1000);
  myESC_R.arm();
  Serial.println("Right ESC armed.");
  delay(2000);

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(SPEED_MIN, SPEED_MAX); // Limit motor speed
}

void loop() {
  unsigned long currentMillis = millis();

  // Sensor reading and distance calculation
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;

    // Read sensor value
    sensorLeft = (analogRead(IR_LEFT_PIN) + sensorLeft) / 2;

    if (sensorLeft <= 0) {
      disLeft = 40.0; // Default to a minimum distance
    } else {
      disLeft = A2 * pow(sensorLeft, B2);
    }

    // Update PID input
    input = disLeft;

    // Debugging
    Serial.print("Sensor Value: ");
    Serial.print(sensorLeft);
    Serial.print(", Distance: ");
    Serial.println(disLeft);
  }

  // If distance is safe, directly set max speed
  if (disLeft > setpoint) {
    myESC_L.speed(SPEED_MAX);
    myESC_R.speed(SPEED_MAX);
    steering_left.write(90);
    steering_right.write(90);
    Serial.println("Distance safe, running at full speed.");
  } else {
    // Compute PID output
    myPID.Compute();

    // Apply PID-controlled speed
    myESC_L.speed(output);
    myESC_R.speed(output);
    steering_left.write(90);
    steering_right.write(90);
    Serial.print("Controlled Speed: ");
    Serial.println(output);
  }

  // Safety stop
  if (disLeft < safety_distance) {
    myESC_L.speed(SPEED_MIN);
    myESC_R.speed(SPEED_MIN);
    steering_left.write(150);
    steering_right.write(150);
    Serial.println("Object detected! Stopping.");

    output = 1000; // Handle output to stop motor
  }
}
