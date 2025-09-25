/*
  Basic Usage Example - Adaptive Fuzzy PID Library

  This example demonstrates how to use the AdaptiveFuzzyPID library
  for controlling a DC motor speed with adaptive fuzzy logic.

  Hardware connections:
  - Motor driver connected to pins 9 (PWM) and 7 (direction)
  - Encoder connected to pins 2 and 3 (interrupts)
  - Potentiometer connected to A0 (setpoint input)
*/

#include <AdaptiveFuzzyPID.h>

// Create an adaptive fuzzy PID controller with initial parameters
AdaptiveFuzzyPID controller(2.0, 1.0, 0.1);

// Motor control pins
const int motorPWM = 9;
const int motorDir = 7;

// Encoder pins
const int encoderA = 2;
const int encoderB = 3;

// Variables
volatile long encoderCount = 0;
double setpoint = 0;
double input = 0;
double output = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);

  // Initialize motor control pins
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

  // Initialize encoder pins
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);

  // Configure the fuzzy PID controller
  controller.setSampleTime(50);           // 50ms sample time
  controller.setInputRange(-1000, 1000);  // Error range in RPM
  controller.setOutputRange(-255, 255);   // PWM output range
  controller.setMembershipFunctionInputType(Triangle);
  controller.setMembershipFunctionOutputType(Triangle);
  controller.setInferenceMode(MamdaniMaxMin);

  Serial.println("Adaptive Fuzzy PID Motor Control Started");
  Serial.println("Setpoint | Input | Output | Kp | Ki | Kd");
}

void loop() {
  // Read setpoint from potentiometer (0-1023 mapped to 0-2000 RPM)
  setpoint = map(analogRead(A0), 0, 1023, 0, 2000);

  // Calculate current speed from encoder (simplified calculation)
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) { // Calculate every 100ms
    input = (encoderCount * 600.0) / (currentTime - lastTime); // RPM calculation
    encoderCount = 0; // Reset counter
    lastTime = currentTime;
  }

  // Update the controller
  output = controller.update(setpoint, input);

  // Apply output to motor
  if (output >= 0) {
    digitalWrite(motorDir, HIGH);
    analogWrite(motorPWM, constrain(output, 0, 255));
  } else {
    digitalWrite(motorDir, LOW);
    analogWrite(motorPWM, constrain(-output, 0, 255));
  }

  // Print debug information
  Serial.print(setpoint);
  Serial.print(" | ");
  Serial.print(input);
  Serial.print(" | ");
  Serial.print(output);
  Serial.print(" | ");
  Serial.print(controller.getKp());
  Serial.print(" | ");
  Serial.print(controller.getKi());
  Serial.print(" | ");
  Serial.println(controller.getKd());

  delay(10);
}

// Encoder interrupt service routine
void encoderISR() {
  if (digitalRead(encoderA) == digitalRead(encoderB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}