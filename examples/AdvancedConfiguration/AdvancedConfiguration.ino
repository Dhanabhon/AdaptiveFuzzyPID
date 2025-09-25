/*
  Advanced Configuration Example - Adaptive Fuzzy PID Library

  This example demonstrates advanced configuration options including:
  - Different membership function types
  - Multiple inference modes
  - Dynamic parameter adjustment
  - Serial communication for real-time monitoring
*/

#include <AdaptiveFuzzyPID.h>

// Create controllers with different configurations
AdaptiveFuzzyPID triangularController(1.5, 0.8, 0.05);
AdaptiveFuzzyPID gaussianController(2.0, 1.2, 0.1);

// Control variables
double setpoint = 100.0;
double simulatedPlant = 0.0; // Simulated system response
double noise = 0.0;
double triangularOutput = 0.0;
double gaussianOutput = 0.0;

unsigned long lastPrintTime = 0;
int testPhase = 0;
unsigned long phaseStartTime = 0;

void setup() {
  Serial.begin(115200);

  // Configure triangular membership function controller
  triangularController.setSampleTime(20);
  triangularController.setInputRange(-200, 200);
  triangularController.setOutputRange(-100, 100);
  triangularController.setMembershipFunctionInputType(Triangle);
  triangularController.setMembershipFunctionOutputType(Triangle);
  triangularController.setInferenceMode(MamdaniMaxMin);

  // Configure Gaussian membership function controller
  gaussianController.setSampleTime(20);
  gaussianController.setInputRange(-200, 200);
  gaussianController.setOutputRange(-100, 100);
  gaussianController.setMembershipFunctionInputType(Gaussian);
  gaussianController.setMembershipFunctionOutputType(Gaussian);
  gaussianController.setInferenceMode(MamdaniMaxProduct);

  Serial.println("Advanced Adaptive Fuzzy PID Configuration Test");
  Serial.println("Time | Phase | Setpoint | Plant | Triangular | Gaussian | Error");

  phaseStartTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  // Test different scenarios
  runTestScenarios(currentTime);

  // Add some realistic noise
  noise = random(-5, 6) * 0.1;
  double noisyPlant = simulatedPlant + noise;

  // Update both controllers
  triangularOutput = triangularController.update(setpoint, noisyPlant);
  gaussianOutput = gaussianController.update(setpoint, noisyPlant);

  // Simple plant simulation (first-order system)
  double averageOutput = (triangularOutput + gaussianOutput) / 2.0;
  simulatedPlant += (averageOutput - simulatedPlant) * 0.02; // Time constant simulation

  // Print results every 200ms
  if (currentTime - lastPrintTime >= 200) {
    double error = setpoint - noisyPlant;

    Serial.print(currentTime);
    Serial.print(" | ");
    Serial.print(testPhase);
    Serial.print(" | ");
    Serial.print(setpoint, 1);
    Serial.print(" | ");
    Serial.print(noisyPlant, 1);
    Serial.print(" | ");
    Serial.print(triangularOutput, 1);
    Serial.print(" | ");
    Serial.print(gaussianOutput, 1);
    Serial.print(" | ");
    Serial.println(error, 1);

    lastPrintTime = currentTime;
  }

  delay(5);
}

void runTestScenarios(unsigned long currentTime) {
  unsigned long phaseTime = currentTime - phaseStartTime;

  switch (testPhase) {
    case 0: // Step response test
      setpoint = 100.0;
      if (phaseTime > 5000) {
        testPhase++;
        phaseStartTime = currentTime;
        Serial.println("# Phase 1: Ramp input test");
      }
      break;

    case 1: // Ramp input test
      setpoint = 50.0 + (phaseTime * 0.01);
      if (phaseTime > 8000) {
        testPhase++;
        phaseStartTime = currentTime;
        Serial.println("# Phase 2: Sine wave tracking test");
      }
      break;

    case 2: // Sine wave tracking
      setpoint = 100.0 + 30.0 * sin(phaseTime * 0.001);
      if (phaseTime > 10000) {
        testPhase++;
        phaseStartTime = currentTime;
        Serial.println("# Phase 3: Step disturbance test");
      }
      break;

    case 3: // Step disturbance
      setpoint = 150.0;
      // Simulate external disturbance
      if (phaseTime > 2000 && phaseTime < 4000) {
        simulatedPlant -= 20.0 * 0.01; // Apply disturbance
      }
      if (phaseTime > 8000) {
        testPhase++;
        phaseStartTime = currentTime;
        Serial.println("# Phase 4: Parameter adaptation test");
      }
      break;

    case 4: // Parameter adaptation test
      setpoint = 80.0;
      // Change controller parameters dynamically
      if (phaseTime == 2000) {
        triangularController.setTunings(3.0, 1.5, 0.2);
        gaussianController.setTunings(2.5, 1.8, 0.15);
        Serial.println("# Parameters updated");
      }
      if (phaseTime > 10000) {
        testPhase = 0; // Restart test cycle
        phaseStartTime = currentTime;
        Serial.println("# Restarting test cycle");

        // Reset parameters
        triangularController.setTunings(1.5, 0.8, 0.05);
        gaussianController.setTunings(2.0, 1.2, 0.1);
      }
      break;
  }
}