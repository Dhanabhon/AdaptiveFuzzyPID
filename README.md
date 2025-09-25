# Adaptive Fuzzy PID

Adaptive Fuzzy PID (AFPID) is a comprehensive Arduino library designed to enhance DC motor speed control performance through intelligent parameter adaptation. By combining traditional PID control with fuzzy logic techniques, AFPID dynamically adjusts control parameters in real-time based on system behavior and error characteristics.

## ⚠️ Development Status

This library is currently under active development. While the core functionality is implemented, some features may be incomplete or subject to change. Please use with caution in production environments.

## Features

**Complete Implementation**
- Fully functional adaptive fuzzy PID controller
- Multiple membership function types (Triangle, Trapezoid, Gaussian, Bell-shaped, S-shaped, Z-shaped, Singleton)
- Configurable inference modes (Mamdani Max-Min, Max-Product, TSK, SAM)
- Real-time parameter adaptation based on error and error rate

**Flexible Configuration**
- Adjustable sample time (1ms - 30s)
- Configurable input/output ranges
- Multiple membership function types for inputs and outputs
- Comprehensive parameter validation

**Performance Optimized**
- Non-blocking operation using Arduino's `millis()`
- Anti-windup protection for integral term
- Smooth parameter transitions
- Efficient fuzzy inference engine

## Quick Start

### Installation

1. Download the library as ZIP file
2. In Arduino IDE, go to **Sketch** → **Include Library** → **Add .ZIP Library**
3. Select the downloaded ZIP file
4. Include the library in your sketch:

```cpp
#include <AdaptiveFuzzyPID.h>
```

### Basic Usage

```cpp
#include <AdaptiveFuzzyPID.h>

// Create controller with initial PID parameters
AdaptiveFuzzyPID controller(2.0, 1.0, 0.1);

void setup() {
  // Configure controller
  controller.setSampleTime(50);           // 50ms sample time
  controller.setInputRange(-100, 100);    // Error range
  controller.setOutputRange(-255, 255);   // PWM output range

  // Set membership function types
  controller.setMembershipFunctionInputType(Triangle);
  controller.setMembershipFunctionOutputType(Triangle);
  controller.setInferenceMode(MamdaniMaxMin);
}

void loop() {
  double setpoint = 100.0;  // Desired speed
  double input = readCurrentSpeed(); // Your sensor reading

  // Update controller and get output
  double output = controller.update(setpoint, input);

  // Apply output to motor
  setMotorSpeed(output);

  delay(10);
}
```

## API Reference

### Constructor
```cpp
AdaptiveFuzzyPID(double kp = 1.0, double ki = 0.0, double kd = 0.0);
```

### Configuration Methods
```cpp
void setTunings(double kp, double ki, double kd);           // Set PID parameters
void setSampleTime(unsigned long sampleTimeMs);            // Set update interval
void setInputRange(double min, double max);                // Set error range
void setOutputRange(double min, double max);               // Set output range
void setMembershipFunctionInputType(MembershipFunctionType type);
void setMembershipFunctionOutputType(MembershipFunctionType type);
void setInferenceMode(InferenceMode mode);
```

### Control Method
```cpp
double update(double setpoint, double input);              // Main control function
```

### Getter Methods
```cpp
long getSampleTime();
double getKp();
double getKi();
double getKd();
```

### Enums

#### MembershipFunctionType
- `Triangle` - Triangular membership functions
- `Trapezoid` - Trapezoidal membership functions
- `Gaussian` - Gaussian (bell curve) membership functions
- `BellShaped` - Generalized bell-shaped membership functions
- `SShaped` - S-shaped membership functions
- `ZShaped` - Z-shaped membership functions
- `Singleton` - Singleton membership functions

#### InferenceMode
- `MamdaniMaxMin` - Mamdani inference with min/max operators
- `MamdaniMaxProduct` - Mamdani inference with product operator
- `TSK` - Takagi-Sugeno-Kang inference
- `SAM` - Standard Additive Model

## Examples

The library includes comprehensive examples:

1. **BasicUsage**: Simple motor control setup
2. **AdvancedConfiguration**: Multiple controllers with different configurations

## How It Works

The Adaptive Fuzzy PID controller operates in three main stages:

1. **Fuzzification**: Converts crisp error and error-rate values into fuzzy sets using membership functions
2. **Inference**: Applies fuzzy rules to determine control adjustments based on linguistic variables (Negative Large, Negative Medium, etc.)
3. **Defuzzification**: Converts fuzzy output back to crisp values for PID parameter adjustment

The controller continuously adapts its parameters based on system performance, providing better control than traditional fixed-parameter PID controllers.

## Applications

- DC motor speed control
- Servo positioning systems
- Temperature control
- Process control applications
- Robotic systems
- Automation projects

## Recommended Hardware for Testing

### Arduino Boards
- **Arduino Uno R3/R4** - Most popular and well-documented
- **Arduino Nano** - Compact version, pin-compatible with Uno
- **ESP32** - For advanced applications with WiFi/Bluetooth
- **Arduino Mega 2560** - For complex projects requiring more I/O

### DC Motors & Drivers
- **12V DC Gear Motors** (100-300 RPM) - Ideal for testing speed control
- **L298N Motor Driver Module** - Dual H-bridge, handles up to 2A per channel
- **DRV8833 Motor Driver** - Low voltage, suitable for smaller motors
- **TB6612FNG Motor Driver** - Higher efficiency alternative to L298N

### Speed Feedback Sensors
- **Rotary Encoders** (600-1000 PPR) - For precise speed measurement
- **Hall Effect Sensors** - Simple speed detection
- **IR Speed Sensors** - Non-contact speed measurement

### Power Supply & Accessories
- **12V 2A Power Supply** - For motor power
- **Breadboards/Jumper Wires** - For prototyping
- **Oscilloscope/Logic Analyzer** - For debugging and tuning (optional)

### Development Kits
- **Arduino Motor Shield R3** - Integrated solution
- **Robot Car Kits** - Complete testing platform
- **DC Motor Control Starter Kits** - All-in-one learning packages

## Hardware Connections

### Basic Setup: Arduino Uno + L298N + DC Motor + Encoder

#### L298N Motor Driver Connections
```
L298N Pin    →  Arduino Pin
ENA          →  Pin 9 (PWM)
IN1          →  Pin 8
IN2          →  Pin 7
IN3          →  Pin 6
IN4          →  Pin 5
ENB          →  Pin 3 (PWM)
+12V         →  External 12V Power Supply (+)
GND          →  Arduino GND + Power Supply (-)
+5V          →  Arduino 5V (if no external logic power)
```

#### DC Motor Connections
```
L298N OUT1   →  DC Motor Terminal 1
L298N OUT2   →  DC Motor Terminal 2
```

#### Rotary Encoder Connections (for speed feedback)
```
Encoder Pin  →  Arduino Pin
VCC          →  5V
GND          →  GND
Channel A    →  Pin 2 (Interrupt pin)
Channel B    →  Pin 4 (Digital pin)
```

### Alternative Setup: Arduino Nano + TB6612FNG

#### TB6612FNG Motor Driver Connections
```
TB6612FNG Pin →  Arduino Nano Pin
PWMA         →  Pin 9 (PWM)
AIN1         →  Pin 8
AIN2         →  Pin 7
STBY         →  Pin 6
VM           →  External Power Supply (+) [6-15V]
VCC          →  5V
GND          →  GND + Power Supply (-)
```

#### Motor and Encoder (same as above)
```
Motor: AO1 and AO2 to DC Motor terminals
Encoder: Same connections as basic setup
```

### ESP32 Setup (Advanced)

#### ESP32 Pin Assignments
```
Function     →  ESP32 Pin
Motor PWM    →  GPIO 25 (PWM Channel)
Motor DIR1   →  GPIO 26
Motor DIR2   →  GPIO 27
Encoder A    →  GPIO 4 (Interrupt capable)
Encoder B    →  GPIO 16
Power        →  3.3V/GND
```

### Important Notes
- Always connect motor driver GND to Arduino GND
- Use external power supply for motors (don't power from Arduino)
- Add 0.1µF capacitors across motor terminals to reduce noise
- Use pull-up resistors (10kΩ) on encoder channels if not built-in
- Connect STBY pin on TB6612FNG to HIGH (3.3V/5V) or control pin

## Testing Instructions

### 1. Hardware Verification Test

Before using the Adaptive Fuzzy PID library, verify your hardware setup:

#### Basic Motor Driver Test
```cpp
// Simple motor driver test (without PID)
void setup() {
  pinMode(8, OUTPUT); // IN1
  pinMode(7, OUTPUT); // IN2
  pinMode(9, OUTPUT); // ENA (PWM)
}

void loop() {
  // Forward rotation
  digitalWrite(8, HIGH);
  digitalWrite(7, LOW);
  analogWrite(9, 150); // 50% speed
  delay(2000);

  // Stop
  analogWrite(9, 0);
  delay(1000);

  // Reverse rotation
  digitalWrite(8, LOW);
  digitalWrite(7, HIGH);
  analogWrite(9, 150);
  delay(2000);

  // Stop
  analogWrite(9, 0);
  delay(1000);
}
```

#### Encoder Reading Test
```cpp
volatile int pulseCount = 0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP); // Encoder Channel A
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
}

void encoderISR() {
  pulseCount++;
}

void loop() {
  Serial.print("Encoder pulses: ");
  Serial.println(pulseCount);
  delay(500);
}
```

### 2. Basic PID Controller Test

Use the BasicUsage example to test the library:

```cpp
#include <AdaptiveFuzzyPID.h>

AdaptiveFuzzyPID controller(2.0, 1.0, 0.1);
volatile int encoderCount = 0;
unsigned long lastTime = 0;
double currentSpeed = 0;

void setup() {
  Serial.begin(9600);

  // Configure controller
  controller.setSampleTime(50);
  controller.setInputRange(-100, 100);
  controller.setOutputRange(-255, 255);

  // Setup encoder interrupt
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);

  // Motor driver pins
  pinMode(8, OUTPUT); // IN1
  pinMode(7, OUTPUT); // IN2
  pinMode(9, OUTPUT); // ENA

  lastTime = millis();
}

void encoderISR() {
  encoderCount++;
}

void loop() {
  // Calculate speed (RPM) every 100ms
  if (millis() - lastTime >= 100) {
    currentSpeed = (encoderCount * 600.0) / 1000.0; // Adjust for your encoder PPR
    encoderCount = 0;
    lastTime = millis();
  }

  double setpoint = 50.0; // Target 50 RPM
  double output = controller.update(setpoint, currentSpeed);

  // Apply output to motor
  if (output > 0) {
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    analogWrite(9, abs(output));
  } else if (output < 0) {
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
    analogWrite(9, abs(output));
  } else {
    analogWrite(9, 0);
  }

  // Debug output
  Serial.print("Setpoint: "); Serial.print(setpoint);
  Serial.print(", Speed: "); Serial.print(currentSpeed);
  Serial.print(", Output: "); Serial.println(output);

  delay(10);
}
```

### 3. Performance Testing

#### Step Response Test
- Set different target speeds (25, 50, 75, 100 RPM)
- Measure rise time, overshoot, and settling time
- Good performance: <2% overshoot, settling time <3 seconds

#### Disturbance Rejection Test
- Apply manual load to the motor shaft during operation
- Observe how quickly the controller recovers target speed
- Expected: Recovery within 1-2 seconds

#### Parameter Tuning Test
```cpp
// Try different PID values and compare performance
controller.setTunings(1.0, 0.5, 0.05); // Conservative
controller.setTunings(3.0, 1.5, 0.2);  // Aggressive
controller.setTunings(2.0, 1.0, 0.1);  // Balanced (recommended)
```

### 4. Expected Results

#### Good Performance Indicators:
- ✅ Stable speed control with minimal oscillation
- ✅ Quick response to setpoint changes (<3 seconds)
- ✅ Good disturbance rejection
- ✅ No excessive overshoot (<5%)

#### Troubleshooting Common Issues:

**Motor doesn't rotate:**
- Check power supply connections
- Verify motor driver wiring
- Test motor driver independently

**Oscillating/unstable control:**
- Reduce Kp value
- Check encoder connections
- Verify sample time settings

**Slow response:**
- Increase Kp value
- Check if sample time is too large
- Verify setpoint and input ranges

**Noisy encoder readings:**
- Add capacitors across motor terminals
- Use shielded encoder cables
- Check pull-up resistors on encoder channels

### 5. Data Logging for Analysis

```cpp
// Add to your main loop for performance analysis
if (millis() % 100 == 0) { // Log every 100ms
  Serial.print(millis()); Serial.print(",");
  Serial.print(setpoint); Serial.print(",");
  Serial.print(currentSpeed); Serial.print(",");
  Serial.print(output); Serial.print(",");
  Serial.print(controller.getKp()); Serial.println();
}
```

Plot this data in Excel/Python to visualize controller performance and tune parameters.

## License

This library is released under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## Author

Tom Dhanabhon - [dev@dhanabhon.com](mailto:dev@dhanabhon.com)

## Version History

- **v0.0.1** - Initial release with complete fuzzy PID implementation