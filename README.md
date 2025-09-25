# Adaptive Fuzzy PID

Adaptive Fuzzy PID (AFPID) is a comprehensive Arduino library designed to enhance DC motor speed control performance through intelligent parameter adaptation. By combining traditional PID control with fuzzy logic techniques, AFPID dynamically adjusts control parameters in real-time based on system behavior and error characteristics.

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

**Well Documented**
- Comprehensive examples included
- Detailed API documentation
- Best practices and usage patterns

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

## License

This library is released under the GNU Lesser General Public License v2.1. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

## Author

Tom Dhanabhon - [dev@dhanabhon.com](mailto:dev@dhanabhon.com)

## Version History

- **v0.0.1** - Initial release with complete fuzzy PID implementation