/*
          _   _
        ( (___) )
        [  x  x ]
         \     /
         ( ' ' )
           (U)
    Cult of the Dead Cow  
    
  AdaptiveFuzzyPID.cpp - Implementation of Adaptive Fuzzy PID Controller

  Copyright (c) 2023 Tom Dhanabhon.  All right reserved.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <Arduino.h>
#include <math.h>
#include "AdaptiveFuzzyPID.h"

AdaptiveFuzzyPID::AdaptiveFuzzyPID(double kp, double ki, double kd) {
    // Initialize PID parameters
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    // Initialize control variables
    this->integral = 0.0;
    this->derivative = 0.0;
    this->lastError = 0.0;

    // Set default ranges
    this->inputMin = -100.0;
    this->inputMax = 100.0;
    this->outputMin = -255.0;
    this->outputMax = 255.0;

    // Set default sample time
    this->sampleTimeInMs = 100;
    this->previousMillis = millis() - this->sampleTimeInMs;

    // Set default fuzzy system configuration
    this->inputMFType = Triangle;
    this->outputMFType = Triangle;
    this->inferenceMode = MamdaniMaxMin;

    // Initialize fuzzy system
    initializeMembershipFunctions();
    initializeRuleBase();
}

long AdaptiveFuzzyPID::getSampleTime(void) {
    return this->sampleTimeInMs;
}

void AdaptiveFuzzyPID::setSampleTime(unsigned long sampleTimeMs) {
    // Validate and constrain sample time
    if (sampleTimeMs < 1) {
        sampleTimeMs = 1;
    } else if (sampleTimeMs > 30000) { // Max 30 seconds
        sampleTimeMs = 30000;
    }
    this->sampleTimeInMs = sampleTimeMs;
}

void AdaptiveFuzzyPID::setTunings(double kp, double ki, double kd) {
    // Validate PID parameters
    if (kp < 0) kp = 0;
    if (ki < 0) ki = 0;
    if (kd < 0) kd = 0;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void AdaptiveFuzzyPID::setInputRange(double min, double max) {
    // Validate input range
    if (max <= min) {
        // Swap if necessary
        double temp = max;
        max = min;
        min = temp;
    }

    this->inputMin = min;
    this->inputMax = max;
    initializeMembershipFunctions();
}

void AdaptiveFuzzyPID::setOutputRange(double min, double max) {
    // Validate output range
    if (max <= min) {
        // Swap if necessary
        double temp = max;
        max = min;
        min = temp;
    }

    this->outputMin = min;
    this->outputMax = max;
    initializeMembershipFunctions();
}

void AdaptiveFuzzyPID::setMembershipFunctionInputType(MembershipFunctionType type) {
    this->inputMFType = type;
    initializeMembershipFunctions();
}

void AdaptiveFuzzyPID::setMembershipFunctionOutputType(MembershipFunctionType type) {
    this->outputMFType = type;
    initializeMembershipFunctions();
}

void AdaptiveFuzzyPID::setInferenceMode(InferenceMode mode) {
    this->inferenceMode = mode;
}

void AdaptiveFuzzyPID::initializeMembershipFunctions() {
    double range = inputMax - inputMin;
    double step = range / 6.0;

    // Define 7 points across the input range
    double points[7];
    for (int i = 0; i < 7; i++) {
        points[i] = inputMin + i * step;
    }

    // Initialize error membership functions (NL, NM, NS, Z, PS, PM, PL)
    for (int i = 0; i < 7; i++) {
        errorMFs[i].type = inputMFType;
        deltaErrorMFs[i].type = inputMFType;
        outputMFs[i].type = outputMFType;

        switch (inputMFType) {
            case Triangle:
                if (i == 0) { // NL
                    errorMFs[i].params[0] = points[0] - step;
                    errorMFs[i].params[1] = points[0];
                    errorMFs[i].params[2] = points[1];
                } else if (i == 6) { // PL
                    errorMFs[i].params[0] = points[5];
                    errorMFs[i].params[1] = points[6];
                    errorMFs[i].params[2] = points[6] + step;
                } else {
                    errorMFs[i].params[0] = points[i-1];
                    errorMFs[i].params[1] = points[i];
                    errorMFs[i].params[2] = points[i+1];
                }
                deltaErrorMFs[i] = errorMFs[i]; // Same for delta error
                break;

            case Gaussian:
                errorMFs[i].params[0] = points[i]; // center
                errorMFs[i].params[1] = step / 2.0; // sigma
                deltaErrorMFs[i] = errorMFs[i];
                break;
        }

        // Output membership functions (normalized to output range)
        double outputRange = outputMax - outputMin;
        double outputStep = outputRange / 6.0;
        double outputPoints[7];
        for (int j = 0; j < 7; j++) {
            outputPoints[j] = outputMin + j * outputStep;
        }

        switch (outputMFType) {
            case Triangle:
                if (i == 0) {
                    outputMFs[i].params[0] = outputPoints[0] - outputStep;
                    outputMFs[i].params[1] = outputPoints[0];
                    outputMFs[i].params[2] = outputPoints[1];
                } else if (i == 6) {
                    outputMFs[i].params[0] = outputPoints[5];
                    outputMFs[i].params[1] = outputPoints[6];
                    outputMFs[i].params[2] = outputPoints[6] + outputStep;
                } else {
                    outputMFs[i].params[0] = outputPoints[i-1];
                    outputMFs[i].params[1] = outputPoints[i];
                    outputMFs[i].params[2] = outputPoints[i+1];
                }
                break;

            case Singleton:
                outputMFs[i].params[0] = outputPoints[i];
                break;
        }
    }
}

double AdaptiveFuzzyPID::update(double setpoint, double input) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= sampleTimeInMs) {
        double timeChange = (double)(currentMillis - previousMillis) / 1000.0; // Convert to seconds
        previousMillis = currentMillis;

        // Calculate error and derivative
        double error = setpoint - input;
        derivative = (error - lastError) / timeChange;

        // Apply fuzzy logic to adapt PID parameters
        double fuzzyOutput = fuzzyInference(error, derivative);

        // Adaptive parameter adjustment
        double adaptiveKp = kp * (1.0 + 0.1 * fuzzyOutput);
        double adaptiveKi = ki * (1.0 + 0.1 * fuzzyOutput);
        double adaptiveKd = kd * (1.0 + 0.1 * fuzzyOutput);

        // PID calculation with adaptive parameters
        integral += error * timeChange;

        // Anti-windup: limit integral term
        double maxIntegral = outputMax / (adaptiveKi + 0.001);
        if (integral > maxIntegral) integral = maxIntegral;
        else if (integral < -maxIntegral) integral = -maxIntegral;

        double output = adaptiveKp * error + adaptiveKi * integral + adaptiveKd * derivative;

        // Constrain output to specified range
        if (output > outputMax) output = outputMax;
        else if (output < outputMin) output = outputMin;

        lastError = error;
        return output;
    }

    return 0.0; // No update needed
}

void AdaptiveFuzzyPID::initializeRuleBase() {
    // Fuzzy rule base: ruleBase[error][deltaError] = output
    // NL=-3, NM=-2, NS=-1, Z=0, PS=1, PM=2, PL=3 (mapped to array indices 0-6)
    int rules[7][7] = {
        {0, 0, 1, 1, 2, 3, 3}, // NL error
        {0, 1, 1, 2, 2, 3, 4}, // NM error
        {1, 1, 2, 2, 3, 4, 4}, // NS error
        {1, 2, 2, 3, 4, 4, 5}, // Z error
        {2, 2, 3, 4, 4, 5, 5}, // PS error
        {2, 3, 4, 4, 5, 5, 6}, // PM error
        {3, 3, 4, 5, 5, 6, 6}  // PL error
    };

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            ruleBase[i][j] = rules[i][j];
        }
    }
}

double AdaptiveFuzzyPID::calculateMembership(double x, const MembershipFunction& mf) {
    switch (mf.type) {
        case Triangle:
            return triangleMF(x, mf.params[0], mf.params[1], mf.params[2]);
        case Trapezoid:
            return trapezoidMF(x, mf.params[0], mf.params[1], mf.params[2], mf.params[3]);
        case Gaussian:
            return gaussianMF(x, mf.params[0], mf.params[1]);
        case BellShaped:
            return bellShapedMF(x, mf.params[0], mf.params[1], mf.params[2]);
        case Singleton:
            return singletonMF(x, mf.params[0]);
        default:
            return 0.0;
    }
}

double AdaptiveFuzzyPID::triangleMF(double x, double a, double b, double c) {
    if (x <= a || x >= c) return 0.0;
    if (x == b) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

double AdaptiveFuzzyPID::trapezoidMF(double x, double a, double b, double c, double d) {
    if (x <= a || x >= d) return 0.0;
    if (x >= b && x <= c) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

double AdaptiveFuzzyPID::gaussianMF(double x, double center, double sigma) {
    double diff = x - center;
    return exp(-(diff * diff) / (2.0 * sigma * sigma));
}

double AdaptiveFuzzyPID::bellShapedMF(double x, double a, double b, double c) {
    double temp = (x - c) / a;
    return 1.0 / (1.0 + pow(abs(temp), 2.0 * b));
}

double AdaptiveFuzzyPID::sShapedMF(double x, double a, double b) {
    if (x <= a) return 0.0;
    if (x >= b) return 1.0;
    double mid = (a + b) / 2.0;
    if (x <= mid) {
        double temp = (x - a) / (b - a);
        return 2.0 * temp * temp;
    }
    double temp = (x - b) / (b - a);
    return 1.0 - 2.0 * temp * temp;
}

double AdaptiveFuzzyPID::zShapedMF(double x, double a, double b) {
    return 1.0 - sShapedMF(x, a, b);
}

double AdaptiveFuzzyPID::singletonMF(double x, double value) {
    return (abs(x - value) < 0.001) ? 1.0 : 0.0;
}

double AdaptiveFuzzyPID::fuzzyInference(double error, double deltaError) {
    // Normalize inputs to [-3, 3] range for indexing
    double normalizedError = 6.0 * (error - inputMin) / (inputMax - inputMin) - 3.0;
    double normalizedDelta = 6.0 * (deltaError - inputMin) / (inputMax - inputMin) - 3.0;

    // Calculate membership values for all fuzzy sets
    double errorMemberships[7], deltaMemberships[7];
    for (int i = 0; i < 7; i++) {
        errorMemberships[i] = calculateMembership(normalizedError, errorMFs[i]);
        deltaMemberships[i] = calculateMembership(normalizedDelta, deltaErrorMFs[i]);
    }

    // Apply fuzzy rules and accumulate output
    double outputMemberships[7] = {0};

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            double ruleStrength = fuzzyAND(errorMemberships[i], deltaMemberships[j]);
            int outputIndex = ruleBase[i][j];
            outputMemberships[outputIndex] = fuzzyOR(outputMemberships[outputIndex], ruleStrength);
        }
    }

    // Defuzzification using centroid method
    return defuzzify(outputMemberships);
}

double AdaptiveFuzzyPID::defuzzify(double* membershipValues) {
    double numerator = 0.0, denominator = 0.0;

    for (int i = 0; i < 7; i++) {
        if (membershipValues[i] > 0) {
            double centroid = outputMFs[i].params[1]; // Use center point
            numerator += membershipValues[i] * centroid;
            denominator += membershipValues[i];
        }
    }

    return (denominator > 0.001) ? numerator / denominator : 0.0;
}

double AdaptiveFuzzyPID::fuzzyAND(double a, double b) {
    return (a < b) ? a : b; // Min operator
}

double AdaptiveFuzzyPID::fuzzyOR(double a, double b) {
    return (a > b) ? a : b; // Max operator
}

double AdaptiveFuzzyPID::fuzzyNOT(double a) {
    return 1.0 - a;
}