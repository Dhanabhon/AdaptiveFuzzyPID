/*
          _   _
        ( (___) )
        [  x  x ]
         \     /
         ( ' ' )
           (U)
    Cult of the Dead Cow  
    
  AdaptiveFuzzyPID.h - Adaptive Fuzzy PID (AFPID) is a powerful 
  Arduino library designed specifically to enhance the control 
  performance of speed for DC motors.- Version 2

  Copyright (c) 2023 Tom Dhanabhon.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef AdaptiveFuzzyPID_h
#define AdaptiveFuzzyPID_h

#define LIBRARY_VERSION     "0.0.1"

// typedef struct {
//   double NL;   // Negative Large
//   double NM;   // Negative Medium
//   double NS;   // Negative Small
//   double Z;    // Zero
//   double PS;   // Positive Small
//   double PM;   // Positive Medium
//   double PL;   // Positive Large
// } FuzzySet;

typedef struct {
  double a;
  double b;
  double c;
} TriangleParams;

typedef struct {
  double a;
  double b;
  double c;
  double d;
} TrapezoidParams;

typedef struct {
  double x;
  double ave;
  double sigma;
} GaussianParams;


typedef struct {
  double a;
  double b;
  double c;
} BellShapedParams;

enum MembershipFunctionType {
  Triangle = 0,
  Trapezoid = 1,
  Gaussian = 2,
  BellShaped = 3,
  SShaped = 4, // S-Shaped
  ZShaped = 5, // Z-Shaped
  Singleton = 6
};

enum InferenceMode {
  MamdaniMaxMin = 0,
  MamdaniMaxProduct = 1,
  TSK = 2, // Takagi-Sugeno-Kang
  SAM = 3  // Standard Additive Model
};

class AdaptiveFuzzyPID
{
public:
  AdaptiveFuzzyPID(double kp = 1.0, double ki = 0.0, double kd = 0.0);

  // Main control function
  double update(double setpoint, double input);

  // Configuration methods
  void setTunings(double kp, double ki, double kd);
  void setSampleTime(unsigned long sampleTimeMs);
  void setInputRange(double min, double max);
  void setOutputRange(double min, double max);
  void setMembershipFunctionInputType(MembershipFunctionType type);
  void setMembershipFunctionOutputType(MembershipFunctionType type);
  void setInferenceMode(InferenceMode mode);

  // Getter methods
  long getSampleTime(void);
  double getKp() const { return kp; }
  double getKi() const { return ki; }
  double getKd() const { return kd; }
private:
  // PID parameters
  double kp, ki, kd;
  double integral;
  double derivative;
  double lastError;

  // Input/Output ranges
  double inputMin, inputMax;
  double outputMin, outputMax;

  // Timing
  unsigned long previousMillis;
  unsigned long sampleTimeInMs;

  // Fuzzy system configuration
  MembershipFunctionType inputMFType;
  MembershipFunctionType outputMFType;
  InferenceMode inferenceMode;

  // Membership function parameters
  struct MembershipFunction {
    double params[4]; // Generic parameter array
    MembershipFunctionType type;
  };

  MembershipFunction errorMFs[7];    // Error membership functions
  MembershipFunction deltaErrorMFs[7]; // Delta error membership functions
  MembershipFunction outputMFs[7];   // Output membership functions

  // Fuzzy rule base (7x7 matrix)
  int ruleBase[7][7];

  // Private methods
  void initializeMembershipFunctions();
  void initializeRuleBase();
  double calculateMembership(double x, const MembershipFunction& mf);
  double triangleMF(double x, double a, double b, double c);
  double trapezoidMF(double x, double a, double b, double c, double d);
  double gaussianMF(double x, double center, double sigma);
  double bellShapedMF(double x, double a, double b, double c);
  double sShapedMF(double x, double a, double b);
  double zShapedMF(double x, double a, double b);
  double singletonMF(double x, double value);

  double fuzzify(double value, MembershipFunction* mfs);
  double defuzzify(double* membershipValues);
  double fuzzyInference(double error, double deltaError);

  // Fuzzy operators
  double fuzzyAND(double a, double b);
  double fuzzyOR(double a, double b);
  double fuzzyNOT(double a);
};

#endif