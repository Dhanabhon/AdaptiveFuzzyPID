/*
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

typedef struct {
  double deltaKp;
  double deltaKi;
  double deltaKp;
} GainRange;

typedef struct {
  double min;
  double max;
} Range;

enum MembershipFunctionType {
  Triangular = 0,
  Trapezoid = 1,
  Gaussian = 2
};

enum InferenceMode {
  MamdaniMaxMin = 0,
  MamdaniMaxProduct = 1,
  TSK = 2, // Takagi-Sugeno-Kang
  SAM = 3, // Standard Additive Model
};

class AdaptiveFuzzyPID
{
public:
  AdaptiveFuzzyPID();
  void update(double, double);

  void setKp(double, double);
  void setKi(double, double);
  void setKd(double, double);

  long getSampleTime(void);
  void setSampleTime(unsigned long);
  void setMembershipFunctionType(MembershipFunctionType);
private:
  double kp, ki, kd;
  double integral;
  double derivative;

  double *input;
  double *output;
  double *setpoint;

  unsigned long previousMillis;
  unsigned long currentMillis;
  unsigned long sampleTimeInMs;

  MembershipFunctionType currentMembershipFunctionType = MembershipFunctionType::Triangular;
  InferenceMode currentInferenceMode = InferenceMode::MamdaniMaxMin;

  void addRule();
  void addTerm();
  void setMembershipFunctionInput();
  void setMembershipFunctionOutput();
};

#endif