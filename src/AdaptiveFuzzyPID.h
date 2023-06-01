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

#define AdaptiveFuzzyPID_VERSION        1   // software version of this library

typedef struct {
  double deltaKp;
  double deltaKi;
  double deltaKp;
} GainRange;

typedef struct {
  double min;
  double max;
} Range;

class AdaptiveFuzzyPID
{
public:
  AdaptiveFuzzyPID();
  void begin();
  void update(double, double);
  void setKp(double, double);
  void setKi(double, double);
  void setKd(double, double);
private:
   double kp, ki, kd;
   double integral;
   double derivative;

   void addRule();
   void setMembershipFunction();

};

#endif