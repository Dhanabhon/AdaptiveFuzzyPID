/*
          _   _
        ( (___) )
        [  x  x ]
         \     /
         ( ' ' )
           (U)
    Cult of the Dead Cow     
*/

#include <Arduino.h>

#include "AdaptiveFuzzyPID.h"

AdaptiveFuzzyPID::AdaptiveFuzzyPID() {
    this->sampleTimeInMs = 100;
    
    this->previousMillis = millis() - this->sampleTimeInMs;
}

long AdaptiveFuzzyPID::getSampleTime(void) {
    return this->sampleTimeInMs;
}

void AdaptiveFuzzyPID::setSampleTime(unsigned long sampleTime) {
    this->sampleTimeInMs = sampleTime;
}

void AdaptiveFuzzyPID::setMembershipFunctionInputType(MembershipFunctionType type) {
    this->currentMembershipFunctionInputType = type;
}

void AdaptiveFuzzyPID::setMembershipFunctionOutputType(MembershipFunctionType type) {
    this->currentMembershipFunctionOutputType = type;
}

void AdaptiveFuzzyPID::setMembershipFunctionInput(double start, double end) {
    double p1 = start;
    double p2 = (end - start) / 6 + start;
    double p3 = (end - start) / 6 + p2;
    double p4 = (end - start) / 6 + p3;
    double p5 = (end - start) / 6 + p4;
    double p6 = (end - start) / 6 + p5;
    double p7 = end;

    switch (this->currentMembershipFunctionInputType)
    {
        case Triangle:
        {
            TriangleParams NL, NM, NS, Z, PS, PM, PL;
            NL.a = p1, NL.b = p1, NL.c = p2;
            NM.a = p1, NM.b = p2, NM.c = p3;
            NS.a = p2, NL.b = p3, NL.c = p4;
            Z.a  = p3, NL.b = p4, NL.c = p5;
            PS.a = p4, NL.b = p5, NL.c = p6;
            PM.a = p5, NL.b = p6, NL.c = p7;
            PL.a = p6, NL.b = p7, NL.c = p7;
            // TODO:
        }
        break;
        case Trapezoid:
        {
            TriangleParams NL, NM, NS, Z, PS, PM, PL;

            // TODO:
        }    
        break;
        case BellShaped:
        {
            BellShapedParams NL, NM, NS, Z, PS, PM, PL;
            double width = (end - start) / 10.0;
            double slope = width * 10.0;
            NL.a = p1, NL.b = width, NL.c = slope;
            NM.a = p2, NM.b = width, NM.c = slope;
            NS.a = p3, NL.b = width, NL.c = slope;
            Z.a  = p4, NL.b = width, NL.c = slope;
            PS.a = p5, NL.b = width, NL.c = slope;
            PM.a = p6, NL.b = width, NL.c = slope;
            PL.a = p7, NL.b = width, NL.c = slope;
            //TODO:
        }
        break;
    default:
        break;
    }

    // addTerm(p1, p2, p3, p4, p5, p6, p7);
}

void AdaptiveFuzzyPID::addTerm(double points, ...) {
    va_list args;
    va_start(args, points);

    double p[7];

    for (auto i = 0; i < points; ++i) {
        p[i] = va_arg(args, double);
    }
    va_end(args);
}

void AdaptiveFuzzyPID::update(double setpoint, double input) {
    this->currentMillis = millis();

    if (this->currentMillis - this->previousMillis >= sampleTimeInMs) {
        this->previousMillis = this->currentMillis;

        // double error = setpoint - input;

    }
}