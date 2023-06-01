
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

void AdaptiveFuzzyPID::setMembershipFunctionType(MembershipFunctionType type) {
    this->currentMembershipFunctionType = type;
}

void AdaptiveFuzzyPID::update(double setpoint, double input) {
    this->currentMillis = millis();

    if (this->currentMillis - this->previousMillis >= sampleTimeInMs) {
        this->previousMillis = this->currentMillis;

        double error = setpoint - input;

    }
}