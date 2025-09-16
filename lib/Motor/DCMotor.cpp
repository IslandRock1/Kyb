
#include <Arduino.h>
#include "DCMotor.hpp"

DCMotor::DCMotor(int pwmPin, int in1Pin, int in2Pin, int channel) {
    _pwmPin = pwmPin;
    _in1Pin = in1Pin;
    _in2Pin = in2Pin;
    _channel = channel;
}

void DCMotor::begin() {
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    ledcSetup(_channel, 5000, 8);
    ledcAttachPin(_pwmPin, _channel);
}

void DCMotor::move(int power) {
    // retning basert på fortegnet
    if (power > 0) {
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
    } else {
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
    }

    // hastighet basert på absoluttverdien
    ledcWrite(_channel, abs(power));
}